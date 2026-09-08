"""SSH installation and atomic ROS Master configuration for the fleet console."""
from datetime import datetime
import errno
import ipaddress
from pathlib import Path
import re
import shlex
import time
import uuid


def shell_path(path):
    if path.startswith('~/'):
        return '"$HOME"/' + shlex.quote(path[2:])
    return shlex.quote(path)


def run_remote(client, command, cancelled, timeout=1200, progress=None, input_data=None):
    channel = client.get_transport().open_session(timeout=5)
    if input_data is None:
        channel.get_pty()  # Closing the channel also hangs up its foreground installation.
    channel.set_combine_stderr(True)
    channel.settimeout(0.2)
    output = ''
    deadline = time.monotonic() + timeout
    reported = 0.0
    try:
        channel.exec_command('bash -c ' + shlex.quote(command))
        if input_data is not None:
            # sudo -S reads from stdin; no PTY means the password cannot be echoed.
            channel.sendall(input_data)
            channel.shutdown_write()
        while True:
            if cancelled.is_set():
                raise RuntimeError('操作已取消')
            if channel.recv_ready():
                output = (output + channel.recv(32768).decode('utf-8', errors='replace'))[-12000:]
            if channel.exit_status_ready() and not channel.recv_ready():
                code = channel.recv_exit_status()
                if code:
                    error = RuntimeError(output.strip()[-2500:] or 'SSH 命令退出：%s' % code)
                    error.returncode = code
                    raise error
                return output
            now = time.monotonic()
            if now > deadline:
                raise TimeoutError('SSH 操作超过 %s 秒' % timeout)
            if progress and now - reported > 3:
                lines = output.strip().splitlines()
                progress(lines[-1][-200:] if lines else '正在连接仓库…')
                reported = now
            cancelled.wait(0.05)
    finally:
        channel.close()


def grant_serial_permissions(client, password, cancelled):
    if '\n' in password or '\r' in password:
        raise ValueError('sudo 密码不能包含换行符')
    command = '''set -e
shopt -s nullglob
devices=(/dev/ttyCH343USB*)
if ((${#devices[@]} == 0)); then
    echo '未找到 /dev/ttyCH343USB* 设备'
    exit 1
fi
sudo -S -p '' -- chmod -R 777 -- "${devices[@]}"
stat -Lc '%n: %a' -- "${devices[@]}"
'''
    return run_remote(client, command, cancelled, timeout=30,
                      input_data=(password + '\n').encode('utf-8')).strip()


def sync_workspace(client, options, ip, robot_id, cancelled, progress=None):
    if not re.fullmatch(r'ugv(?:0|[1-9][0-9]*)', robot_id):
        raise ValueError('车辆编号必须是 ugv0、ugv1 等格式')
    ipaddress.IPv4Address(ip)
    ipaddress.IPv4Address(options['master_ip'])
    remote = '/tmp/formation-install-%s.sh' % uuid.uuid4().hex
    try:
        with client.open_sftp() as sftp:
            sftp.put(str(Path(__file__).resolve().parents[1] / 'deploy/install-robot.sh'), remote)
            sftp.chmod(remote, 0o600)
        command = ' '.join(['bash', shlex.quote(remote), shlex.quote(robot_id),
                            shell_path(options['workspace']), shlex.quote(options['repository']),
                            shlex.quote(ip), shlex.quote(options['master_ip'])])
        run_remote(client, command, cancelled, progress=progress)
        return read_robot_config(client)
    finally:
        try:
            with client.open_sftp() as sftp:
                sftp.remove(remote)
        except Exception:
            pass


def parse_robot_config(text):
    result = {}
    for line in text.splitlines():
        match = re.match(r'^\s*(?:export\s+)?(ROS_MASTER_URI|ROS_IP|UGV_ID|CAR_MODE|UWB_PORT|UGV_OFFSET_X|UGV_OFFSET_Y)=([^#]*)', line)
        if match:
            value = shlex.split(match.group(2))
            result[match.group(1)] = value[0] if value else ''
    return {'ros_master': result.get('ROS_MASTER_URI', ''),
            'robot_id': 'ugv' + result['UGV_ID'] if result.get('UGV_ID', '').isdigit() else '',
            'ros_ip': result.get('ROS_IP', ''),
            'uwb_port': result.get('UWB_PORT', ''),
            'offset_x': result.get('UGV_OFFSET_X', '-0.8'),
            'offset_y': result.get('UGV_OFFSET_Y', '0.8')}


def read_robot_config(client):
    with client.open_sftp() as sftp:
        path = sftp.normalize('.') + '/.config/formation/robot.env'
        try:
            with sftp.open(path) as stream:
                return parse_robot_config(stream.read().decode('utf-8'))
        except IOError as error:
            if error.errno != errno.ENOENT:
                raise
            return {'ros_master': '', 'robot_id': '', 'ros_ip': ''}


def replace_master(text, master_ip):
    uri = 'http://%s:11311' % ipaddress.IPv4Address(master_ip)
    return replace_setting(text, 'ROS_MASTER_URI', uri)


def robot_number(name):
    if not re.fullmatch(r'ugv(?:0|[1-9][0-9]*)', name):
        raise ValueError('车辆名称必须是 ugv0、ugv1 等格式；其他描述请填写备注')
    return int(name[3:])


def replace_setting(text, key, value):
    pattern = r'^\s*(?:export\s+)?' + re.escape(key) + r'\s*=.*$'
    declaration = 'export %s=%s\n' % (key, shlex.quote(str(value)))
    lines = text.splitlines(keepends=True)
    updated, found = [], False
    for line in lines:
        if re.match(pattern, line):
            if not found:
                updated.append(declaration)
                found = True
        else:
            updated.append(line)
    if not found:
        if updated and not updated[-1].endswith('\n'):
            updated.append('\n')
        updated.append(declaration)
    return ''.join(updated)


def update_master(client, master_ip):
    return update_config(client, {'ROS_MASTER_URI': 'http://%s:11311' % ipaddress.IPv4Address(master_ip)}, 'master')


def update_identity(client, name):
    return update_config(client, {'UGV_ID': str(robot_number(name))}, 'identity')


def update_config(client, values, reason='config'):
    allowed = {'ROS_MASTER_URI', 'ROS_IP', 'UGV_ID', 'UWB_PORT', 'CAR_MODE', 'UGV_OFFSET_X', 'UGV_OFFSET_Y'}
    if not set(values) <= allowed:
        raise ValueError('不支持的车端配置字段')
    with client.open_sftp() as sftp:
        path = sftp.normalize('.') + '/.config/formation/robot.env'
        with sftp.open(path) as stream:
            original = stream.read().decode('utf-8')
        updated = original
        for key, value in values.items():
            updated = replace_setting(updated, key, value)
        if updated == original:
            return read_robot_config(client)
        mode = sftp.stat(path).st_mode & 0o777
        suffix = datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8]
        backup, temporary = path + '.before-' + reason + '-' + suffix, path + '.tmp-' + suffix
        with sftp.open(backup, 'w') as stream:
            stream.write(original.encode('utf-8'))
        sftp.chmod(backup, mode)
        try:
            with sftp.open(temporary, 'w') as stream:
                stream.write(updated.encode('utf-8'))
            sftp.chmod(temporary, mode)
            # Detect edits made while the SSH write was in progress.
            with sftp.open(path) as stream:
                if stream.read().decode('utf-8') != original:
                    raise RuntimeError('车端配置同时被修改，请读取后重试')
            sftp.posix_rename(temporary, path)
        finally:
            try:
                sftp.remove(temporary)
            except IOError as error:
                if error.errno != errno.ENOENT:
                    raise
        return read_robot_config(client)
