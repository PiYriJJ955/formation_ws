#!/usr/bin/env python3
"""Exercise update refusal and build retry using temporary Git repositories."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(*args, cwd, ok=True):
    result = subprocess.run(args, cwd=str(cwd), text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if ok:
        assert result.returncode == 0, result.stdout
    return result


with tempfile.TemporaryDirectory(prefix='formation-update-') as directory:
    root = Path(directory)
    publisher = root / 'publisher'
    publisher.mkdir()
    run('git', 'init', '-q', cwd=publisher)
    run('git', 'symbolic-ref', 'HEAD', 'refs/heads/master', cwd=publisher)
    run('git', 'config', 'user.name', 'Test', cwd=publisher)
    run('git', 'config', 'user.email', 'test@example.invalid', cwd=publisher)
    (publisher / '.gitignore').write_text('.local/\n')
    (publisher / 'scripts').mkdir()
    shutil.copyfile(str(ROOT / 'scripts/update.sh'), str(publisher / 'scripts/update.sh'))
    (publisher / 'scripts/build.sh').write_text('#!/bin/sh\ntest ! -e .local/fail-build\n')
    (publisher / 'version').write_text('one\n')
    run('git', 'add', '.', cwd=publisher)
    run('git', 'commit', '-qm', 'one', cwd=publisher)
    run('git', 'clone', '-q', str(publisher), 'robot', cwd=root)
    robot = root / 'robot'

    run('bash', 'scripts/update.sh', cwd=robot)
    revision = run('git', 'rev-parse', 'HEAD', cwd=robot).stdout.strip()
    assert (robot / '.local/built-revision').read_text().strip() == revision

    (publisher / 'version').write_text('two\n')
    run('git', 'commit', '-qam', 'two', cwd=publisher)
    (robot / 'version').write_text('local edit\n')
    assert run('bash', 'scripts/update.sh', cwd=robot, ok=False).returncode != 0
    assert (robot / 'version').read_text() == 'local edit\n'
    (robot / 'version').write_text('one\n')

    process = subprocess.Popen(['bash', '-c', 'exec -a roslaunch sleep 30'])
    try:
        result = run('bash', 'scripts/update.sh', cwd=robot)
        assert 'deferred' in result.stdout, result.stdout
        assert run('git', 'rev-parse', 'HEAD', cwd=robot).stdout.strip() == revision
    finally:
        process.terminate()
        process.wait()

    (robot / '.local/fail-build').touch()
    assert run('bash', 'scripts/update.sh', cwd=robot, ok=False).returncode != 0
    assert (robot / '.local/built-revision').read_text().strip() == revision
    (robot / '.local/fail-build').unlink()
    run('bash', 'scripts/update.sh', cwd=robot)
    revision = run('git', 'rev-parse', 'HEAD', cwd=robot).stdout.strip()
    assert (robot / '.local/built-revision').read_text().strip() == revision

    run('git', 'checkout', '-qb', 'local-work', cwd=robot)
    assert run('bash', 'scripts/update.sh', cwd=robot, ok=False).returncode != 0
    assert run('git', 'branch', '--show-current', cwd=robot).stdout.strip() == 'local-work'
    run('git', 'checkout', '-q', 'master', cwd=robot)
    run('git', 'config', 'user.name', 'Test', cwd=robot)
    run('git', 'config', 'user.email', 'test@example.invalid', cwd=robot)
    (robot / 'version').write_text('local commit\n')
    run('git', 'commit', '-qam', 'local commit', cwd=robot)
    local_revision = run('git', 'rev-parse', 'HEAD', cwd=robot).stdout
    assert run('bash', 'scripts/update.sh', cwd=robot, ok=False).returncode != 0
    assert run('git', 'rev-parse', 'HEAD', cwd=robot).stdout == local_revision

print('Update checks passed: dirty files, running ROS, failed build retry, branches and local commits.')
