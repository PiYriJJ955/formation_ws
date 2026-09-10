#!/usr/bin/env python3
"""Check sync bootstrap uploads without SSH or vehicle access."""
from pathlib import Path
import shlex
import sys
import threading
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_deploy import sync_workspace, ensure_mpc_dependencies


class SyncBootstrapChecks(unittest.TestCase):
    def test_current_updater_is_passed_to_installer_and_cleaned_up(self):
        for failure in (None, RuntimeError('update failed')):
            with self.subTest(failure=failure):
                client = MagicMock()
                sftp = client.open_sftp.return_value.__enter__.return_value
                options = dict(workspace='/home/robot/formation ws',
                               repository='http://192.0.2.1:8000/formation.git', master_ip='192.0.2.1')
                with patch('fleet_deploy.ensure_mpc_dependencies') as dependencies, \
                        patch('fleet_deploy.run_remote', side_effect=failure) as run, \
                        patch('fleet_deploy.read_robot_config', return_value={'robot_id': 'ugv2'}):
                    if failure:
                        with self.assertRaisesRegex(RuntimeError, 'update failed'):
                            sync_workspace(client, options, '192.0.2.2', 'ugv2', threading.Event())
                    else:
                        self.assertEqual(sync_workspace(client, options, '192.0.2.2', 'ugv2',
                                                        threading.Event()), {'robot_id': 'ugv2'})
                dependencies.assert_called_once()
                uploads = [call.args for call in sftp.put.call_args_list]
                self.assertEqual([Path(local).name for local, _ in uploads], ['install-robot.sh', 'update.sh'])
                installer, updater = [remote for _, remote in uploads]
                self.assertEqual(shlex.split(run.call_args.args[1]), [
                    'bash', installer, 'ugv2', options['workspace'], options['repository'],
                    '192.0.2.2', '192.0.2.1', updater])
                self.assertEqual([call.args for call in sftp.chmod.call_args_list],
                                 [(installer, 0o600), (updater, 0o600)])
                self.assertEqual([call.args[0] for call in sftp.remove.call_args_list], [installer, updater])



    def test_dependency_install_uses_stdin_and_cleans_uploaded_script(self):
        for failure in (None, RuntimeError('dependency install failed')):
            with self.subTest(failure=failure):
                client = MagicMock()
                sftp = client.open_sftp.return_value.__enter__.return_value
                options = {'password': "example secret with spaces"}
                with patch('fleet_deploy.run_remote', side_effect=failure) as run:
                    if failure:
                        with self.assertRaisesRegex(RuntimeError, 'dependency install failed'):
                            ensure_mpc_dependencies(client, options, threading.Event())
                    else:
                        ensure_mpc_dependencies(client, options, threading.Event())
                command = run.call_args.args[1]
                self.assertNotIn(options['password'], command)
                self.assertIn('--sudo-stdin', command)
                self.assertEqual(run.call_args.kwargs['input_data'], b'example secret with spaces\n')
                local, remote = sftp.put.call_args.args
                self.assertEqual(Path(local).name, 'install_mpc_dependencies.sh')
                sftp.chmod.assert_called_once_with(remote, 0o600)
                sftp.remove.assert_called_once_with(remote)

    def test_dependency_password_newline_is_rejected_before_upload(self):
        client = MagicMock()
        with self.assertRaises(ValueError):
            ensure_mpc_dependencies(client, {'password': 'first\nsecond'}, threading.Event())
        client.open_sftp.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
