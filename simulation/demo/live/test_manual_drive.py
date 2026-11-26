from time import monotonic
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock

from manual_drive import ManualDrive


class ManualDriveTests(TestCase):
    def test_manual_drive_remains_available_when_tracking_is_lost(self):
        panel = ManualDrive.__new__(ManualDrive)
        panel.backend = SimpleNamespace(
            running=True, telemetry={'drive_ready': '1'},
            telemetry_received_monotonic_s=monotonic(),
            live_scan=SimpleNamespace(tracking_status=lambda: {
                'tracking_state': 'lost', 'message': 'Giữ bản đồ cuối',
            }),
            command_drive=Mock(), command_stop=Mock(),
        )
        panel.root, panel.status = Mock(), Mock()
        panel.direction, panel.closed, panel.driving = None, False, False
        panel.press('Up')
        panel.update()
        panel.backend.command_drive.assert_called_once_with(.10, 0.)
        panel.backend.telemetry_received_monotonic_s -= 1
        panel.update()
        panel.backend.command_stop.assert_called_once()
        self.assertIsNone(panel.direction)
