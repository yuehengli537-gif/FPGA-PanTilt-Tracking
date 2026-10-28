"""Serial discovery tests use synthetic port metadata, never Windows devices."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from .controller import ServoController
from .ports import describe_ports


def fake_port(device, vid=None, pid=None, description='Test port', hwid=''):
    return SimpleNamespace(device=device, vid=vid, pid=pid, description=description, hwid=hwid)


class PortDiscoveryTests(unittest.TestCase):
    def test_classifies_candidates_without_com_number_or_serial_number_assumption(self):
        items = describe_ports([
            fake_port('COM4', description='Bluetooth Port', hwid='BTHENUM\\TEST'),
            fake_port('COM29', 0x0403, 0x6010, 'USB Serial Port'),
            fake_port('COM9', 0x1A86, 0x7523, 'USB-SERIAL CH340'),
            fake_port('COM6', 0x0403, 0x6001, 'Other FTDI device'),
        ])
        self.assertEqual(items[0].device, 'COM29')
        self.assertEqual([p.device for p in items if p.board_candidate], ['COM29'])
        self.assertIn('候选', items[0].label)
        self.assertEqual([p.device for p in items if p.bluetooth], ['COM4'])
        self.assertIn('非开发板', items[-1].label)

    def test_controller_enumerates_once_and_returns_metadata(self):
        with patch('serial.tools.list_ports.comports', return_value=[
            fake_port('COM22', 0x0403, 0x6010),
            fake_port('COM3', description='蓝牙链接上的标准串行', hwid='BTHENUM\\LOCAL'),
        ]) as enumerate_ports:
            items = ServoController.ports()
        enumerate_ports.assert_called_once_with()
        self.assertTrue(items[0].board_candidate)
        self.assertTrue(items[1].bluetooth)

    def test_generic_usb_is_not_claimed_as_board(self):
        items = describe_ports([fake_port('COM55', 0x1234, 0x5678), fake_port('COM1')])
        self.assertFalse(any(p.board_candidate for p in items))
        self.assertIn('其他 USB', next(p.label for p in items if p.device == 'COM55'))
        self.assertEqual(describe_ports([]), [])


if __name__ == '__main__':
    unittest.main()