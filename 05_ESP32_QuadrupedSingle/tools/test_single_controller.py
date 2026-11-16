import unittest
from web_panel import Bridge
from test_web_panel import Fake


class ReceivePort(Fake):
    def __init__(self, text):
        super().__init__()
        self.data = text.encode()

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, n):
        data, self.data = self.data[:n], self.data[n:]
        return data


class SingleControllerTests(unittest.TestCase):
    def bridge(self, text):
        b = Bridge(lambda: 10)
        b.port = ReceivePort(text)
        b.owner = 'one'
        b.last_rx = b.browser_seen = b.last_hb = 10
        return b

    def test_single_off_status_verifies_teaching_without_peer_fields(self):
        b = self.bridge('FEATURES TEACHING_V1\nrole=SINGLE_CONTROLLER controller=1 epoch=1 state=0 fault=0 loop_hz=320\n')
        b.tick()
        self.assertTrue(b.master)
        self.assertTrue(b.wheel_off)
        self.assertEqual(b.port.sent, ['DIRSHOW', 'LIMITS', 'TEACHSHOW'])
        self.assertNotIn('ARM', b.port.sent)

    def test_old_dual_board_firmware_cannot_be_controlled(self):
        b = self.bridge('role=FRONT_MASTER link=1 state=0 peer_state=0\n')
        b.tick()
        self.assertFalse(b.master)
        with self.assertRaises(ValueError):
            b.drive('one', 1, 1, 0)

    def test_single_running_status_does_not_read_hardware_limits(self):
        b = self.bridge('FEATURES TEACHING_V1\nrole=SINGLE_CONTROLLER controller=1 epoch=2 state=3 fault=0 loop_hz=300\n')
        b.tick()
        self.assertTrue(b.master)
        self.assertFalse(b.wheel_off)
        self.assertNotIn('LIMITS', b.port.sent)
        b.test_wheel('one', 1, 4, 0.5)
        self.assertEqual(b.port.sent[-1], 'W 4 0.500')


if __name__ == '__main__':
    unittest.main()
