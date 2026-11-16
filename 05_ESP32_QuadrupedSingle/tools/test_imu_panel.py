import unittest
from web_panel import Bridge
from test_web_panel import Fake


class ImuPanelTests(unittest.TestCase):
    def setUp(self):
        self.b=Bridge(lambda:10)
        self.b.port=Fake(); self.b.owner='one'; self.b.master=True
        self.b.last_rx=self.b.browser_seen=self.b.last_hb=10

    def test_old_firmware_cannot_receive_imu_commands(self):
        with self.assertRaises(ValueError): self.b.command('one',1,'IMU CAL',fixed=True)
        self.assertEqual(self.b.port.sent,[])

    def test_calibration_and_save_require_fixed_body_confirmation(self):
        self.b.features.add('IMU_V1')
        for c in ('IMU CAL','IMU SAVE'):
            with self.assertRaises(ValueError): self.b.command('one',1,c)
        self.assertEqual(self.b.port.sent,[])
        self.b.command('one',2,'IMU CAL',fixed=True)
        self.assertEqual(self.b.port.sent,['STOP','IMU CAL'])

    def test_read_does_not_cancel_held_wheel_drive(self):
        self.b.features.add('IMU_V1'); self.b.drive('one',1,1,0)
        self.b.command('one',2,'IMU READ')
        self.assertEqual(self.b.motion,'MOVE 1.0000 0.0000')
        self.assertEqual(self.b.port.sent[-1],'IMU READ')
        self.assertNotIn('STOP',self.b.port.sent)

    def test_cancel_does_not_implicitly_enable_or_release_actuators(self):
        self.b.features.add('IMU_V1'); self.b.command('one',1,'IMU CANCEL')
        self.assertEqual(self.b.port.sent,['IMU CANCEL'])

    def test_invalid_imu_commands_are_not_forwarded(self):
        self.b.features.add('IMU_V1')
        for c in ('IMU','IMU RESET','IMU CAL 1','IMU READ NOW'):
            with self.assertRaises(ValueError): self.b.command('one',1,c,fixed=True)
        self.assertEqual(self.b.port.sent,[])


if __name__=='__main__': unittest.main()
