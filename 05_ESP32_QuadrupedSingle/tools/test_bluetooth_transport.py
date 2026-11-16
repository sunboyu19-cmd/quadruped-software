import unittest
from unittest.mock import patch
from web_panel import Bridge
from test_web_panel import Fake


class FakeSerial(Fake):
    def __init__(self, data=b''):
        super().__init__()
        self.data=data
        self.opened=False
        self.closed=False

    @property
    def in_waiting(self):
        return len(self.data)

    def open(self):
        self.opened=True

    def close(self):
        self.closed=True

    def read(self,n):
        data,self.data=self.data[:n],self.data[n:]
        return data


class BluetoothTests(unittest.TestCase):
    def connect(self):
        b=Bridge(lambda:10)
        p=FakeSerial()
        with patch('web_panel.serial.Serial',return_value=p):
            b.connect('one','COM21',mode='bluetooth')
        return b,p

    def test_bluetooth_com_opens_without_asserting_usb_reset_lines(self):
        b,p=self.connect()
        self.assertEqual(b.transport,'bluetooth')
        self.assertEqual(p.port,'COM21')
        self.assertTrue(p.opened)
        self.assertFalse(p.dtr)
        self.assertFalse(p.rts)
        self.assertEqual(p.sent,['H','REPORT ON','S'])
        self.assertFalse(b.master)
        with self.assertRaises(ValueError):b.drive('one',1,1,0)

    def test_wrong_usb_port_in_bluetooth_mode_does_not_enable_control(self):
        b,p=self.connect()
        p.data=b'role=SINGLE_CONTROLLER controller=1 state=0 transport=serial\n'
        b.tick()
        self.assertFalse(b.master)
        self.assertFalse(b.wheel_off)
        self.assertTrue(any('传出COM' in s for s in b.lines))

    def test_bluetooth_com_in_usb_mode_does_not_mislabel_or_allow_control(self):
        b,p=self.connect();b.transport='serial'
        p.data=b'role=SINGLE_CONTROLLER state=0 transport=bluetooth\n'
        b.tick()
        self.assertFalse(b.master)

    def test_old_firmware_without_transport_identity_is_rejected_in_bt_mode(self):
        b,p=self.connect()
        p.data=b'role=SINGLE_CONTROLLER controller=1 state=0\n'
        b.tick()
        self.assertFalse(b.master)

    def test_bluetooth_status_uses_normal_preflight_and_movement_lease(self):
        now=[10]
        b,p=self.connect();b.clock=lambda:now[0]
        p.data=b'FEATURES TEACHING_V1\nFEATURES BLUETOOTH_SPP_V1\nrole=SINGLE_CONTROLLER state=0 transport=bluetooth\n'
        b.tick()
        self.assertTrue(b.master)
        self.assertIn('LIMITS',p.sent)
        b.drive('one',1,1,0)
        now[0]=10.31;b.tick()
        self.assertIsNone(b.motion)
        self.assertEqual(p.sent[-1],'STOP')

    def test_same_com_switching_usb_to_bt_reopens_and_clears_handshake(self):
        b,p=self.connect()
        b.transport='serial';b.master=True;b.features.add('FOC_CACHE_V1')
        second=FakeSerial()
        with patch('web_panel.serial.Serial',return_value=second):
            b.connect('one','COM21',mode='bluetooth')
        self.assertTrue(p.closed)
        self.assertIn('DISARM',p.sent)
        self.assertFalse(b.master)
        self.assertFalse(b.features)
        self.assertIs(b.port,second)

    def test_explicit_disconnect_keeps_existing_servo_release_policy(self):
        b,p=self.connect()
        p.data=b'all_servo_torque_off_verified=1\n'
        self.assertTrue(b.disconnect(release=True))
        self.assertEqual(p.sent[-2:],['STOP','RELAX'])
        self.assertTrue(p.closed)


if __name__=='__main__':unittest.main()
