import unittest
from web_panel import Bridge

class Fake:
    in_waiting=0
    def __init__(self): self.sent=[]
    def write(self,b): self.sent.append(b.decode().strip())
    def read(self,n): return b''
    def close(self): pass
    def flush(self): pass
    def reset_input_buffer(self): pass

class WebTests(unittest.TestCase):
    def setUp(self):
        self.now=10; self.b=Bridge(lambda:self.now); self.p=Fake()
        self.b.port=self.p; self.b.owner='one'; self.b.master=True
        self.b.last_rx=self.b.browser_seen=10; self.b.last_hb=10
    def test_expired_browser_drive_stops_without_releasing_servos(self):
        self.b.drive('one',1,1,0); self.now=10.31; self.b.tick()
        self.assertIsNone(self.b.motion); self.assertEqual(self.p.sent[-1],'STOP')
        self.assertNotIn('RELAX',self.p.sent)
    def test_delayed_drive_cannot_override_stop(self):
        self.b.drive('one',1,1,0); self.b.command('one',3,'STOP')
        self.b.drive('one',2,1,0); self.assertIsNone(self.b.motion)
    def test_joint_command_does_not_cancel_held_drive(self):
        self.b.drive('one',1,1,0); self.b.command('one',2,'J 2 1')
        self.assertEqual(self.b.motion,'MOVE 1.0000 0.0000'); self.assertEqual(self.p.sent[-1],'J 2 1')
    def test_other_browser_and_out_of_range_rejected(self):
        with self.assertRaises(ValueError): self.b.drive('two',1,1,0)
        with self.assertRaises(ValueError): self.b.drive('one',1,12,1)
        with self.assertRaises(ValueError): self.b.drive('one',1,float('nan'),0)
    def test_browser_timeout_disarms_but_preserves_port(self):
        self.now=14; self.b.tick(); self.assertEqual(self.b.owner,'one')
        self.assertIn('DISARM',self.p.sent); self.assertIs(self.b.port,self.p)
        count=self.p.sent.count('DISARM'); self.now=14.1; self.b.tick()
        self.assertEqual(self.p.sent.count('DISARM'),count)
    def test_feedback_timeout_blocks_drive_without_closing_port(self):
        self.now=14; self.b.browser_seen=14; self.b.tick()
        self.assertFalse(self.b.master); self.assertIs(self.b.port,self.p)
        with self.assertRaises(ValueError): self.b.drive('one',1,1,0)
    def test_manual_drive_cannot_bypass_lease(self):
        with self.assertRaises(ValueError): self.b.command('one',1,'MOVE 1 0')
    def test_held_rotation_continues_until_release(self):
        for i in range(1,31):
            self.now=10+i*.1;self.b.browser_seen=self.b.last_rx=self.now
            self.b.drive('one',i,0,4);self.b.tick()
        self.assertEqual(self.p.sent.count('MOVE 0.0000 4.0000'),30)
        self.assertNotIn('STOP',self.p.sent);self.assertNotIn('DISARM',self.p.sent)
        self.b.stop();self.assertEqual(self.p.sent[-1],'STOP')
    def test_log_cursor_retains_events_beyond_old_300_line_window(self):
        for i in range(800): self.b.log(str(i))
        first=self.b.log_page(0);self.assertEqual(first['records'][0]['text'],'0')
        second=self.b.log_page(first['records'][-1]['id'])
        self.assertEqual(second['records'][0]['text'],'500')
        self.assertEqual(second['records'][-1]['text'],'799')
    def test_archive_contains_all_lines_even_after_transport_buffer_rolls(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parents[1]) as folder:
            self.b.archive=True;self.b.log_path=Path(folder)/'test.log'
            self.b.log_file=self.b.log_path.open('a',encoding='utf-8')
            try:
                for i in range(5010):self.b.log(str(i))
                self.assertTrue(self.b.log_page(0)['gap'])
                data=self.b.export_log().decode().splitlines()
                self.assertEqual(len(data),5010);self.assertEqual(data[0],'0');self.assertEqual(data[-1],'5009')
            finally:self.b.log_file.close()
    def test_auto_verify_only_once_when_controller_off(self):
        class Status(Fake):
            in_waiting=200
            def read(self,n): return b'FEATURES TEACHING_V1\nrole=SINGLE_CONTROLLER controller=1 state=0\n'
        p=Status();self.b.port=p;self.b.tick()
        self.assertEqual(p.sent,['DIRSHOW','LIMITS','TEACHSHOW'])
        self.b.tick();self.assertEqual(p.sent.count('LIMITS'),1)
        self.assertNotIn('ARM',p.sent);self.assertNotIn('RELAX',p.sent)
    def test_auto_verify_defers_while_wheels_running(self):
        self.b.features.add('TEACHING_V1'); self.b.wheel_off=False
        self.b.tick();self.assertEqual(self.p.sent,[])
    def test_explicit_disconnect_releases_and_verifies(self):
        class Ack(Fake):
            in_waiting=64
            def read(self,n): return b'all_servo_torque_off_verified=1\n'
        p=Ack(); self.b.port=p
        self.assertTrue(self.b.disconnect(release=True))
        self.assertEqual(p.sent,['STOP','RELAX']); self.assertIsNone(self.b.port)
    def test_internal_disconnect_keeps_servo_torque(self):
        self.b.disconnect(); self.assertEqual(self.p.sent,['STOP','DISARM'])
    def test_cached_enable_requires_updated_firmware_and_cancels_drive(self):
        with self.assertRaises(ValueError): self.b.command('one',1,'ENABLE')
        self.b.features.add('FOC_CACHE_V1'); self.b.drive('one',2,1,0)
        self.b.command('one',3,'ENABLE')
        self.assertIsNone(self.b.motion); self.assertEqual(self.p.sent[-2:],['STOP','ENABLE'])
    def test_pose_key_timeout_and_release_keep_wheel_lease(self):
        self.b.features.add('TEACHING_V1'); self.b.drive('one',1,1,0)
        self.b.command('one',2,'SHIFT FRONT 0.0075')
        self.now=10.36; self.b.motion_at=self.now; self.b.tick()
        self.assertIn('POSTURESTOP',self.p.sent); self.assertIsNotNone(self.b.motion)
        self.b.command('one',4,'POSTURESTOP'); self.b.command('one',3,'SHIFT FRONT 0.0075')
        self.assertEqual(self.b.pose_until,0)
    def test_pose_key_delta_is_bounded(self):
        self.b.features.add('TEACHING_V1')
        for c in ('SHIFT ALL 1','SHIFT WRONG 0.01','SHIFT ALL NAN'):
            with self.assertRaises(ValueError): self.b.command('one',1,c)
    def test_percent_posture_preserves_browser_drive_lease(self):
        self.b.features.add('TEACHING_V1')
        self.b.drive('one',1,1,0); self.b.command('one',2,'BLEND 0.75')
        self.assertEqual(self.b.motion,'MOVE 1.0000 0.0000')
        self.assertEqual(self.p.sent[-1],'BLEND 0.75')
        self.b.command('one',3,'BLEND FRONT 0.58')
        self.assertEqual(self.p.sent[-1],'BLEND FRONT 0.58')
        self.b.command('one',4,'BLEND REAR 0.25')
        self.assertEqual(self.p.sent[-1],'BLEND REAR 0.25')
        self.assertIsNotNone(self.b.motion)

if __name__=='__main__': unittest.main()
