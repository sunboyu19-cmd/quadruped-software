import time
import tkinter as tk
import unittest
from unittest.mock import patch
from control_panel import Panel,parse_teaching

LINE='TEACHDATA id=2 pos=2048 feedback=1 hw_min=0 hw_max=4095 hw_known=1 stand=2400 lie=1700 sw_min=1700 sw_max=2400 dir=1 state=VALID offset=1920 taught_offset=1920 near_zero=0'

class FakePort:
    in_waiting=0
    def __init__(self): self.sent=[]
    def write(self,data): self.sent.append(data.decode().strip()); return len(data)
    def read(self,n): return b''
    def close(self): pass

class TeachingParserTests(unittest.TestCase):
    def test_valid_joint_and_direction(self):
        d=parse_teaching(LINE); self.assertEqual(d['id'],2); self.assertEqual(d['stand'],2400); self.assertEqual(d['dir'],1)
    def test_wrap_state_preserved_instead_of_inventing_direction(self):
        d=parse_teaching(LINE.replace('stand=2400','stand=3990').replace('lie=1700','lie=90').replace('dir=1','dir=0').replace('state=VALID','state=WRAP'))
        self.assertEqual(d['state'],'WRAP'); self.assertEqual(d['dir'],0)
    def test_rejects_truncated_or_wrong_id(self):
        self.assertIsNone(parse_teaching('TEACHDATA id=2 state=VALID'))
        self.assertIsNone(parse_teaching(LINE.replace('id=2','id=9')))
        self.assertIsNone(parse_teaching(LINE.replace('stand=2400','stand=bad')))

class TeachingPanelTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk(); self.root.withdraw(); self.panel=Panel(self.root)
        self.fake=FakePort(); self.panel.port=self.fake; self.panel.master=True
        self.panel.teaching_supported=True; self.panel.received=time.monotonic()
    def tearDown(self):
        self.panel.port=None; self.root.destroy()
    def test_capture_requires_physical_fixed_confirmation(self):
        self.panel.cal_action('TEACH STAND',True); self.assertEqual(self.fake.sent,[])
        self.panel.fixed.set(True); self.panel.cal_action('TEACH STAND',True)
        self.assertEqual(self.fake.sent,['TEACH STAND'])
        self.panel.handle_line('TEACH captured STAND in RAM; no motion or EEPROM writes.')
        self.assertFalse(self.panel.fixed.get())
    def test_table_updates_from_firmware_and_marks_direction(self):
        self.panel.handle_line(LINE)
        row=self.panel.teach_table.item('2','values'); self.assertIn('2400',row); self.assertIn('＋1 到站立',row)
        self.assertEqual(self.panel.teach_records[2]['state'],'VALID')
    def test_old_firmware_cannot_receive_calibration_from_buttons(self):
        self.panel.teaching_supported=False; self.panel.fixed.set(True)
        self.panel.cal_action('SETMID 2 FIXED',True); self.assertEqual(self.fake.sent,[])
    def test_heartbeat_does_not_send_drive_or_keepalive(self):
        self.panel.tick(); self.assertEqual(self.fake.sent,['HB'])
    def test_feedback_timeout_stops_and_disarms_without_resuming_motion(self):
        self.panel.hold.press('F 0.5'); self.fake.sent.clear()
        self.panel.received=time.monotonic()-4; self.panel.tick()
        self.assertIn('STOP',self.fake.sent); self.assertIn('DISARM',self.fake.sent)
        self.assertNotIn('F 0.5',self.fake.sent); self.assertNotIn('HB',self.fake.sent)
        self.assertIsNone(self.panel.hold.command); self.assertFalse(self.panel.master)

    def test_quiet_report_heartbeat_keeps_connection_without_motion(self):
        class HeartbeatPort(FakePort):
            in_waiting=64
            def read(self,n): return b'HOST_ALIVE role=SINGLE_CONTROLLER link=1\n'
        self.panel.port=HeartbeatPort(); self.panel.received=100
        self.panel.last_hb=100; self.panel.snapshot_pending=True
        for now in (101,102,103,104,105):
            with patch('control_panel.time.monotonic',return_value=now): self.panel.tick()
        self.assertTrue(self.panel.master)
        self.assertEqual(self.panel.received,105)
        self.assertEqual(self.panel.port.sent,['HB']*5)
        self.assertTrue(self.panel.snapshot_pending)
        self.assertFalse(self.panel.snapshot_logging)
        self.assertNotIn('HOST_ALIVE',self.panel.log.get('1.0','end'))

    def test_cockpit_spin_and_arc_respect_wheel_limits(self):
        self.panel.rotation.set('1.5'); self.panel.cockpit_drive(0,1)
        self.assertEqual(self.fake.sent[-1],'MOVE 0.0000 1.5000')
        self.panel.hold.release(); self.panel.cruise.set('7')
        self.panel.cockpit_drive(1,-1)
        _,f,t=self.fake.sent[-1].split(); f,t=float(f),float(t)
        self.assertLessEqual(max(abs(f-t),abs(f+t)),12)
        self.assertGreater(f,0); self.assertLess(t,0)
        self.panel.hold.release(); self.fake.sent.clear()
        self.panel.rotation.set('5'); self.panel.cockpit_drive(0,1)
        self.assertEqual(self.fake.sent,[])

    def test_cockpit_cannot_move_without_master(self):
        self.panel.master=False; self.panel.cockpit_drive(1,0)
        self.assertEqual(self.fake.sent,[])

    def test_heartbeat_cannot_authenticate_unconfirmed_board(self):
        self.panel.master=False
        self.panel.handle_line('HOST_ALIVE role=SINGLE_CONTROLLER link=1')
        self.assertFalse(self.panel.master)

if __name__=='__main__': unittest.main()
