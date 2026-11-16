import tkinter as tk
import unittest
from control_panel import Panel,parse_wheel_profile
from test_teaching_panel import FakePort

def line(sid=1,value=1,dirty=0):
    return f'DIRDATA id={sid} value={value} verified=1 saved_value=1 saved_verified=1 saved_present=1 dirty={dirty} source=PROFILE'

class WheelProfileTests(unittest.TestCase):
    def test_parser_rejects_partial_or_invalid_profile(self):
        self.assertEqual(parse_wheel_profile(line(value=-1,dirty=1))['value'],-1)
        for text in ('DIRDATA id=1',line(5),line().replace('verified=1','verified=2'),line(value=0)):
            self.assertIsNone(parse_wheel_profile(text))

class WheelPanelTests(unittest.TestCase):
    def setUp(self):
        self.root=tk.Tk(); self.root.withdraw(); self.panel=Panel(self.root)
        self.fake=FakePort(); self.panel.port=self.fake; self.panel.master=True
        self.panel.wheel_save_supported=True
        for sid in range(1,5): self.panel.handle_line(line(sid))
    def tearDown(self):
        self.panel.port=None; self.root.destroy()
    def test_requires_supported_firmware(self):
        self.panel.wheel_save_supported=False; self.panel.wheel_action('SAVEWHEELS')
        self.assertEqual(self.fake.sent,[])
    def test_save_requires_applied_values_and_four_feedback_rows(self):
        self.panel.wheel_values[3].set('-1'); self.panel.wheel_action('SAVEWHEELS')
        self.assertEqual(self.fake.sent,[])
        self.panel.handle_line(line(3,-1,1)); self.panel.wheel_action('SAVEWHEELS')
        self.assertEqual(self.fake.sent,['SAVEWHEELS'])
        self.fake.sent.clear(); self.panel.wheel_profiles.pop(4); self.panel.wheel_action('SAVEWHEELS')
        self.assertEqual(self.fake.sent,['DIRSHOW'])
    def test_apply_uses_correct_wheel_and_sign(self):
        self.panel.wheel_values[4].set('-1'); self.panel.apply_wheel(4)
        self.assertEqual(self.fake.sent,['WDIR 4 -1'])
    def test_readback_preserves_draft_but_explicit_restore_replaces_it(self):
        self.panel.wheel_values[2].set('-1'); self.panel.handle_line(line(2))
        self.assertEqual(self.panel.wheel_values[2].get(),'-1')
        self.panel.wheel_action('LOADWHEELS')
        for sid in range(1,5): self.panel.handle_line(line(sid))
        self.assertEqual(self.panel.wheel_values[2].get(),'1')
        self.assertFalse(self.panel.wheel_loading)
        self.assertIn('已保存',self.panel.wheel_labels[2].get())
    def test_feature_handshake_requests_persisted_status(self):
        self.panel.wheel_save_supported=False; self.panel.handle_line('FEATURES WHEEL_SAVE_V1')
        self.assertTrue(self.panel.wheel_save_supported); self.assertEqual(self.fake.sent,['DIRSHOW'])

if __name__=='__main__': unittest.main()
