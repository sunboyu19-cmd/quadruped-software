import unittest
from control_panel import HoldDrive

class HoldDriveTests(unittest.TestCase):
    def setUp(self):
        self.now=0.0; self.sent=[]
        self.drive=HoldDrive(self.sent.append,lambda:self.now)

    def test_does_not_send_motion_without_press(self):
        self.drive.tick(); self.assertEqual(self.sent,[])

    def test_renews_only_while_held_and_stops_on_release(self):
        self.drive.press('F 0.5'); self.assertEqual(self.sent,['F 0.5'])
        self.now=0.05; self.drive.tick(); self.assertEqual(len(self.sent),1)
        self.now=0.11; self.drive.tick(); self.assertEqual(self.sent[-1],'F 0.5')
        self.drive.release(); self.assertEqual(self.sent[-1],'STOP')
        n=len(self.sent); self.now=2; self.drive.tick(); self.assertEqual(len(self.sent),n)

    def test_repeated_release_does_not_restart_or_flood(self):
        self.drive.press('W 3 0.5'); self.drive.release(); self.drive.release()
        self.assertEqual(self.sent,['W 3 0.5','STOP'])

if __name__=='__main__': unittest.main()
