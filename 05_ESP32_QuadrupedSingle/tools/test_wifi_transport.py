import socket,threading,time,unittest
from wifi_transport import WifiTransport
from web_panel import Bridge
from test_web_panel import Fake
from unittest.mock import patch

class WifiTests(unittest.TestCase):
    def test_tcp_receives_status_and_transmits_commands(self):
        server=socket.socket(); server.bind(('127.0.0.1',0)); server.listen(1); received=[]
        def peer():
            conn,_=server.accept()
            with conn:
                conn.settimeout(2); conn.sendall(b'role=SINGLE_CONTROLLER link=1\n'); received.append(conn.recv(1024))
        t=threading.Thread(target=peer);t.start()
        transport=WifiTransport('127.0.0.1',server.getsockname()[1])
        try:
            end=time.monotonic()+2
            while not transport.in_waiting and time.monotonic()<end: time.sleep(.01)
            self.assertIn(b'SINGLE_CONTROLLER',transport.read(1000))
            transport.write(b'STOP\n');t.join(2);self.assertEqual(received,[b'STOP\n'])
        finally:transport.close();server.close()
    def test_bridge_wifi_uses_same_preflight_and_protocol(self):
        fake=Fake(); fake.port='wifi:192.168.4.1:9000'
        b=Bridge()
        with patch('web_panel.WifiTransport',return_value=fake): b.connect('one','',mode='wifi')
        self.assertFalse(b.master);self.assertEqual(fake.sent,['H','REPORT ON','S'])
        with self.assertRaises(ValueError): b.drive('one',1,1,0)
    def test_single_wheel_lease_stops_and_validates(self):
        now=[10];b=Bridge(lambda:now[0]);p=Fake();b.port=p;b.owner='one';b.master=True
        b.browser_seen=b.last_rx=b.last_hb=10
        b.test_wheel('one',1,2,.5); self.assertEqual(p.sent[-1],'W 2 0.500')
        now[0]=10.31;b.tick();self.assertEqual(p.sent[-1],'STOP')
        with self.assertRaises(ValueError):b.test_wheel('one',2,2,2)
    def test_invalid_connection_mode_rejected(self):
        with self.assertRaises(ValueError):Bridge().connect('one','COM13','unknown')

if __name__=='__main__':unittest.main()
