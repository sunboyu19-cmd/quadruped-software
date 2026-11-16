"""Serial-like interface for the ESP32 AP TCP console; no motion policy here."""
import socket,select,ipaddress
class WifiTransport:
    def __init__(self,host,port=9000):
        address=ipaddress.IPv4Address(host)
        if not address.is_private: raise ValueError('WiFi目标应为机器人本地IPv4地址。')
        self.port=f'wifi:{host}:{port}'; self.buffer=bytearray()
        self.sock=socket.create_connection((host,port),timeout=3)
        self.sock.settimeout(.15); self.sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    @property
    def in_waiting(self):
        if select.select([self.sock],[],[],0)[0]:
            data=self.sock.recv(4096)
            if not data: raise ConnectionError('WiFi连接已关闭')
            self.buffer.extend(data)
        return len(self.buffer)
    def read(self,n):
        data=bytes(self.buffer[:n]); del self.buffer[:n]; return data
    def write(self,data): self.sock.sendall(data); return len(data)
    def flush(self): pass
    def reset_input_buffer(self):
        self.buffer.clear()
        for _ in range(16):
            if not select.select([self.sock],[],[],0)[0]: break
            if not self.sock.recv(4096): break
    def close(self): self.sock.close()
