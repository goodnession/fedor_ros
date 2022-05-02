import socket


class TcpConnector:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False

    def __init__(self, host, port) -> None:
        self.host = host
        self.port = port
        self.connect()

    def connect(self) -> None:
        self.sock.connect((self.host, self.port))
        self.connected = True

    def disconnect(self) -> None:
        self.sock.close()
        self.connected = False

    def reconnect(self) -> None:
        self.disconnect()
        self.connect()

    def request(self, req):
        while True:
            try:
                self.sock.send((req + '\n').encode())
                # sleep(0.01)
                data = self.sock.recv(1024)
                if b'\xf0' in data:
                    return None
                elif b'\xf1' in data:
                    return data.replace(b'\xf1', b'').replace(b'\r\n', b'').decode()
                elif data == b'':
                    self.reconnect()
                else:
                    raise BaseException(b'Request error: ' + data)
            except (ConnectionResetError, ConnectionAbortedError):
                self.reconnect()
