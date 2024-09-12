import socket
from threading import Thread, Lock
import sys

sys.path.append('../')

from pythonLibsTCP.tcp_packer_unpacker import PackerAndUnpacker

class TCPServer:
    _port = 0
    _server_socket = None
    isAlive = False
    _remoteAddress = ""
    _remotePort = 53317
    _remoteSocket = None
    _dataParser = 0
    unpacker = PackerAndUnpacker()
    _client_sockets = []  # Список активных клиентских сокетов
    _lock = Lock()  # Мьютекс для синхронизации доступа к списку сокетов

    def __init__(self, port, callback):
        self._port = port
        self.callback = callback

    def StartServer(self):
        self.isAlive = True
        listener = Thread(target=self.TCPlistener, args=())
        listener.setDaemon(True)
        listener.start()

    def TCPlistener(self):
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.bind(('', self._port))
        self._server_socket.listen(5)
        print("[tcp_comm]: Server started on port {}".format(self._port))

        while self.isAlive:
            try:
                self._server_socket.settimeout(1)  # Timeout для accept
                connection, address = self._server_socket.accept()
                self._lock.acquire()
                self._client_sockets.append(connection)
                self._lock.release()
                print('[tcp_comm]: Incoming connection from {}'.format(address))
                # Создаем поток для клиента
                sd = Thread(target=self.ClientListener, args=(connection,))
                sd.setDaemon(True)
                sd.start()
            except socket.timeout:
                continue  # Игнорируем timeout и проверяем флаг isAlive
            except Exception as e:
                print("[tcp_comm]: Server listener error: {}".format(e))
                break

    def StopServer(self):
        self.isAlive = False
        self._lock.acquire()
        for sock in self._client_sockets:
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
        self._client_sockets = []
        self._lock.release()
        if self._server_socket:
            self._server_socket.close()
        print("[tcp_comm]: Server and all client connections have been closed.")

    def ClientListener(self, sock):
        while self.isAlive:
            try:
                buf = sock.recv(1024)
                if buf:
                    data = self.unpacker.unpack(buf)
                    self.callback(data)
            except Exception as e:
                print("[tcp_comm]: Client listener error: {}".format(e))
                break
        sock.close()
