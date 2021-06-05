import socket
import struct
from select import select
import json

class NetworkManager(object):
    def __init__(self, port):
        #Boot up the socket server, set in in blocking mode for select operations
        self._connections = []
        self._masterSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._masterSocket.bind(('0.0.0.0', port))
        self._masterSocket.setblocking(True)
        self._masterSocket.listen(3)
        self._connections.append(self._masterSocket)

        #Generate Signature Dictionary: Byte Key, Function Pointer
        self._signatureDictionary = {}

    def addCallback(self, id, callback):
        if id in self._signatureDictionary:
            return False
        else:
            self._signatureDictionary[id] = callback
            return True
    def removeCallback(self, id):
        if id in self._signatureDictionary:
            self._signatureDictionary.pop(id)
            return True
        else:
            return False
    
    def spinSocket(self):
        rs, _, _ = select(self._connections, [], [])
        for valid in rs:
            #Server received connect call
            if valid == self._masterSocket:
                socketDescriptor, _ = self._masterSocket.accept()
                self._connections.append(socketDescriptor)
                print("Client Added")

            #Server received procedure call
            else:
                try:
                    if len(valid.recv(8, socket.MSG_PEEK)) == 0:
                        print("Client Disconnected")
                        self._connections.remove(valid)
                        continue
                    headerRaw = bytes()
                    while len(headerRaw) < 8:
                        headerRaw += valid.recv(8 - len(headerRaw))
                        
                    header = struct.unpack('ii', headerRaw)
                    bytesBuf = bytes()
                    while len(bytesBuf) < header[1]:
                        bytesBuf += valid.recv(header[1] - len(bytesBuf))

                    if header[0] in self._signatureDictionary:
                        dataDic = json.loads(bytesBuf)
                        (resultCode, returnDic) = self._signatureDictionary[header[0]](dataDic)
                        dataEnc = json.dumps(returnDic)
                        bytesBuf = struct.pack('ii', resultCode, len(dataEnc))
                        bytesBuf += bytes(dataEnc, 'ascii')
                    else:
                        bytesBuf = struct.pack('ii', -1, 0)
                    valid.sendall(bytesBuf)
                except Exception as ex:
                    valid.close()
                    self._connections.remove(valid)
                    print("Client Removed: ", ex)
    
    def close(self):
        self._masterSocket.close()

class NetworkClient(object):
    def __init__(self, address, port):
        self.readyState = False
        self._clientSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._clientSocket.setblocking(True)
        try:
            self._clientSocket.connect((address, port))
            self.readyState = True
        except Exception as ex:
            #print("Error: ", ex)
            pass

    def networkCall(self, instruction, arguments):
        if self.readyState:
            try:
                encodedMsg = json.dumps(arguments)
                header =  struct.pack('ii', 0x00, len(encodedMsg))
                self._clientSocket.sendall(header + bytes(encodedMsg))
                
                bytesBuff = bytes()
                while len(bytesBuff) < 8:
                    bytesBuff += self._clientSocket.recv(8 - len(bytesBuff))
                decodedHeader = struct.unpack('ii', bytesBuff)
                bytesBuff = bytes()
                while len(bytesBuff) < decodedHeader[1]:
                    bytesBuff += self._clientSocket.recv(decodedHeader[1] - len(bytesBuff))
                return json.loads(bytesBuff)
            except Exception as ex:
                #print("Error: ", ex)
                self.readyState = False
                return None
        else:
            return None