import socket

UR_IP = "192.168.10.103"

PORT = 29999

def dash(cmd):
   s = socket.socket()
   s.connect((UR_IP, PORT))
   s.recv(1024)  # banner
   s.sendall((cmd + "\n").encode())
   out = s.recv(4096).decode()
   s.close()

   return out

print(dash("load grip_open.urp"))

print(dash("play"))


