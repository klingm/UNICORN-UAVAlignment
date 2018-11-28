#!/usr/bin/python
import socket
import sys
import time

# Echo client program
import socket
import sys

HOST = '192.168.137.254'    # The remote host
PORT = 50000              # The same port as used by the server
s = None
for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res
    try:
        s = socket.socket(af, socktype, proto)
    except OSError as msg:
        s = None
        continue
    try:
        s.connect(sa)
    except OSError as msg:
        s.close()
        s = None
        continue
    break
if s is None:
    print('could not open socket')
    sys.exit(1)

while True:
	cmd = input("Command: (a)lign, (h)alt, (r)eset, (e)xit >> ")
	if cmd == "a" or cmd == "h" or cmd == "r":
		s.sendall(str.encode(cmd[:1]));
	elif cmd == "e":
		break
	else:
		print("Invalid command!")

print('Exiting...')