import socket

HOST = '127.0.0.1'
PORT = 1234

def menu():
    

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #s.connect((HOST, PORT))
    while True:
        Test= input('Select setting \r\n 0:Power \r\n 1:Depth \r\n')
        #s.sendall(bytes(Test,'utf-8'))
        #data = s.recv(1024)
        #print('Received', repr(data))    
        if Test == 'q':
            break
    #s.sendall(b'Hella world')
    
    #s.sendall(b'Test')
    

#print('Received', repr(data))