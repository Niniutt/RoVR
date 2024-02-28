import socket
import time

host, port = "127.0.0.1", 25001
data = ("{"
            "PositionDelta:[0,0,-1],"
            "Message:['Greetings']"
        "}")

# data = "0,0,-1"
# SOCK_STREAM means TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the server and send the data
    sock.connect((host, port))

    # for i in range(5):
    while True:
        sock.sendall(data.encode("utf-8"))
        response = sock.recv(1024).decode("utf-8")
        print(response)
        time.sleep(0.5)
finally:
    if sock:
        sock.close()
        print("Closed socket")
