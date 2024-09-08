import socket 
import pickle 
import numpy as np


if __name__ == '__main__':
    HOST = 'localhost'
    PORT = 1234

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    i = 0
    while True:
        if i > 20:
            break
        i += 1
        s.sendall(pickle.dumps(np.array([0.1, 0.1, 0.1, 1, 0, 0])))
        action_raw = s.recv(1024)
    # print("Received action raw type: ", type(action_raw))
    # print("Received action raw: ", action_raw)
        action = pickle.loads(action_raw, encoding='ASCII')

    s.close()
    print('disconnected')