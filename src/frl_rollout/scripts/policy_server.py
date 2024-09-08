import stable_baselines3 as sb3 
import socket 
import pickle 
import numpy as np


class PolicyServer:
    def __init__(self, model_path='src/frl_rollout/scripts/prismatic_model', port=1234):
        self.model = sb3.TD3.load(model_path)
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def run(self):
        self.server.bind(('localhost', self.port)) 
        self.server.listen()
        print("Server is listening to port 1234")
        conn, addr = self.server.accept()
        with conn:
            print(f"Connected by {addr}")

            while True:
                data = conn.recv(1024)

                if not data:
                    print(data)
                    break
                data = pickle.loads(data)
                # print("Received data: ", data)

                action = self.model.predict(data)

                action = pickle.dumps(action)
                conn.sendall(action)


if __name__ == '__main__':
    policy_server = PolicyServer()
    # while True:
    policy_server.run()