import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import subprocess
import os

class MPPICostMapEnv(gym.Env):
    def __init__(self, env_id=0, base_port=5000):
        super(MPPICostMapEnv, self).__init__()
        
        self.env_id = env_id
        self.udp_ip = "127.0.0.1"
        
        # Unique ports are created for each agent
        self.udp_port_cpp = base_port + (env_id * 2)
        self.udp_port_py = base_port + (env_id * 2) + 1
        
        # --- Automatically Start C++ Engine ---
        exe_path = "MppiCpp.exe" 
        if os.path.exists(exe_path):
            print(f"[Env {env_id}] Starting C++... Ports: C++({self.udp_port_cpp}), Py({self.udp_port_py})")
            # Pass ports to the C++ program as command line arguments
            self.cpp_process = subprocess.Popen([exe_path, str(self.udp_port_cpp), str(self.udp_port_py), "train"])
        else:
            print(f"[ERROR] {exe_path} not found!")
            self.cpp_process = None
        

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port_py))
        self.sock.settimeout(10000.0) 
        
        self.action_space = spaces.Box(low=0.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)

    def step(self, action):
        MAX_WEIGHT = 50.0 
        w_x   = float(action[0]) * MAX_WEIGHT
        w_y   = float(action[1]) * MAX_WEIGHT
        w_yaw = float(action[2]) * MAX_WEIGHT
        w_v   = float(action[3]) * MAX_WEIGHT
        
        message = f"{w_x},{w_y},{w_yaw},{w_v}"
        
        try:
            self.sock.sendto(message.encode('utf-8'), (self.udp_ip, self.udp_port_cpp))
            data, _ = self.sock.recvfrom(1024)
            reward = float(data.decode('utf-8'))
        except socket.timeout:
            print(f"[Env {self.env_id}] Timeout!")
            
        terminated = True  
        truncated = False
        return np.array([0.0], dtype=np.float32), reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        return np.array([0.0], dtype=np.float32), {}

    def close(self):
        # Clean up the background C++ process when training ends or is interrupted
        if self.cpp_process is not None:
            self.cpp_process.terminate()
        self.sock.close()