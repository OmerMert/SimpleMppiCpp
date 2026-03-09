import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket

class MPPICostMapEnv(gym.Env):
    """
    UDP-based Gymnasium Environment for MPPI Cost Map Optimizer.
    The agent takes an action once (determines the weights),
    and the C++ engine is executed over UDP, returning the simulation result (reward).
    """
    def __init__(self):
        super(MPPICostMapEnv, self).__init__()
        
        # --- UDP COMMUNICATION SETTINGS ---
        self.udp_ip = "127.0.0.1"
        self.udp_port_cpp = 5006
        self.udp_port_py = 5007
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port_py))
        self.sock.settimeout(10000.0)  # Maximum wait time for the simulation to finish (seconds)
        
        # 1. Action Space: w_target and w_obstacle (between 0.0 and 1.0)
        self.action_space = spaces.Box(low=0.0, high=1.0, shape=(4,), dtype=np.float32)
        
        # 2. Observation Space: Dummy (fixed) state since the agent decides before driving.
        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)

    def step(self, action):
        """
        Sends the weights produced by the agent to C++ via UDP and waits for the result.
        """
        # Retrieve the 4 weights produced by the agent
        MAX_WEIGHT = 50.0 
        w_x   = float(action[0]) * MAX_WEIGHT
        w_y   = float(action[1]) * MAX_WEIGHT
        w_yaw = float(action[2]) * MAX_WEIGHT
        w_v   = float(action[3]) * MAX_WEIGHT
        
        # Prepare the message
        message = f"{w_x},{w_y},{w_yaw},{w_v}"

        try:
            # 1. Send the weights to the C++ Engine
            self.sock.sendto(message.encode('utf-8'), (self.udp_ip, self.udp_port_cpp))
            
            # 2. Wait for the simulation result (reward) from the C++ engine
            data, _ = self.sock.recvfrom(1024)
            reward = float(data.decode('utf-8'))
            
        except socket.timeout:
            print("[Python Error] C++ engine timed out! The simulation may be too heavy.")
            
        except Exception as e:
            print(f"[Python Error] UDP Connection issue: {e}")
            
        # End the episode since the simulation is finished
        terminated = True  
        truncated = False
        
        # Return a dummy state since we won't proceed to the next step
        return np.array([0.0], dtype=np.float32), reward, terminated, truncated, {}
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # When the environment is reset, we send fixed initial data to the agent
        return np.array([0.0], dtype=np.float32), {}
    
    def close(self):
        """
        Clean up the socket when the environment is closed.
        """
        self.sock.close()
