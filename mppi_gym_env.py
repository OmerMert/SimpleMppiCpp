import gymnasium as gym
from gymnasium import spaces
import numpy as np
import socket
import subprocess
import struct
import os


class MPPICostMapEnv(gym.Env):
    """
    RL Environment that receives a global cost map from C++ MPPI engine.
    
    The cost map is loaded from costmap.csv (a 0/1 grid file).
    Grid dimensions are derived from reference path bounds + margin.
    
    Observation: (rows, cols, 1) global cost map with gradient
    Action:      [w_x, w_y, w_yaw, w_v] in [0, 1], scaled to MAX_WEIGHT
    Reward:      Total episode reward from C++ simulation
    """

    MAX_WEIGHT = 50.0

    # Default grid size (path X[-30,30]+10m margin=80cols, Y[0,30]+10m=50rows @ 1m/cell)
    DEFAULT_ROWS = 50
    DEFAULT_COLS = 80

    def __init__(self, env_id=0, base_port=5000):
        super(MPPICostMapEnv, self).__init__()

        self.env_id = env_id
        self.udp_ip = "127.0.0.1"
        self.udp_port_cpp = base_port + (env_id * 2)
        self.udp_port_py  = base_port + (env_id * 2) + 1

        self.grid_rows = self.DEFAULT_ROWS
        self.grid_cols = self.DEFAULT_COLS

        # --- Start C++ Engine ---
        exe_path = "MppiCpp.exe"
        if os.path.exists(exe_path):
            print(f"[Env {env_id}] Starting C++... Ports: C++({self.udp_port_cpp}), Py({self.udp_port_py})")
            self.cpp_process = subprocess.Popen(
                [exe_path, str(self.udp_port_cpp), str(self.udp_port_py), "train"],
                stderr=subprocess.DEVNULL
            )
        else:
            print(f"[ERROR] {exe_path} not found!")
            self.cpp_process = None

        # --- UDP Socket ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port_py))
        self.sock.settimeout(10000.0)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)

        # --- Gym Spaces ---
        self.observation_space = spaces.Box(
            low=0.0, high=1.0,
            shape=(self.grid_rows, self.grid_cols, 1),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=0.0, high=1.0, shape=(4,), dtype=np.float32
        )

        self._current_costmap = np.zeros(
            (self.grid_rows, self.grid_cols, 1), dtype=np.float32
        )

    def step(self, action):
        w_x   = float(action[0]) * self.MAX_WEIGHT
        w_y   = float(action[1]) * self.MAX_WEIGHT
        w_yaw = float(action[2]) * self.MAX_WEIGHT
        w_v   = float(action[3]) * self.MAX_WEIGHT

        message = f"{w_x},{w_y},{w_yaw},{w_v}"
        reward = 0.0

        try:
            self.sock.sendto(
                message.encode('utf-8'),
                (self.udp_ip, self.udp_port_cpp)
            )

            # Receive: [rows:i][cols:i][res:f][x_min:f][y_min:f][reward:f] + costmap
            data, _ = self.sock.recvfrom(131072)

            offset = 0
            rows = struct.unpack_from('i', data, offset)[0]; offset += 4
            cols = struct.unpack_from('i', data, offset)[0]; offset += 4
            _    = struct.unpack_from('f', data, offset)[0]; offset += 4  # resolution
            _    = struct.unpack_from('f', data, offset)[0]; offset += 4  # x_min
            _    = struct.unpack_from('f', data, offset)[0]; offset += 4  # y_min
            reward = struct.unpack_from('f', data, offset)[0]; offset += 4

            num_cells = rows * cols
            costmap_flat = struct.unpack_from(f'{num_cells}f', data, offset)

            self._current_costmap = np.array(
                costmap_flat, dtype=np.float32
            ).reshape(rows, cols, 1)

        except socket.timeout:
            print(f"[Env {self.env_id}] Timeout!")
        except struct.error as e:
            print(f"[Env {self.env_id}] Parse error: {e}")

        terminated = True
        truncated = False
        return self._current_costmap, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        return self._current_costmap, {}

    def close(self):
        if self.cpp_process is not None:
            self.cpp_process.terminate()
        self.sock.close()
