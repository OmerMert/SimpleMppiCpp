import socket
import struct
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import Tuple
import json

# --- Vehicle Parameters ---
VEHICLE_W = None
VEHICLE_L = None
WHEEL_W = None
WHEEL_L = None
MAX_STEER_ABS = None  # [rad]
MAX_ACCEL_ABS = None  # [m/s^2]
OBSTACLES = []

def load_config(file_path):

    global VEHICLE_W, VEHICLE_L, WHEEL_W, WHEEL_L, MAX_STEER_ABS, MAX_ACCEL_ABS, OBSTACLES

    try:
        with open(file_path, 'r') as f:
            data = json.load(f)
            
        # Vehicle Config
        vehicle_config = data["VEHICLE_CONFIG"]
        VEHICLE_W = vehicle_config["W"]
        VEHICLE_L = vehicle_config["L"]
        WHEEL_W = vehicle_config["WHEEL_W"]
        WHEEL_L = vehicle_config["WHEEL_L"]
        
        # Diğer Sabitleri Atama
        MAX_STEER_ABS = data["max_steer_abs"]
        MAX_ACCEL_ABS = data["max_accel_abs"]

        # Engel Listesini Atama
        OBSTACLES = [tuple(obs) for obs in data["OBSTACLES"]]
        
        print("Konfigürasyon başarıyla yüklendi ve global modülde saklandı.")
        print("Config loaded successfully.")
        return True

    except Exception as e:
        print(f"HATA: Konfigürasyon yüklenemedi: {e}")
        print(f"[ERROR] Config can not loaded: {e}")
        return False

def affine_transform(xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
    transformed_x = []
    transformed_y = []
    for i, xval in enumerate(xlist):
        transformed_x.append((xlist[i])*np.cos(angle)-(ylist[i])*np.sin(angle)+translation[0])
        transformed_y.append((xlist[i])*np.sin(angle)+(ylist[i])*np.cos(angle)+translation[1])
    transformed_x.append(transformed_x[0])
    transformed_y.append(transformed_y[0])
    return transformed_x, transformed_y



# --- Update Animation Function ---
def update_animation(t, x, y, yaw, v, steer, accel):
 
    # --- Save Path History ---
    path_history_x.append(x)
    path_history_y.append(y)
    
    # --- Clear Axes ---
    main_ax.clear()
    minimap_ax.clear()
    steer_ax.clear()
    accel_ax.clear()

    # --- 1. Main View ---
    main_ax.set_aspect('equal')
    main_ax.set_xlim(x - 20.0, x + 20.0) # Follow the vehicle
    main_ax.set_ylim(y - 25.0, y + 25.0) # Follow the vehicle
    main_ax.axis('off')

    # Draw Obstacles
    for obs in OBSTACLES:
        obs_x, obs_y, obs_r = obs
        # Create Circle Patch
        circle = patches.Circle((obs_x, obs_y), radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
        # Add Patch to Axes
        main_ax.add_patch(circle)
        # Add to Mini Map
        circle_mini = patches.Circle((obs_x, obs_y), radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
        minimap_ax.add_patch(circle_mini)
    
    # Draw Reference Path (Global)
    main_ax.plot(ref_path[:, 0], ref_path[:, 1], color='black', linestyle="dashed", linewidth=1.5)
    # Draw Path History (Global)
    main_ax.plot(path_history_x, path_history_y, color='blue', linewidth=1.0)
    
    # Draw Vehicle Body
    v_body_x, v_body_y = affine_transform(v_shape_x, v_shape_y, yaw, [x, y])
    main_ax.plot(v_body_x, v_body_y, color='black', linewidth=2.0, zorder=3)
    
    # Draw Wheels
    # Rear-left
    w_rl_x, w_rl_y = affine_transform(w_shape_x, w_shape_y, 0.0, w_pos_rl)
    w_rl_rot_x, w_rl_rot_y = affine_transform(w_rl_x, w_rl_y, yaw, [x, y])
    main_ax.fill(w_rl_rot_x, w_rl_rot_y, color='black', zorder=3)
    
    # Rear-right
    w_rr_x, w_rr_y = affine_transform(w_shape_x, w_shape_y, 0.0, w_pos_rr)
    w_rr_rot_x, w_rr_rot_y = affine_transform(w_rr_x, w_rr_y, yaw, [x, y])
    main_ax.fill(w_rr_rot_x, w_rr_rot_y, color='black', zorder=3)
    
    # Front-left (rotated by steer)
    w_fl_x, w_fl_y = affine_transform(w_shape_x, w_shape_y, steer, w_pos_fl)
    w_fl_rot_x, w_fl_rot_y = affine_transform(w_fl_x, w_fl_y, yaw, [x, y])
    main_ax.fill(w_fl_rot_x, w_fl_rot_y, color='black', zorder=3)

    # Front-right (rotated by steer)
    w_fr_x, w_fr_y = affine_transform(w_shape_x, w_shape_y, steer, w_pos_fr)
    w_fr_rot_x, w_fr_rot_y = affine_transform(w_fr_x, w_fr_y, yaw, [x, y])
    main_ax.fill(w_fr_rot_x, w_fr_rot_y, color='black', zorder=3)

    # Draw Info Text
    text = f"Velocity = {v:>+6.1f} [m/s]\nTime = {t:>5.2f} [s]"
    main_ax.text(0.5, 0.05, text, ha='center', transform=main_ax.transAxes, fontsize=12, fontfamily='monospace')

    # --- 2. Mini Map View (Global) ---
    minimap_ax.set_aspect('equal')
    minimap_ax.axis('off')
    minimap_ax.plot(ref_path[:, 0], ref_path[:, 1], color='black', linestyle='dashed')
    minimap_ax.plot(path_history_x, path_history_y, color='blue', linewidth=1.0)
    v_body_global_x, v_body_global_y = affine_transform(v_shape_x, v_shape_y, yaw, [x, y])
    minimap_ax.plot(v_body_global_x, v_body_global_y, color='black', linewidth=1.0)
    
    # --- 3. Steering and Acceleration Gauges ---
    pie_colors = ["lightgray", "black", "lightgray", "white"]
    pie_props = {'linewidth': 0, "edgecolor":"white", "width":0.4}
    PIE_RATE = 3.0/4.0
    PIE_STARTANGLE = 225

    # Steering
    steer_ax.set_title("Steering Angle", fontsize="12")
    steer_ax.axis('off')
    s_abs = np.clip(np.abs(steer), 0, MAX_STEER_ABS)
    if steer < 0.0:
        steer_ax.pie([MAX_STEER_ABS*PIE_RATE, s_abs*PIE_RATE, (MAX_STEER_ABS-s_abs)*PIE_RATE, 2*MAX_STEER_ABS*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=pie_colors, wedgeprops=pie_props)
    else:
        steer_ax.pie([(MAX_STEER_ABS-s_abs)*PIE_RATE, s_abs*PIE_RATE, MAX_STEER_ABS*PIE_RATE, 2*MAX_STEER_ABS*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=pie_colors, wedgeprops=pie_props)
    steer_ax.text(0, -1, f"{np.rad2deg(steer):+.2f} [deg]", size = 14, ha='center', va='center', fontfamily='monospace')

    # Acceleration
    accel_ax.set_title("Acceleration", fontsize="12")
    accel_ax.axis('off')
    a_abs = np.clip(np.abs(accel), 0, MAX_ACCEL_ABS)
    if accel > 0.0:
        accel_ax.pie([MAX_ACCEL_ABS*PIE_RATE, a_abs*PIE_RATE, (MAX_ACCEL_ABS-a_abs)*PIE_RATE, 2*MAX_ACCEL_ABS*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=pie_colors, wedgeprops=pie_props)
    else:
        accel_ax.pie([(MAX_ACCEL_ABS-a_abs)*PIE_RATE, a_abs*PIE_RATE, MAX_ACCEL_ABS*PIE_RATE, 2*MAX_ACCEL_ABS*(1-PIE_RATE)], startangle=PIE_STARTANGLE, counterclock=False, colors=pie_colors, wedgeprops=pie_props)
    accel_ax.text(0, -1, f"{accel:+.2f} [m/s^2]", size = 14, ha='center', va='center', fontfamily='monospace')
    
    # --- Refresh Figure ---
    fig.canvas.flush_events()


# --- UDP Listener ---
def start_udp_listener():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 5005
    BUFFER_SIZE = 56 # 7 double x 8 byte

    # Struct format for 7 doubles
    DATA_FORMAT = "ddddddd" 

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"[INFO] Python UDP listenner {UDP_IP}:{UDP_PORT} is started.")

    try:
        while True:
            # Receive UDP Packet
            data, addr = sock.recvfrom(BUFFER_SIZE)
            
            # Unpack Data
            t, x, y, yaw, v, steer, accel = struct.unpack(DATA_FORMAT, data)
            
            # Update Animation
            update_animation(t, x, y, yaw, v, steer, accel)
            
    except Exception as e:
        print(f"Error occured: {e}")
        sock.close()

if __name__ == "__main__":
    load_config("config.json")
        # --- Vehicle and Wheel Shapes ---
    v_shape_x = [-0.5*VEHICLE_L, -0.5*VEHICLE_L, +0.5*VEHICLE_L, +0.5*VEHICLE_L, -0.5*VEHICLE_L]
    v_shape_y = [0.0, +0.5*VEHICLE_W, +0.5*VEHICLE_W, -0.5*VEHICLE_W, -0.5*VEHICLE_W]
    w_shape_x = np.array([-0.5*WHEEL_L, -0.5*WHEEL_L, +0.5*WHEEL_L, +0.5*WHEEL_L, -0.5*WHEEL_L])
    w_shape_y = np.array([0.0, +0.5*WHEEL_W, +0.5*WHEEL_W, -0.5*WHEEL_W, -0.5*WHEEL_W])
    w_pos_rl = [-0.3*VEHICLE_L,  0.3*VEHICLE_W]
    w_pos_rr = [-0.3*VEHICLE_L, -0.3*VEHICLE_W]
    w_pos_fl = [ 0.3*VEHICLE_L,  0.3*VEHICLE_W]
    w_pos_fr = [ 0.3*VEHICLE_L, -0.3*VEHICLE_W]

    # --- Load Reference Path ---
    try:
        ref_path_df = pd.read_csv("data/ovalpath.csv")
        ref_path = ref_path_df[['x', 'y']].to_numpy()
    except Exception as e:
        print(f"Referans path can not loaded: {e}")
        ref_path = np.array([[-1, -1], [1, 1]]) # Dummy path

    # --- Matplotlib Figure Setup ---
    print("Matplotlib figure is being set up...")
    plt.ion()
    fig = plt.figure(figsize=(9,9))
    main_ax = plt.subplot2grid((3,4), (0,0), rowspan=3, colspan=3)
    minimap_ax = plt.subplot2grid((3,4), (0,3))
    steer_ax = plt.subplot2grid((3,4), (1,3))
    accel_ax = plt.subplot2grid((3,4), (2,3))
    fig.tight_layout()

    # --- Path History ---
    path_history_x = []
    path_history_y = []
    
    start_udp_listener()