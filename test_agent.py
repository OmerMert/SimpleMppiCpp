import subprocess
import os
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

# --- Cost Map ---
COSTMAP_GRID = None       # 2D numpy array (rows x cols) of 0/1
COSTMAP_RESOLUTION = 1.0
COSTMAP_X_MIN = -40.0
COSTMAP_Y_MIN = -10.0

def load_config(file_path):
    global VEHICLE_W, VEHICLE_L, WHEEL_W, WHEEL_L, MAX_STEER_ABS, MAX_ACCEL_ABS
    global COSTMAP_GRID, COSTMAP_RESOLUTION, COSTMAP_X_MIN, COSTMAP_Y_MIN

    try:
        with open(file_path, 'r') as f:
            data = json.load(f)

        # Vehicle Config
        vehicle_config = data["VEHICLE_CONFIG"]
        VEHICLE_W = vehicle_config["W"]
        VEHICLE_L = vehicle_config["L"]
        WHEEL_W = vehicle_config["WHEEL_W"]
        WHEEL_L = vehicle_config["WHEEL_L"]

        MAX_STEER_ABS = data["max_steer_abs"]
        MAX_ACCEL_ABS = data["max_accel_abs"]

        # Cost Map config
        COSTMAP_RESOLUTION = data["COSTMAP_RESOLUTION"]
        costmap_margin = data["COSTMAP_MARGIN"]
        costmap_file = data["COSTMAP_FILE"]
        ref_path_file = data["REF_PATH_FILE"]

        print(f"[INFO] Config loaded. CostMap: {costmap_file}, RefPath: {ref_path_file}")
        return costmap_file, costmap_margin, ref_path_file

    except Exception as e:
        print(f"[ERROR] Config cannot be loaded: {e}")
        return None, 0.0, None


def load_costmap(costmap_file, ref_path, costmap_margin):
    """Load costmap.csv and compute map bounds from reference path."""
    global COSTMAP_GRID, COSTMAP_X_MIN, COSTMAP_Y_MIN

    try:
        COSTMAP_GRID = np.loadtxt(costmap_file, delimiter=',', dtype=int)
        print(f"[INFO] CostMap loaded: {COSTMAP_GRID.shape[0]}x{COSTMAP_GRID.shape[1]} "
              f"({np.sum(COSTMAP_GRID)} obstacle cells)")
    except Exception as e:
        print(f"[ERROR] CostMap cannot be loaded: {e}")
        COSTMAP_GRID = None
        return

    # Compute map bounds (same logic as C++ main.cpp)
    COSTMAP_X_MIN = ref_path[:, 0].min() - costmap_margin
    COSTMAP_Y_MIN = ref_path[:, 1].min() - costmap_margin


def affine_transform(xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
    transformed_x = []
    transformed_y = []
    for i, xval in enumerate(xlist):
        transformed_x.append((xlist[i])*np.cos(angle)-(ylist[i])*np.sin(angle)+translation[0])
        transformed_y.append((xlist[i])*np.sin(angle)+(ylist[i])*np.cos(angle)+translation[1])
    transformed_x.append(transformed_x[0])
    transformed_y.append(transformed_y[0])
    return transformed_x, transformed_y


def draw_costmap_obstacles(ax, xlim=None, ylim=None):
    """Draw obstacle cells from costmap grid onto a matplotlib axis."""
    if COSTMAP_GRID is None:
        return

    rows, cols = COSTMAP_GRID.shape
    res = COSTMAP_RESOLUTION

    for r in range(rows):
        for c in range(cols):
            if COSTMAP_GRID[r, c] == 1:
                wx = COSTMAP_X_MIN + c * res
                wy = COSTMAP_Y_MIN + r * res

                # Frustum cull: skip cells outside view
                if xlim is not None:
                    if wx + res < xlim[0] or wx > xlim[1]:
                        continue
                if ylim is not None:
                    if wy + res < ylim[0] or wy > ylim[1]:
                        continue

                rect = patches.Rectangle(
                    (wx, wy), res, res,
                    fc='#D85A30', ec='#993C1D', linewidth=0.3,
                    alpha=0.85, zorder=0
                )
                ax.add_patch(rect)


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
    view_xlim = (x - 20.0, x + 20.0)
    view_ylim = (y - 25.0, y + 25.0)
    main_ax.set_xlim(*view_xlim)
    main_ax.set_ylim(*view_ylim)
    main_ax.axis('off')

    # Draw Obstacles from cost map (only visible cells)
    draw_costmap_obstacles(main_ax, xlim=view_xlim, ylim=view_ylim)

    # Draw Obstacles on mini map (all cells)
    draw_costmap_obstacles(minimap_ax)

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

    print(f"[INFO] Python UDP listener {UDP_IP}:{UDP_PORT} is started.")

    exe_path = "MppiCpp.exe"
    if os.path.exists(exe_path):
       subprocess.Popen([exe_path, "5006", "5005", "normal"])
    else:
        print(f"[ERROR] {exe_path} not found!")

    try:
        while True:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            t, x, y, yaw, v, steer, accel = struct.unpack(DATA_FORMAT, data)
            update_animation(t, x, y, yaw, v, steer, accel)

    except Exception as e:
        print(f"Error occured: {e}")
        sock.close()


if __name__ == "__main__":
    costmap_file, costmap_margin, ref_path_file = load_config("config.json")

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
        ref_path_df = pd.read_csv(ref_path_file)
        ref_path = ref_path_df[['x', 'y']].to_numpy()
        print(f"[INFO] Reference path loaded from: {ref_path_file} ({len(ref_path)} points)")
    except Exception as e:
        print(f"Reference path cannot be loaded: {e}")
        ref_path = np.array([[-1, -1], [1, 1]])

    # --- Load Cost Map from CSV ---
    if costmap_file is not None:
        load_costmap(costmap_file, ref_path, costmap_margin)

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