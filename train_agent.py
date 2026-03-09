from stable_baselines3 import PPO  # One of the best algorithms for continuous actions
from mppi_gym_env import MPPICostMapEnv  # Importing the UDP environment we just wrote
import numpy as np

print("[Brain] Starting MPPI Cost Map Optimizer Training Script...")

# 1. Set Up the Race Track (Environment)
# The AI does not know that this environment is connected to C++ via UDP in the background; it assumes it is a standard game.
env = MPPICostMapEnv()

# 2. Create the RL Agent (Driver)
# We are using the PPO algorithm. 'MlpPolicy' refers to a standard neural network.
STEP_COUNT = 1000  # If we want it to only try 50 times and finish:

model = PPO(
    "MlpPolicy", 
    env, 
    n_steps=STEP_COUNT,  # Reducing the memory limit
    batch_size=STEP_COUNT,  # Batch size cannot be greater than n_steps
    verbose=1
)

print("[Brain] Agent created, trial-and-error process is starting...")

# 3. Start Training (For example, let it learn over a total of 5000 steps/episodes)
# As soon as this line runs, the agent will call the step() function thousands of times, 
# continuously sending different weights to the C++ engine and updating its network based on the rewards received.
model.learn(total_timesteps=STEP_COUNT)

print("[Brain] Training Completed! The best weights have been found.")

# 4. Save the Learned Neural Network (For future testing)
model.save("mppi_cost_map_optimizer_ppo_model")

# Close the environment
env.close()


# 1. Load the model from the zip file
model = PPO.load("mppi_cost_map_optimizer_ppo_model")

# 2. Ask the agent the question (In our system, the state was always [0.0])
dummy_obs = np.array([0.0], dtype=np.float32)

# We use deterministic=True so that the agent no longer "explores/discovers", 
# we want it to give us the BEST (certain) result it has found directly.
action, _ = model.predict(dummy_obs, deterministic=True)

# 3. Convert the percentages (0-1) to the maximum weight we specified (50.0)
MAX_WEIGHT = 50.0
w_x = float(action[0]) * MAX_WEIGHT
w_y = float(action[1]) * MAX_WEIGHT
w_yaw = float(action[2]) * MAX_WEIGHT
w_v = float(action[3]) * MAX_WEIGHT

print("\nBEST WEIGHTS FOUND AS A RESULT OF TRAINING")
print("-" * 50)
print(f"w_x   (X Axis Error)     : {w_x}")
print(f"w_y   (Y Axis Error)     : {w_y}")
print(f"w_yaw (Yaw Error)        : {w_yaw}")
print(f"w_v   (Velocity Error)   : {w_v}")
print("-" * 50)
