from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.vec_env import VecNormalize
from mppi_gym_env import MPPICostMapEnv
import numpy as np

# Helper function to create environments for multiprocessing
def make_env(env_id):
    def _init():
        return MPPICostMapEnv(env_id=env_id)
    return _init

if __name__ == '__main__':
    print("[Brain] Parallel MPPI RL Training Script Starting...")

    # Create agents
    NUM_ENVS = 5 
    
    # Launch different Gym environments in completely separate processes
    env = SubprocVecEnv([make_env(i) for i in range(NUM_ENVS)])
    env = VecNormalize(env, training=True, norm_obs=False, norm_reward=True)

    # Model settings 
    model = PPO(
        "MlpPolicy", 
        env, 
        n_steps=25,       # Data collected by each agent 
        batch_size=25,    # Batch size for training
        n_epochs=10,
        learning_rate=0.0003,
        verbose=1
    )

    print(f"[Brain] {NUM_ENVS} parallel agents created. Training starts...")

    # step total_timesteps / (NUM_ENVS x n_steps)
    model.learn(total_timesteps=1000)

    print("[Brain] Training Completed! Optimal weights have been found.")
    model.save("mppi_cost_map_optimizer_ppo_model")
    
    # Close all C++ windows
    env.close()

    # 1. Load the model from the zip file
    model = PPO.load("mppi_cost_map_optimizer_ppo_model")

    # 2. Ask the agent the question (In our system, the state was always [0.0])
    dummy_obs = np.array([0.0], dtype=np.float32)

    # deterministic=True so that the agent no longer "explores/discovers", 
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