import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from mppi_gym_env import MPPICostMapEnv
import gymnasium as gym
import numpy as np


class CostMapCNN(BaseFeaturesExtractor):
    """
    Custom CNN feature extractor for global cost map (50x80x1).
    Uses AdaptiveAvgPool so it works with any grid size.
    
    Architecture:
        Conv2d(1, 16, 5, stride=2, pad=2) -> ReLU
        Conv2d(16, 32, 3, stride=2, pad=1) -> ReLU
        Conv2d(32, 64, 3, stride=2, pad=1) -> ReLU
        AdaptiveAvgPool2d(4, 4) -> Flatten -> Linear(1024, 128) -> ReLU
    """

    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 128):
        super(CostMapCNN, self).__init__(observation_space, features_dim)

        n_input_channels = observation_space.shape[2]  # (H, W, C) -> C=1

        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Flatten(),
        )

        self.linear = nn.Sequential(
            nn.Linear(64 * 4 * 4, features_dim),
            nn.ReLU(),
        )

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # SB3 VecTransposeImage: (N, H, W, C) -> (N, C, H, W)
        if observations.shape[-1] in (1, 3) and observations.shape[1] not in (1, 3):
            observations = observations.permute(0, 3, 1, 2)
        return self.linear(self.cnn(observations))


def make_env(env_id):
    def _init():
        return MPPICostMapEnv(env_id=env_id)
    return _init


if __name__ == '__main__':
    print("=" * 60)
    print("[Brain] MPPI RL Training with Global Cost Map from CSV")
    print("=" * 60)

    NUM_ENVS = 5

    env = SubprocVecEnv([make_env(i) for i in range(NUM_ENVS)])
    env = VecNormalize(env, training=True, norm_obs=False, norm_reward=True)

    policy_kwargs = dict(
        features_extractor_class=CostMapCNN,
        features_extractor_kwargs=dict(features_dim=128),
        net_arch=dict(pi=[64, 32], vf=[64, 32]),
    )

    model = PPO(
        "CnnPolicy",
        env,
        policy_kwargs=policy_kwargs,
        n_steps=25,
        batch_size=25,
        n_epochs=10,
        learning_rate=0.0003,
        verbose=1,
    )

    print(f"[Brain] {NUM_ENVS} parallel agents with CNN policy.")
    print(f"[Brain] Cost map source: data/costmap.csv")
    print("=" * 60)

    model.learn(total_timesteps=1000)

    print("[Brain] Training Completed!")
    model.save("mppi_global_costmap_model")
    env.close()

    # --- Inference ---
    print("\n" + "=" * 60)
    print("[Brain] Loading model for inference...")

    model = PPO.load("mppi_global_costmap_model")

    # Dummy cost map (50x80, zeros = no obstacles)
    dummy_costmap = np.zeros((50, 80, 1), dtype=np.float32)
    action, _ = model.predict(dummy_costmap, deterministic=True)

    MAX_WEIGHT = 50.0
    print("\nPREDICTED WEIGHTS:")
    print("-" * 50)
    print(f"w_x   : {float(action[0]) * MAX_WEIGHT:.2f}")
    print(f"w_y   : {float(action[1]) * MAX_WEIGHT:.2f}")
    print(f"w_yaw : {float(action[2]) * MAX_WEIGHT:.2f}")
    print(f"w_v   : {float(action[3]) * MAX_WEIGHT:.2f}")
    print("-" * 50)
