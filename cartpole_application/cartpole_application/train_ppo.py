import time
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback

from cartpole_application.cartpole_env import GazeboCartPoleEnv

def main():
    # IMPORTANT: one env only (Gazebo cannot be vectorized)
    env = GazeboCartPoleEnv()

    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log="./tb_cartpole/",
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        gae_lambda=0.95,
        learning_rate=3e-4,
        clip_range=0.2,
    )

    checkpoint_dir = "./tb_cartpole/checkpoints"
    checkpoint_callback = CheckpointCallback(
        save_freq=100_000,
        save_path=checkpoint_dir,
        name_prefix="ppo_cartpole"
    )

    print("Starting PPO training...")
    model.learn(
        total_timesteps=1_000_000,
        tb_log_name="ppo_cartpole",
        callback=checkpoint_callback
    )

    model.save("ppo_cartpole_final")
    env.close()

if __name__ == "__main__":
    main()
