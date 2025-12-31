import time
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback

from cartpole_application.cartpole_env import GazeboCartPoleEnv


def main():
    # IMPORTANT: one env only (Gazebo cannot be vectorized)
    env = GazeboCartPoleEnv()

    model = SAC(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log="./tb_cartpole/"
    )

    checkpoint_dir = "./tb_cartpole/checkpoints"
    checkpoint_callback = CheckpointCallback(
        save_freq=50_000,
        save_path=checkpoint_dir,
        name_prefix="sac_cartpole"
    )

    print("Starting SAC training...")
    model.learn(
        total_timesteps=1_000_000,
        tb_log_name="sac_cartpole",
        callback=checkpoint_callback,
        log_interval=1
    )

    model.save("sac_cartpole_final")
    env.close()


if __name__ == "__main__":
    main()
