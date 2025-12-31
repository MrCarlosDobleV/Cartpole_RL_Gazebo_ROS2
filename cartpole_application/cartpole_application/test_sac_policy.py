import argparse

import numpy as np
from stable_baselines3 import SAC

from cartpole_application.cartpole_env import GazeboCartPoleEnv


def main():
    parser = argparse.ArgumentParser(description="Test a SAC checkpoint for 2 episodes.")
    parser.add_argument(
        "--model",
        default="./tb_cartpole/checkpoints/sac_cartpole_300000_steps.zip",
        help="Path to SAC checkpoint (.zip).",
    )
    args = parser.parse_args()

    env = GazeboCartPoleEnv()
    model = SAC.load(args.model, env=env)

    for episode in range(11):
        obs, _ = env.reset()
        done = False
        total_reward = 0.0

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            total_reward += float(reward)
            done = terminated or truncated

        print(f"Episode {episode}: total_reward={total_reward:.3f}")

    env.close()


if __name__ == "__main__":
    main()
