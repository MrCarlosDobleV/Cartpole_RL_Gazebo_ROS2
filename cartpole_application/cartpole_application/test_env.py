import numpy as np
from cartpole_application.cartpole_env import GazeboCartPoleEnv

env = GazeboCartPoleEnv()

for episode in range(3):
    print(f"\n=== EPISODE {episode} ===")
    obs, _ = env.reset()
    print("Reset obs:", obs)

    for i in range(200000):
        action = np.array([np.random.uniform(-10, 10)], dtype=np.float32)
        obs, reward, terminated, truncated, _ = env.step(action)

        if terminated or truncated:
            print("Episode ended")
            break

env.close()
