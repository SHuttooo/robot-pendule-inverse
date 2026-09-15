# -*- coding: utf-8 -*-
"""
Debit reel d un entrainement PPO sur cette machine.

Mesure ce qui compte : des pas de commande par seconde, reseau compris, avec
la parallelisation reelle. Pas une extrapolation.

Sous Windows, SubprocVecEnv relance l interpreteur pour chaque worker : la
classe d environnement DOIT vivre dans un module importable, et tout le code
de lancement sous "if __name__ == '__main__'". D ou ce fichier.

    python bench_rl.py
"""
import time
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
import modele


class Mini(gym.Env):
    """Environnement minimal : uniquement pour mesurer le debit."""

    def __init__(self, dimension='3d', actionneur='vitesse'):
        self.m = mujoco.MjModel.from_xml_string(modele.xml(actionneur, dimension))
        self.d = mujoco.MjData(self.m)
        self.n = round((1 / 200) / self.m.opt.timestep)
        self.observation_space = spaces.Box(-np.inf, np.inf, (10,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)

    def _obs(self):
        return np.concatenate([self.d.sensordata[:8],
                               self.d.qvel[:2]]).astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        return self._obs(), {}

    def step(self, a):
        self.d.ctrl[:] = np.asarray(a, dtype=np.float64) * 12.0
        for _ in range(self.n):
            mujoco.mj_step(self.m, self.d)
        return self._obs(), 1.0, False, self.d.time > 10.0, {}


if __name__ == '__main__':
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv

    print('  reseau : MLP 2 couches de 32, sur CPU')
    print()
    for nenv in (1, 4, 8, 16):
        cls = DummyVecEnv if nenv == 1 else SubprocVecEnv
        venv = cls([Mini for _ in range(nenv)])
        mdl = PPO('MlpPolicy', venv, n_steps=256, batch_size=1024, verbose=0,
                  policy_kwargs=dict(net_arch=[32, 32]), device='cpu')
        mdl.learn(total_timesteps=nenv * 256)                  # chauffe
        t = time.perf_counter()
        mdl.learn(total_timesteps=nenv * 256 * 6)
        fps = nenv * 256 * 6 / (time.perf_counter() - t)
        print('  %2d env  %8.0f pas/s   %6.2f M pas en 10 min   %5.0f episodes de 10 s'
              % (nenv, fps, fps * 600 / 1e6, fps * 600 / 2000))
        venv.close()
