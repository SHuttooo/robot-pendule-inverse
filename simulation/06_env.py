# -*- coding: utf-8 -*-
"""
06 - Verification de l environnement.

L environnement lui-meme vit dans balancier_env.py : Python refuse d importer
un module dont le nom commence par un chiffre, et SubprocVecEnv sous Windows
relance l interpreteur pour chaque worker -- il lui faut donc un vrai module.

    python 06_env.py            debits et sanite
    python 06_env.py --cascade  fait jouer TA cascade dedans, comme temoin
"""
import argparse
import numpy as np
from balancier_env import Balancier, MAX_ACCEL, HZ_POLITIQUE

# ====================================================================== essais
def _essai(env, politique, n_episodes=3):
    tot, longueurs, chutes = [], [], 0
    for _ in range(n_episodes):
        o, _ = env.reset()
        s, k = 0.0, 0
        while True:
            o, r, tombe, fini, _ = env.step(politique(o, env))
            s += r
            k += 1
            if tombe or fini:
                chutes += tombe
                break
        tot.append(s)
        longueurs.append(k)
    return np.mean(tot), np.mean(longueurs), chutes


def _cascade(o, env):
    """TA cascade, ecrite dans l interface de l agent. Temoin de reference."""
    from firmware import Firmware
    if not hasattr(env, '_fw'):
        env._fw = Firmware(dual=True)
    fw = env._fw
    fw.pitch = float(o[0]) * 10.0
    fw.gyroFilt = float(o[1]) * 100.0
    fw.wheel_sps = float(np.mean(env.sps))
    accel = fw.Kp_a * fw.pitch + fw.Kd_a * fw.gyroFilt
    return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1, 1)


if __name__ == '__main__':
    import time
    ap = argparse.ArgumentParser()
    ap.add_argument('--cascade', action='store_true')
    ap.add_argument('--difficulte', type=float, default=0.0)
    a = ap.parse_args()

    env = Balancier(difficulte=a.difficulte)
    o, _ = env.reset()
    print('  observation : %d nombres' % len(o))
    for nom, val in zip(
            ['angle', 'gyro tangage', 'gyro lacet', 'sps droite', 'sps gauche',
             'ecart position', 'ecart cap', 'consigne v', 'consigne w',
             'action-1 d', 'action-1 g', 'reserve'], o):
        print('     %-16s %+8.3f' % (nom, val))
    print('  action      : 2 nombres dans [-1, 1] = acceleration de chaque roue')
    print()

    t = time.perf_counter()
    n = 3000
    for _ in range(n):
        _, _, tombe, fini, _ = env.step(env.action_space.sample())
        if tombe or fini:
            env.reset()
    fps = n / (time.perf_counter() - t)
    print('  debit : %.0f pas de politique/s sur un coeur (%.0fx le temps reel)'
          % (fps, fps / HZ_POLITIQUE))
    print()

    pol = _cascade if a.cascade else (lambda o, e: e.action_space.sample())
    nom = 'TA CASCADE' if a.cascade else 'actions aleatoires'
    for diff in (0.0, 0.3, 0.6, 1.0):
        env.regler_difficulte(diff)
        rec, lon, ch = _essai(env, pol, 4)
        print('  difficulte %.1f  %-20s  recompense %+8.1f   duree %5.0f pas   %d chute(s)/4'
              % (diff, nom, rec, lon, ch))
