# -*- coding: utf-8 -*-
"""
10 - Jusqu ou chacun tient.

Meme protocole que 04_pousser.py -- poussee de 150 ms, dichotomie sur la force
jusqu au newton pres -- mais applique a l AGENT et a TA CASCADE, dans le meme
environnement, sur le meme robot.

    python 10_limites.py                 les deux, a plusieurs hauteurs
    python 10_limites.py --difficulte 0.6
"""
import argparse
import glob
import math
import os

import numpy as np
import mujoco

from balancier_env import (Balancier, MAX_ACCEL, HZ_POLITIQUE, RAD_PAR_PAS,
                           RAYON)
from firmware import Firmware

DUREE_POUSSEE = 0.15
INSTANT = 2.0

ap = argparse.ArgumentParser()
ap.add_argument('--agent', type=str, default=None)
ap.add_argument('--difficulte', type=float, default=0.0)
ap.add_argument('--essais', type=int, default=5)
a = ap.parse_args()

chemin = a.agent or sorted(glob.glob(os.path.join('agents', '*.zip')),
                           key=os.path.getmtime)[-1]
from stable_baselines3 import PPO
mdl = PPO.load(chemin, device='cpu')
fw = Firmware(dual=True)


def cascade(o, env):
    fw.pitch = float(o[0]) * 10.0
    fw.gyroFilt = float(o[1]) * 100.0
    fw.wheel_sps = float(np.mean(env.sps))
    corr = fw.Kp_spd * fw.wheel_sps + fw.Ki_spd * (float(o[5]) * 0.20
                                                   / (RAD_PAR_PAS * RAYON))
    cible = -max(-6.0, min(6.0, corr))
    accel = fw.Kp_a * (fw.pitch - cible) + fw.Kd_a * fw.gyroFilt
    return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1.0, 1.0)


def agent(o, env):
    return mdl.predict(o, deterministic=True)[0]


def tient(pilote, force, hauteur, graine):
    env = Balancier(difficulte=a.difficulte, avec_consignes=False, graine=graine)
    o, _ = env.reset(seed=graine)
    env.t_poussee = 1e9
    env.hauteur_externe = hauteur
    fw.__init__(dual=True)
    while env.d.time < 6.0:
        env.force_externe = (force if INSTANT <= env.d.time
                             < INSTANT + DUREE_POUSSEE else 0.0)
        o, r, tombe, fini, info = env.step(pilote(o, env))
        if tombe:
            return False
        if fini:
            break
    return True


def limite(pilote, hauteur):
    """Dichotomie. Une force n est retenue que si elle tient sur TOUS les essais
    -- sinon on mesure la chance, pas la robustesse."""
    bas, haut = 0.0, 12.0
    while haut - bas > 0.05:
        mil = (bas + haut) / 2
        ok = all(tient(pilote, mil, hauteur, 200 + i) for i in range(a.essais))
        if ok:
            bas = mil
        else:
            haut = mil
    return bas


if __name__ == '__main__':
    print('  agent %s   |  difficulte %.1f  |  %d essais par force'
          % (chemin, a.difficulte, a.essais))
    print('  poussee de %.0f ms, dichotomie a 0,05 N pres' % (DUREE_POUSSEE * 1000))
    print()
    print('  %-22s %10s %10s %9s' % ('point d application', 'AGENT', 'CASCADE', 'ecart'))
    print('  ' + '-' * 56)
    for nom, h in (('sommet     210 mm', 0.210),
                   ('mi-hauteur 120 mm', 0.120),
                   ('essieu       0 mm', 0.000)):
        la, lc = limite(agent, h), limite(cascade, h)
        ecart = ('%+.0f %%' % (100 * (la / lc - 1))) if lc > 0.01 else '--'
        print('  %-22s %8.2f N %8.2f N %9s' % (nom, la, lc, ecart))
    print()
    print('  Rappel : impulsion = force x duree. %.0f ms de poussee, donc une'
          % (DUREE_POUSSEE * 1000))
    print('  limite de 1 N vaut 0,15 N.s -- une gomme de 20 g lancee a 7,5 m/s.')
