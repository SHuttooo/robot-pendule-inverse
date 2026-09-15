# -*- coding: utf-8 -*-
"""
08 - Regarder l agent, et le confronter a ta cascade.

    python 08_agent.py                        le dernier agent, en direct
    python 08_agent.py --agent agents/balancier_400000_pas.zip
    python 08_agent.py --duel                 agent contre cascade, chiffre
    python 08_agent.py --film                 un GIF au lieu du viewer

Le duel utilise EXACTEMENT les criteres du carnet, plus la poussee encaissee :
les chiffres sont directement comparables a ceux de 03_pid.py et 04_pousser.py.
"""
import argparse
import glob
import math
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

import etat
from balancier_env import (Balancier, MAX_ACCEL, HZ_POLITIQUE, RAD_PAR_PAS,
                           RAYON, V_MAX_CONSIGNE)
from firmware import Firmware


def dernier_agent():
    f = sorted(glob.glob(os.path.join('agents', '*.zip')), key=os.path.getmtime)
    if not f:
        raise SystemExit('  aucun agent dans agents/ -- lance d abord 07_ppo.py')
    return f[-1]


# ------------------------------------------------------- les deux pilotes
def pilote_agent(chemin):
    from stable_baselines3 import PPO
    mdl = PPO.load(chemin, device='cpu')
    # Seul l ACTEUR est deploye sur l ESP32. Le critique n existe que pendant
    # l entrainement : le compter serait trompeur.
    acteur = (list(mdl.policy.mlp_extractor.policy_net.parameters())
              + list(mdl.policy.action_net.parameters()))
    n = sum(p.numel() for p in acteur)
    tot = sum(p.numel() for p in mdl.policy.parameters())
    print('  agent    %s' % chemin)
    print('           acteur %d poids (a porter sur l ESP32), %d au total avec le critique'
          % (n, tot))

    def f(o, env):
        a, _ = mdl.predict(o, deterministic=True)
        return a
    return f


def pilote_cascade():
    """Ta cascade, dans l interface de l agent. Meme point d insertion."""
    print('  cascade  Kp=4500  Ki=40  Kd=600  (reglage du carnet)')
    memo = {}

    def f(o, env):
        fw = memo.setdefault('fw', Firmware(dual=True))
        fw.pitch = float(o[0]) * 10.0
        fw.gyroFilt = float(o[1]) * 100.0
        fw.wheel_sps = float(np.mean(env.sps))
        # boucle externe : consigne de vitesse -> inclinaison cible
        v = fw.wheel_sps * RAD_PAR_PAS * RAYON
        corr = fw.Kp_spd * (fw.wheel_sps - env.consigne[0] / (RAD_PAR_PAS * RAYON))
        corr += fw.Ki_spd * (float(o[5]) * 0.20 / (RAD_PAR_PAS * RAYON))
        cible = -max(-6.0, min(6.0, corr))
        err = fw.pitch - cible
        accel = fw.Kp_a * err + fw.Kd_a * fw.gyroFilt
        return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1.0, 1.0)
    return f


# ------------------------------------------------------------- evaluation
def evaluer(pilote, difficulte, n=12, graine=100):
    """Retourne les criteres du carnet, sur n episodes."""
    env = Balancier(actionneur=ACTIONNEUR, difficulte=difficulte, graine=graine)
    err, sps, deriv, longueurs, chutes, recompenses = [], [], [], [], 0, []
    inversions, rugosite = [], []
    for i in range(n):
        o, _ = env.reset(seed=graine + i)
        e, s, x0, R, k = [], [], None, 0.0, 0
        actions = []
        while True:
            o, r, tombe, fini, info = env.step(pilote(o, env))
            R += r
            k += 1
            actions.append(float(info['action'][0]))
            if env.d.time > 1.0:
                e.append(info['tangage'])
                s.append(float(np.mean(env.sps)))
                if x0 is None:
                    x0 = etat.avance(env.m, env.d)
            if tombe or fini:
                chutes += tombe
                break
        longueurs.append(k)
        recompenses.append(R)
        if len(e) > 20:
            err.append(np.std(e))
            sps.append(np.std(s))
            deriv.append(abs(etat.avance(env.m, env.d) - x0) * 1000)
            # Tremblement, par la methode du carnet : on compte les changements
            # de signe. Ici ceux de la VARIATION de commande, c est-a-dire les
            # INVERSIONS. Une correction franche n en produit aucune ; une
            # commande qui tremble en produit des centaines par seconde.
            act = np.array(actions)
            d1 = np.diff(act)
            if len(d1) > 2:
                inversions.append(float(np.sum(np.diff(np.sign(d1)) != 0))
                                  / (len(act) / HZ_POLITIQUE))
                rugosite.append(float(np.sqrt(np.mean(np.diff(act, 2) ** 2))))
    return dict(err=np.mean(err) if err else float('nan'),
                sps=np.mean(sps) if sps else float('nan'),
                deriv=np.mean(deriv) if deriv else float('nan'),
                duree=np.mean(longueurs) / HZ_POLITIQUE,
                survie=100.0 * (1 - chutes / n),
                recompense=np.mean(recompenses),
                inv=np.mean(inversions) if inversions else float('nan'),
                rug=np.mean(rugosite) if rugosite else float('nan'))


ACTIONNEUR = 'couple'


def duel(chemin):
    ag, ca = pilote_agent(chemin), pilote_cascade()
    print()
    for diff in (0.0, 0.3, 0.6, 1.0):
        a, c = evaluer(ag, diff), evaluer(ca, diff)
        print('=' * 74)
        print('  DIFFICULTE %.1f' % diff)
        print('=' * 74)
        print('  %-26s %12s %12s' % ('', 'AGENT', 'CASCADE'))
        print('  %-26s %11.0f%% %11.0f%%' % ('episodes survecus', a['survie'], c['survie']))
        print('  %-26s %12.2f %12.2f  s' % ('duree moyenne', a['duree'], c['duree']))
        print('  %-26s %12.3f %12.3f  deg' % ('erreur d angle, ecart-type', a['err'], c['err']))
        print('  %-26s %12.0f %12.0f  pas/s' % ('commande, ecart-type', a['sps'], c['sps']))
        print('  %-26s %12.0f %12.0f  mm' % ('derive de position', a['deriv'], c['deriv']))
        print('  %-26s %12.0f %12.0f  /s' % ('inversions de commande', a['inv'], c['inv']))
        print('  %-26s %12.4f %12.4f' % ('rugosite, RMS de d2a', a['rug'], c['rug']))
        print('  %-26s %12.0f %12.0f' % ('recompense', a['recompense'], c['recompense']))
        print()


# ---------------------------------------------------------------- regarder
def regarder(pilote, difficulte, duree=120.0):
    env = Balancier(actionneur=ACTIONNEUR, difficulte=difficulte)
    o, _ = env.reset()
    print()
    print('  Le robot suit une consigne de vitesse tiree au hasard toutes les')
    print('  2 a 4 s, et se fait bousculer. Ferme la fenetre pour arreter.')
    print()
    dt = 1.0 / HZ_POLITIQUE
    t_aff = 0.0
    with mujoco.viewer.launch_passive(env.m, env.d) as vue:
        while vue.is_running() and env.d.time < duree:
            debut = time.time()
            o, r, tombe, fini, info = env.step(pilote(o, env))
            if env.d.time >= t_aff:
                print('  t %6.1f s   tangage %+6.2f d   v %+6.3f / consigne %+6.3f m/s'
                      '   lacet %+6.1f d/s'
                      % (env.d.time, info['tangage'], info['v'], env.consigne[0],
                         math.degrees(env.consigne[1])))
                t_aff += 1.0
            if tombe:
                print('  >>> CHUTE a t = %.2f s, on relance' % env.d.time)
                o, _ = env.reset(); t_aff = 0.0
            elif fini:
                o, _ = env.reset(); t_aff = 0.0
            vue.sync()
            retard = dt - (time.time() - debut)
            if retard > 0:
                time.sleep(retard)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent', type=str, default=None)
    ap.add_argument('--cascade', action='store_true', help='regarder la cascade a la place')
    ap.add_argument('--duel', action='store_true')
    ap.add_argument('--film', action='store_true')
    ap.add_argument('--difficulte', type=float, default=0.5)
    ap.add_argument('--actionneur', choices=('vitesse', 'couple'), default='couple',
                    help="'couple' = moteur.py, le ressort magnetique recale sur les "
                         "71 Hz mesures. C est le defaut depuis le 9 sept 2026 : "
                         "juger un agent sur l actionneur en vitesse revient a le "
                         "juger sur un robot qui n existe pas.")
    a = ap.parse_args()
    ACTIONNEUR = a.actionneur

    chemin = a.agent or dernier_agent()

    if a.duel:
        duel(chemin)
    elif a.film:
        from stable_baselines3 import PPO
        import importlib
        p7 = importlib.import_module('07_ppo') if False else None
        mdl = PPO.load(chemin, device='cpu')
        import sys
        sys.argv = ['x']
        exec(open('07_ppo.py', encoding='utf-8').read().split(
            "if __name__ == '__main__':")[0], globals())
        filmer(mdl, 0, a.difficulte)
    else:
        pilote = pilote_cascade() if a.cascade else pilote_agent(chemin)
        regarder(pilote, a.difficulte)
