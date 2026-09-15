"""Limite de poussee encaissee, sur le MOTEUR FIDELE.

Les limites du 5 septembre (0,56 N cascade, 0,70 N agent) ont ete mesurees sur
l actionneur en vitesse, qui suppose la roue instantanee. Le scenario des
rendus poussait a 0,64 N en se fiant a elles -- et le nouvel agent est tombe.
Ce script refait la mesure sur le moteur mesure.

    python 17_pousser_agents.py
"""
import numpy as np
from balancier_env import Balancier
from stable_baselines3 import PPO

DUREE_P = 0.15
HAUTEUR = 0.21
INSTANT = 2.0
GRAINES = range(8)


def pilote_agent(chemin):
    mdl = PPO.load(chemin, device='cpu')
    return lambda o, e: mdl.predict(o, deterministic=True)[0]


def pilote_cascade():
    """Meme cablage que film.py : la cascade lit l observation, pas les capteurs
    bruts. C est ce qui rend la comparaison honnete -- meme information pour
    tout le monde."""
    from firmware import Firmware
    from balancier_env import RAD_PAR_PAS, RAYON, MAX_ACCEL
    fw = Firmware(dual=True)

    def f(o, e):
        fw.pitch = float(o[0]) * 10.0
        fw.gyroFilt = float(o[1]) * 100.0
        fw.wheel_sps = float(np.mean(e.sps))
        corr = fw.Kp_spd * (fw.wheel_sps - e.consigne[0] / (RAD_PAR_PAS * RAYON))
        corr += fw.Ki_spd * (float(o[5]) * 0.20 / (RAD_PAR_PAS * RAYON))
        cible = -max(-6.0, min(6.0, corr))
        accel = fw.Kp_a * (fw.pitch - cible) + fw.Kd_a * fw.gyroFilt
        return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1.0, 1.0)
    return f


def tient(pilote, force, graine):
    e = Balancier(difficulte=0.0, avec_consignes=False, graine=graine,
                  actionneur='couple')
    o, _ = e.reset(seed=graine)
    e.hauteur_externe = HAUTEUR
    while e.d.time < 8.0:
        e.force_externe = force if INSTANT <= e.d.time < INSTANT + DUREE_P else 0.0
        o, r, tombe, fini, i = e.step(pilote(o, e))
        if tombe:
            return False
    return True


def limite(pilote, nom):
    basse, haute = 0.0, 2.0
    for _ in range(7):
        mid = 0.5 * (basse + haute)
        n = sum(tient(pilote, mid, g) for g in GRAINES)
        if n >= len(list(GRAINES)) * 0.75:
            basse = mid
        else:
            haute = mid
    print('  %-34s %.2f N' % (nom, basse))
    return basse


if __name__ == '__main__':
    print('  poussee de %.0f ms a %.2f m de l axe, tenue par 6 graines sur 8'
          % (1000 * DUREE_P, HAUTEUR))
    print('  MOTEUR FIDELE (moteur.py, resonance 71 Hz)')
    print()
    limite(pilote_cascade(), 'cascade')
    limite(pilote_agent('agents_v1_vitesse/balancier_9000000_pas.zip'),
           'ancien agent (roue ideale)')
    limite(pilote_agent('agents/balancier_final.zip'),
           'nouvel agent (moteur mesure)')
