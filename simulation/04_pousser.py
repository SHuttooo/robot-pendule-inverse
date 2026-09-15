# -*- coding: utf-8 -*-
"""
04 - Le pousser, en newtons.

Une poussee se decrit par trois nombres : une FORCE, une DUREE et un POINT
D APPLICATION. Ce qui compte vraiment est leur produit, l impulsion
J = F x dt en N.s : c est la quantite de mouvement injectee. Pousser 10 N
pendant 50 ms ou 1 N pendant 500 ms, le robot voit la meme chose.

    python 04_pousser.py --force 3            un essai
    python 04_pousser.py --force 3 --vue      et on regarde
    python 04_pousser.py --balayage           jusqu ou il tient ?
    python 04_pousser.py --balayage --max-sps 2400
                                              ce que vaudrait la carte reparee

MuJoCo applique une force externe par mj_applyFT, qui prend un point
d application quelconque : la force et le couple de bras de levier sont
calcules pour toi. C est la bonne facon de simuler un doigt qui pousse.
"""
import argparse
import math
import time
import numpy as np
import mujoco
import mujoco.viewer
import etat
from firmware import Firmware, RAD_PAR_PAS

ap = argparse.ArgumentParser()
ap.add_argument('--force', type=float, default=3.0, help='newtons, vers l avant')
ap.add_argument('--duree-poussee', type=float, default=0.10, help='s')
ap.add_argument('--hauteur', type=float, default=0.21,
                help='m au-dessus de l axe des roues. Le plateau du haut est a'
                     ' 0,199 et les piles montent a 0,217 : 0,21 = tout en haut')
ap.add_argument('--instant', type=float, default=2.0, help='s')
ap.add_argument('--duree', type=float, default=10.0)
ap.add_argument('--max-sps', type=float, default=1600.0)
ap.add_argument('--balayage', action='store_true')
ap.add_argument('--vue', action='store_true')
ap.add_argument('--propre', action='store_true', help='capteurs parfaits')
a = ap.parse_args()

import modele as _mod
BRUIT_GYRO = 0.0 if a.propre else _mod.V['bruit_gyro']
BRUIT_ACCEL = 0.0 if a.propre else _mod.V['bruit_accel']

m = mujoco.MjModel.from_xml_path('modeles/balancier.xml')
B_CHASSIS = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
DT_CTRL = 1.0 / 200
N_PHYS = round(DT_CTRL / m.opt.timestep)
NUL = np.zeros(3)


def essai(force, vue=None, graine=1):
    """Retourne (tombe, angle_max, temps_retour, derive_mm)."""
    d = mujoco.MjData(m)
    fw = Firmware(dual=True)
    fw.MAX_SPS = a.max_sps
    alea = np.random.default_rng(graine)

    etat.poser(m, d, tangage_deg=0.3)
    fw.pitch = 0.3

    t_fin_poussee = a.instant + a.duree_poussee
    angle_max, t_retour, tombe, tick = 0.0, None, False, 0
    x0 = None

    while d.time < a.duree:
        debut = time.time()
        fw.imu(etat.gyro(m, d) + alea.normal(0, BRUIT_GYRO, 3),
               etat.accel(m, d) + alea.normal(0, BRUIT_ACCEL, 3), DT_CTRL)
        fw.inner(DT_CTRL)
        if tick % 5 == 0:
            fw.outer(DT_CTRL * 5, etat.odometrie_pas(m, d, RAD_PAR_PAS))
        d.ctrl[:] = round(fw.wheel_sps) * RAD_PAR_PAS

        # --- la poussee : force horizontale, appliquee a --hauteur du sol
        d.qfrc_applied[:] = 0
        if a.instant <= d.time < t_fin_poussee:
            point = d.xpos[B_CHASSIS] + np.array([0.0, 0.0, a.hauteur])
            mujoco.mj_applyFT(m, d, np.array([force, 0.0, 0.0]), NUL,
                              point, B_CHASSIS, d.qfrc_applied)

        for _ in range(N_PHYS):
            mujoco.mj_step(m, d)

        ang = etat.tangage(m, d)
        if d.time >= a.instant:
            if x0 is None:
                x0 = etat.avance(m, d)
            angle_max = max(angle_max, abs(ang))
            if t_retour is None and d.time > t_fin_poussee and abs(ang) < 0.5:
                t_retour = d.time - a.instant
        if abs(ang) > 30.0:
            tombe = True
            break

        tick += 1
        if vue is not None:
            vue.sync()
            r = DT_CTRL - (time.time() - debut)
            if r > 0:
                time.sleep(r)

    derive = (etat.avance(m, d) - x0) * 1000 if x0 is not None else 0.0
    return tombe, angle_max, t_retour, derive


def equivalent(J):
    """Rendre une impulsion parlante."""
    return ('%.3f N.s  =  une gomme de 20 g lancee a %.1f m/s'
            '  =  une chiquenaude de %.1f N pendant 0,1 s' % (J, J / 0.020, J / 0.1))


if __name__ == '__main__':
    print('  point d application : %.0f mm au-dessus de l axe des roues' % (a.hauteur * 1000))
    print('  duree de la poussee : %.0f ms      MAX_SPS = %.0f' % (a.duree_poussee * 1000, a.max_sps))
    print()

    if not a.balayage:
        vue = mujoco.viewer.launch_passive(m, mujoco.MjData(m)) if a.vue else None
        if a.vue:
            vue.close()
            vue = None
            print('  (le viewer suit dans une seconde fenetre)')
        J = a.force * a.duree_poussee
        print('  poussee de %.1f N  ->  impulsion %s' % (a.force, equivalent(J)))
        print()
        tombe, amax, tret, der = essai(a.force, None)
        if tombe:
            print('  >>> IL TOMBE.')
        else:
            print('  angle maximum atteint    %6.2f deg' % amax)
            print('  retour sous 0,5 deg en   %6s' % ('%.2f s' % tret if tret else 'jamais'))
            print('  derive de position       %6.0f mm' % der)
        raise SystemExit

    # ------------------------------------------------ balayage : la limite
    print('  %7s %10s  %9s %9s %8s  %s'
          % ('force', 'impulsion', 'angle max', 'retour', 'derive', ''))
    print('  ' + '-' * 70)
    for f in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0):
        tombe, amax, tret, der = essai(f)
        if tombe:
            print('  %5.1f N %9.3f N.s  %s' % (f, f * a.duree_poussee, 'IL TOMBE'))
            break
        print('  %5.1f N %9.3f N.s  %7.2f d %8s %6.0f mm  %s'
              % (f, f * a.duree_poussee, amax,
                 '%.2f s' % tret if tret else 'jamais', der,
                 '#' * max(1, int(amax * 2))))

    # dichotomie : la vraie limite, a 1 % pres
    bas, haut = 0.0, 10.0
    while haut - bas > 0.01:
        mil = (bas + haut) / 2
        if essai(mil)[0]:
            haut = mil
        else:
            bas = mil
    J = bas * a.duree_poussee
    print()
    print('  LIMITE = %.2f N pendant %.0f ms,  impulsion %.3f N.s'
          % (bas, a.duree_poussee * 1000, J))
    print('  %s' % equivalent(J))
    print()

    # ------------------------------- ce que vaudrait la carte reparee
    if a.max_sps <= 1600:
        v_max = a.max_sps * RAD_PAR_PAS * _mod.V['rayon_roue']
        print('  POURQUOI SI PEU ? La vitesse de roue plafonne a %.0f pas/s,'
              % a.max_sps)
        print('  soit %.3f m/s seulement avec tes roues de 65 mm. Pour rattraper'
              % v_max)
        print('  une chute il faut deplacer le point d appui VITE ; ce plafond est')
        print('  la vraie limite, pas les gains.')
        print()
        ancien = bas
        for sps in (2400.0, 2667.0):
            a.max_sps = sps
            b2, h2 = 0.0, 10.0
            while h2 - b2 > 0.01:
                mm = (b2 + h2) / 2
                if essai(mm)[0]:
                    h2 = mm
                else:
                    b2 = mm
            print('  MAX_SPS %.0f  ->  limite %.2f N   (%+.0f %%)   %.3f m/s'
                  % (sps, b2, 100 * (b2 / ancien - 1), sps * RAD_PAR_PAS * _mod.V['rayon_roue']))
        print()
        print('  Voila ce que vaut la reparation de la carte, chiffre.')
    print()
    print('  Ce chiffre est la reference a battre pour l agent RL : meme')
    print('  protocole, meme point d application, meme duree.')
