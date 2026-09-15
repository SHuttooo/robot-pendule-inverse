# -*- coding: utf-8 -*-
"""
03 - Ton firmware, dans la simulation.

Portage direct de robot_balancier.ino : memes gains, memes unites (degres et
pas/s), meme cascade, meme telemetrie. Rien n est "adapte a la simu".

  boucle interne  200 Hz   accel = Kp*err + Ki*integ + Kd*gyroFilt   [pas/s2]
                           wheel_sps += accel*dt   <- l integrateur EST la commande
  boucle externe   40 Hz   target_angle = angleOffset - (Kp_spd*err_v + Ki_spd*err_p)
  angle                    filtre complementaire sur gyro et accelerometre SIMULES

Seul ecart avec le vrai robot : ici la verticale vaut 0 deg. Sur le tien elle
vaut -91,4 deg parce que le MPU est monte couche. Tout le reste est identique,
y compris le format des lignes de log.

    python 03_pid.py                    # cascade complete, 20 s
    python 03_pid.py --vue              # avec le viewer
    python 03_pid.py --mode G           # boucle interne seule (il derive)
    python 03_pid.py --pousser 40       # poussee de 40 deg/s a t = 5 s
    python 03_pid.py --tau-speed 0.15   # L ANCIEN reglage : coupure a 1,06 Hz
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
ap.add_argument('--vue', action='store_true')
ap.add_argument('--mode', choices=['G', 'K'], default='K',
                help='G = angle seul, K = cascade complete (commandes serie)')
ap.add_argument('--duree', type=float, default=20.0)
ap.add_argument('--pousser', type=float, default=0.0, help='deg/s injectes a t=5 s')
ap.add_argument('--tau-speed', type=float, default=0.40)
ap.add_argument('--kd', type=float, default=600.0)
ap.add_argument('--silence', action='store_true')
ap.add_argument('--propre', action='store_true',
                help='capteurs parfaits (ancien comportement, irrealiste)')
ap.add_argument('--bruit-gyro', type=float, default=None, help='rad/s, ecart-type')
ap.add_argument('--bruit-accel', type=float, default=None, help='m/s2, ecart-type')
a = ap.parse_args()


# ============================================================= la simulation
m = mujoco.MjModel.from_xml_path('modeles/balancier.xml')
d = mujoco.MjData(m)


# ---- Le bruit des capteurs. Sans lui le robot a l air POSE : la simulation
# est trop propre, l erreur d angle tombe a 0,003 deg contre 0,064 mesures.
# Les pas-a-pas font vibrer le chassis, l accelerometre prend chaque vibration
# pour une acceleration reelle, et le gyroscope a son propre plancher de bruit.
import modele as _mod
BRUIT_GYRO = 0.0 if a.propre else (a.bruit_gyro if a.bruit_gyro is not None
                                   else _mod.V['bruit_gyro'])
BRUIT_ACCEL = 0.0 if a.propre else (a.bruit_accel if a.bruit_accel is not None
                                    else _mod.V['bruit_accel'])
alea = np.random.default_rng(1)

fw = Firmware(dual=(a.mode == 'K'))
fw.TAU_SPEED = a.tau_speed
fw.Kd_a = a.kd

DT_CTRL = 1.0 / 200                     # cadence de l INT du MPU
N_PHYS = round(DT_CTRL / m.opt.timestep)
DT_OUTER = DT_CTRL * 5                  # 40 Hz

etat.poser(m, d, tangage_deg=0.3)       # pose main levee, presque droit
fw.pitch = 0.3

journal = []
tick = 0
vue = mujoco.viewer.launch_passive(m, d) if a.vue else None

if not a.silence:
    print('  capteurs : bruit gyro %.4f rad/s, accel %.3f m/s2'
          % (BRUIT_GYRO, BRUIT_ACCEL))
    print('  P angle | tgt cible | e erreur | g gyro | sps commande roue')
    print('  c correction externe | pos odometrie (pas) | x position reelle (mm)')
    print()

while d.time < a.duree:
    debut = time.time()

    if a.pousser and abs(d.time - 5.0) < DT_CTRL / 2:
        etat.bousculer(m, d, a.pousser)

    # --- capteurs bruites, puis les deux boucles
    fw.imu(etat.gyro(m, d) + alea.normal(0, BRUIT_GYRO, 3),
           etat.accel(m, d) + alea.normal(0, BRUIT_ACCEL, 3), DT_CTRL)
    err = fw.inner(DT_CTRL)
    if tick % 5 == 0:
        fw.outer(DT_OUTER, etat.odometrie_pas(m, d, RAD_PAR_PAS))

    # --- la commande : pas/s -> rad/s. Un seul timer, donc une seule valeur.
    d.ctrl[:] = round(fw.wheel_sps) * RAD_PAR_PAS   # le timer ne fait que des pas entiers

    # Le viewer passif EXPOSE la perturbation souris mais ne l applique pas :
    # sans cet appel, Ctrl + glisser ne produirait aucune force.
    d.xfrc_applied[:] = 0
    if vue is not None and vue.perturb.active:
        mujoco.mjv_applyPerturbForce(m, d, vue.perturb)

    for _ in range(N_PHYS):
        mujoco.mj_step(m, d)

    journal.append((d.time, fw.pitch, err, fw.gyroFilt, fw.wheel_sps,
                    fw.speed_angle_corr, etat.avance(m, d) * 1000))

    if not a.silence and tick % 20 == 0:                 # une ligne / 100 ms
        print('P:%7.2f tgt:%6.2f e:%6.2f g:%6.1f sps:%7.0f c:%6.2f pos:%7.0f x:%7.1f'
              % (fw.pitch, fw.target_angle, err, fw.gyroFilt, fw.wheel_sps,
                 fw.speed_angle_corr, etat.odometrie_pas(m, d, RAD_PAR_PAS),
                 etat.avance(m, d) * 1000))

    if abs(etat.tangage(m, d)) > 30.0:                   # FALL_LIMIT
        print('\n  !! CHUTE a t = %.2f s   (|angle| > 30 deg)' % d.time)
        break

    tick += 1
    if vue is not None:
        vue.sync()
        retard = DT_CTRL - (time.time() - debut)
        if retard > 0:
            time.sleep(retard)

if vue is not None:
    vue.close()

# ================================================ la meme analyse que le carnet
j = np.array(journal)
calme = j[j[:, 0] > 3.0]                # on jette le transitoire de depart
if len(calme) > 50:
    e, sps, x = calme[:, 2], calme[:, 4], calme[:, 6]
    passages = int(np.sum(np.diff(np.sign(e)) != 0))
    duree = calme[-1, 0] - calme[0, 0]
    print()
    print('=' * 66)
    print('  MESURES sur la fenetre calme  (%.1f s, methode du carnet)' % duree)
    print('=' * 66)
    print('  erreur d angle, ecart-type   %7.3f deg     (robot reel : 0,064)' % e.std())
    print('  erreur d angle, maximum      %7.3f deg' % np.abs(e).max())
    print('  commande moteur, ecart-type  %7.0f pas/s   (robot reel : 46)' % sps.std())
    print('  derive de position           %7.1f mm      (robot reel : 3)'
          % (x.max() - x.min()))
    print('  saturation de vitesse        %7.1f %%'
          % (100 * np.mean(np.abs(sps) >= 0.98 * fw.MAX_SPS)))
    if passages > 4:
        print('  OSCILLATION                  %7.2f Hz      (%d passages par zero)'
              % (passages / (2 * duree), passages))
    else:
        print('  pas d oscillation entretenue (%d passages par zero)' % passages)
