# -*- coding: utf-8 -*-
"""
Les deux modeles de moteur, cote a cote.

  VITESSE   actionneur <velocity> de MuJoCo. Le moteur suit sa consigne, avec
            une erreur proportionnelle a la charge. Simple et rapide.

  COUPLE    la vraie loi du pas-a-pas, calculee par moteur.py a chaque pas de
            physique : C = C_maintien * sin(50 * (theta_cmd - theta_rotor)).
            Le decrochage, l angle de charge, le crantage et la chute de
            couple ne sont pas imposes -- ils sortent de l equation.

On compare sur les criteres du carnet, plus la poussee encaissee.

    python moteur_duel.py
"""
import math
import numpy as np
import mujoco
import modele
from firmware import Firmware, RAD_PAR_PAS
from moteur import PasAPas

DT_CTRL = 1.0 / 200


def simuler(mode, duree=14.0, poussee=0.0, graine=1, hauteur=0.21,
            instant=4.0, duree_poussee=0.10):
    m = mujoco.MjModel.from_xml_string(modele.xml(mode))
    d = mujoco.MjData(m)
    n_phys = round(DT_CTRL / m.opt.timestep)

    def cap(n):
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, n)
        return slice(m.sensor_adr[i], m.sensor_adr[i] + m.sensor_dim[i])

    S_G, S_A, S_R = cap('gyro'), cap('accel'), cap('pos_roue_d')
    jt = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tangage')
    QT, VT = m.jnt_qposadr[jt], m.jnt_dofadr[jt]
    jr = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'roue_d')
    QR, VR = m.jnt_qposadr[jr], m.jnt_dofadr[jr]
    QX = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'glissiere_x')]
    B = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    NUL = np.zeros(3)

    fw = Firmware(dual=True)
    mo = PasAPas() if mode == 'couple' else None
    al = np.random.default_rng(graine)
    bg, ba = modele.V['bruit_gyro'], modele.V['bruit_accel']

    d.qpos[QT] = math.radians(0.3)
    mujoco.mj_forward(m, d)
    fw.pitch = 0.3
    if mo:
        mo.theta_cmd = d.qpos[QR]

    e, sps, x, charge, decroches, tick = [], [], [], [], 0, 0
    while d.time < duree:
        fw.imu(d.sensordata[S_G] + al.normal(0, bg, 3),
               d.sensordata[S_A] + al.normal(0, ba, 3), DT_CTRL)
        err = fw.inner(DT_CTRL)
        if tick % 5 == 0:
            fw.outer(DT_CTRL * 5, (mo.pas_emis if mo else
                                   d.sensordata[S_R][0] / RAD_PAR_PAS))

        d.qfrc_applied[:] = 0
        if instant <= d.time < instant + duree_poussee and poussee:
            pt = d.xpos[B] + np.array([0.0, 0.0, hauteur])
            mujoco.mj_applyFT(m, d, np.array([poussee, 0.0, 0.0]), NUL,
                              pt, B, d.qfrc_applied)

        for _ in range(n_phys):
            if mo:
                mo.avancer(fw.wheel_sps, m.opt.timestep)
                c = mo.couple(d.qpos[QR], d.qvel[VR])
                d.ctrl[:] = c
                if mo.decroche:
                    decroches += 1
            else:
                d.ctrl[:] = round(fw.wheel_sps) * RAD_PAR_PAS
            mujoco.mj_step(m, d)

        if abs(math.degrees(d.qpos[QT])) > 30.0:
            return None                                    # tombe
        if d.time > 3.0:
            e.append(err); sps.append(fw.wheel_sps); x.append(d.qpos[QX] * 1000)
            if mo:
                charge.append(mo.angle_de_charge_deg(d.qpos[QR]))
        tick += 1

    r = {'e': np.std(e), 'emax': np.max(np.abs(e)), 'sps': np.std(sps),
         'bande': np.ptp(x)}
    if mo:
        r['charge'] = np.max(np.abs(charge))
        r['decroche'] = 100.0 * decroches / (tick * n_phys)
    return r


def limite(mode):
    bas, haut = 0.0, 5.0
    while haut - bas > 0.02:
        mil = (bas + haut) / 2
        if simuler(mode, duree=10.0, poussee=mil) is None:
            haut = mil
        else:
            bas = mil
    return bas


if __name__ == '__main__':
    print('=' * 72)
    print('  AU REPOS  (11 s de fenetre calme, bruit capteur actif)')
    print('=' * 72)
    print('  %-26s %10s %10s' % ('', 'VITESSE', 'COUPLE'))
    a = simuler('vitesse')
    b = simuler('couple')
    print('  %-26s %9.3f  %9.3f   deg   (robot reel : 0,064)'
          % ('erreur d angle, ecart-type', a['e'], b['e']))
    print('  %-26s %9.3f  %9.3f   deg' % ('erreur d angle, maximum', a['emax'], b['emax']))
    print('  %-26s %9.0f  %9.0f   pas/s (robot reel : 46)'
          % ('commande, ecart-type', a['sps'], b['sps']))
    print('  %-26s %9.1f  %9.1f   mm' % ('bande de position', a['bande'], b['bande']))
    print()
    print('  Ce que seul le modele fidele sait dire :')
    print('    angle de charge maximum   %.2f deg   (decrochage a 1,80)' % b['charge'])
    print('    temps passe en decrochage %.2f %%' % b['decroche'])

    print()
    print('=' * 72)
    print('  POUSSEE MAXIMALE ENCAISSEE  (100 ms a 210 mm de l axe)')
    print('=' * 72)
    lv, lc = limite('vitesse'), limite('couple')
    print('  modele VITESSE   %.2f N' % lv)
    print('  modele COUPLE    %.2f N   (%+.0f %%)' % (lc, 100 * (lc / lv - 1)))
    print()
    print('  Si les deux tombent au meme endroit, le modele simple suffit pour')
    print('  la commande, et on garde sa vitesse. Sinon il faut le modele fidele.')
