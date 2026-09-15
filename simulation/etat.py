# -*- coding: utf-8 -*-
"""
Lire l etat du robot -- par les CAPTEURS, jamais par qpos.

Deux raisons, et la seconde est la vraie.

  1. Les scripts deviennent independants du modele. En plan le tangage est une
     charnière, en 3D c est un quaternion d articulation libre. Les capteurs,
     eux, repondent pareil dans les deux cas.

  2. C est ce que fait le vrai robot. Il n a pas acces a qpos ; il a un gyro,
     un accelerometre et un compteur de pas. Prendre l habitude de passer par
     les capteurs, c est se rendre incapable de tricher -- et la triche par
     information privilegiee est LA facon de rater un transfert sim-to-real.

Les capteurs 'orientation' et 'position' font exception : ce sont des verites
terrain, reservees a la MESURE et aux conditions de fin d episode. Jamais dans
l observation d un agent.
"""
import math
import numpy as np
import mujoco

_CACHE = {}


def _idx(m):
    """Adresses des capteurs, calculees une seule fois par modele."""
    k = id(m)
    if k not in _CACHE:
        c = {}
        for nom in ('gyro', 'accel', 'pos_roue_d', 'pos_roue_g',
                    'orientation', 'position'):
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, nom)
            c[nom] = slice(m.sensor_adr[i], m.sensor_adr[i] + m.sensor_dim[i])
        c['3d'] = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'libre') >= 0
        _CACHE[k] = c
    return _CACHE[k]


def _rot(m, d):
    """Matrice de rotation du chassis, depuis le capteur d orientation."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, d.sensordata[_idx(m)['orientation']])
    return R.reshape(3, 3)


# ---------------------------------------------------------------- angles
#
#  Tangage et roulis se mesurent dans le repere DU ROBOT, pas dans celui du
#  monde -- sinon un robot qui a tourne de 30 deg verrait son inclinaison
#  avant-arriere se repartir entre tangage et roulis. On projette donc la
#  gravite dans le repere du chassis : c est exactement ce que lit un
#  accelerometre, et c est independant du cap.
def _gravite_locale(m, d):
    return _rot(m, d).T @ np.array([0.0, 0.0, -1.0])


def tangage(m, d):
    """deg. > 0 = penche vers l avant. Independant du cap."""
    g = _gravite_locale(m, d)
    return math.degrees(math.atan2(g[0], -g[2]))


def roulis(m, d):
    """deg. > 0 = penche sur le cote. Toujours 0 en modele plan."""
    g = _gravite_locale(m, d)
    return math.degrees(math.atan2(-g[1], -g[2]))


def lacet(m, d):
    """deg. Le cap. N existe qu en 3D."""
    xb = _rot(m, d)[:, 0]
    return math.degrees(math.atan2(xb[1], xb[0]))


# ------------------------------------------------------------ positions
def position(m, d):
    """m, dans le repere du monde."""
    return np.array(d.sensordata[_idx(m)['position']])


def avance(m, d):
    """m parcourus vers l avant depuis l origine (projection sur x)."""
    return float(d.sensordata[_idx(m)['position']][0])


# --------------------------------------------------------------- roues
def roues_rad(m, d):
    """Angles des deux roues, en radians."""
    c = _idx(m)
    return float(d.sensordata[c['pos_roue_d']][0]), float(d.sensordata[c['pos_roue_g']][0])


def odometrie_pas(m, d, rad_par_pas):
    """La moyenne des deux roues, en pas. L equivalent de stepCount."""
    a, b = roues_rad(m, d)
    return (a + b) / 2.0 / rad_par_pas


# --------------------------------------------------------------- gyro
def gyro(m, d):
    return np.array(d.sensordata[_idx(m)['gyro']])


def accel(m, d):
    return np.array(d.sensordata[_idx(m)['accel']])


# ------------------------------------------------------------- reglages
def poser(m, d, tangage_deg=0.3, lacet_deg=0.0):
    """Remet le robot debout, incline de tangage_deg. Marche dans les deux
    modeles : en 3D on ecrit le quaternion, en plan la charnière."""
    mujoco.mj_resetData(m, d)
    if _idx(m)['3d']:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'libre')
        adr = m.jnt_qposadr[j]
        qt = np.zeros(4)
        mujoco.mju_axisAngle2Quat(qt, np.array([0.0, 1.0, 0.0]),
                                  math.radians(tangage_deg))
        ql = np.zeros(4)
        mujoco.mju_axisAngle2Quat(ql, np.array([0.0, 0.0, 1.0]),
                                  math.radians(lacet_deg))
        q = np.zeros(4)
        mujoco.mju_mulQuat(q, ql, qt)
        d.qpos[adr + 3:adr + 7] = q
    else:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tangage')
        d.qpos[m.jnt_qposadr[j]] = math.radians(tangage_deg)
    mujoco.mj_forward(m, d)


def bousculer(m, d, deg_par_s):
    """Injecte une vitesse de tangage, en deg/s. Pour les essais rapides ;
    04_pousser.py applique une vraie force en newtons."""
    if _idx(m)['3d']:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'libre')
        d.qvel[m.jnt_dofadr[j] + 4] += math.radians(deg_par_s)   # omega_y
    else:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tangage')
        d.qvel[m.jnt_dofadr[j]] += math.radians(deg_par_s)


if __name__ == '__main__':
    import modele
    for dim in ('plan', '3d'):
        m = modele.construire(dimension=dim)
        d = mujoco.MjData(m)
        poser(m, d, tangage_deg=7.0, lacet_deg=(0.0 if dim == 'plan' else 30.0))
        print('  %-5s  tangage %+6.2f   roulis %+6.2f   lacet %+7.2f   z %.4f m'
              % (dim, tangage(m, d), roulis(m, d), lacet(m, d), position(m, d)[2]))
