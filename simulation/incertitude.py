# -*- coding: utf-8 -*-
"""
Combien mon ignorance coute-t-elle ?

Les masses ne sont pas mesurees mais ESTIMEES. Plutot que de faire comme si
elles etaient justes, on propage l incertitude jusqu a la seule grandeur qui
compte -- la periode du pendule suspendu, donc le taux de divergence.

Deux sorties :
  * la FOURCHETTE de T, c est-a-dire ce que la mesure d etabli va trancher ;
  * le CLASSEMENT des parametres par leur contribution a cette fourchette,
    c est-a-dire ce qu il faut mesurer en premier si on ne mesure qu une chose.

    python incertitude.py
    python incertitude.py --tirages 20000

Methode. m*g*d et I sont ADDITIFS sur les pieces : inutile de recompiler le
modele a chaque tirage. On mesure une fois pour toutes, par piece et pour une
masse unite, sa hauteur et son inertie propre ; ensuite tout est arithmetique.
"""
import argparse
import math
import numpy as np
import mujoco
import modele

G = 9.81

# ------------------------------------------------------- fourchettes plausibles
#   parametre        bas      haut    pourquoi
PLAGES = {
    'masse_piece1':  (0.026,  0.060, 'remplissage entre 10 et 40 %'),
    'masse_piece2':  (0.027,  0.058, 'idem'),
    'masse_piece3':  (0.044,  0.095, 'idem'),
    'masse_piece4':  (0.034,  0.074, 'idem'),
    'masse_moteur':  (0.200,  0.280, '17HS3401 34 mm = 220 g, 17HS4401 40 mm = 280 g'),
    'masse_bat_haut':(0.140,  0.185, '3 x 18650 CONFIRMEES : 45 a 50 g piece, + supports'),
    'h_bat_haut':    (0.196,  0.220, 'posees sur la piece du haut, +-12 mm'),
    'masse_bat_bas': (0.045,  0.065, '1 x 18650 collee, 45 a 50 g + colle'),
    'h_bat_bas':     (0.115,  0.139, 'sous la piece du haut, +-12 mm'),
    'masse_elec':    (0.040,  0.120, 'Labdec + ESP32 + 2 drivers + cablage'),
    'h_elec':        (0.060,  0.160, 'JAMAIS SITUEE -- la seule position encore inventee'),
    'masse_roue':    (0.020,  0.050, 'roue plastique 65x26'),
}


def profil_des_pieces():
    """Pour chaque piece : hauteur du CdM, ecart lateral, inertie propre par
    kilo. Mesure une fois sur le modele reel, en annulant les autres masses."""
    ref = modele.V
    profil = {}
    for cle in [k for k in ref if k.startswith('masse_')]:
        # un modele ou SEULE cette piece pese
        zeros = {k: 1e-9 for k in ref if k.startswith('masse_')}
        zeros[cle] = 1.0
        m = modele.construire(**zeros)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        if cle == 'masse_roue':
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'corps_roue_d')
        mt = m.body_mass[b]
        x, z = m.body_ipos[b][0], m.body_ipos[b][2]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.body_iquat[b])
        R = R.reshape(3, 3)
        Iyy = (R @ np.diag(m.body_inertia[b]) @ R.T)[1, 1]
        profil[cle] = (x, z, Iyy / mt)          # par kilo
    return profil


def periode(v, profil):
    """T du pendule suspendu par l essieu, en secondes."""
    m_tot = md = I = 0.0
    for cle, (x0, z0, iyy) in profil.items():
        mm = v[cle]
        if cle == 'masse_moteur':
            mm *= 2                             # deux moteurs
        if cle == 'masse_roue':
            continue                            # les roues sont sur l axe
        z = v.get(HAUTEURS.get(cle), z0)
        m_tot += mm
        md += mm * z
        I += mm * iyy + mm * (x0 * x0 + z * z)
    w = math.sqrt(G * md / I)
    return 2 * math.pi / w


HAUTEURS = {'masse_bat_haut': 'h_bat_haut',
            'masse_bat_bas': 'h_bat_bas',
            'masse_elec': 'h_elec'}


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--tirages', type=int, default=8000)
    a = ap.parse_args()

    profil = profil_des_pieces()
    nominal = dict(modele.V)
    T0 = periode(nominal, profil)

    print('=' * 74)
    print('  NOMINAL')
    print('=' * 74)
    print('  T = %.3f s      20 oscillations en %.1f s      w = %.2f rad/s'
          % (T0, 20 * T0, 2 * math.pi / T0))
    print()

    # ------------------------------------------------- propagation par tirages
    rng = np.random.default_rng(0)
    ech = np.empty(a.tirages)
    for i in range(a.tirages):
        v = dict(nominal)
        for k, (lo, hi, _) in PLAGES.items():
            v[k] = rng.uniform(lo, hi)
        ech[i] = periode(v, profil)
    q = np.percentile(ech, [5, 25, 50, 75, 95])

    print('=' * 74)
    print('  FOURCHETTE  (%d tirages uniformes sur les plages plausibles)' % a.tirages)
    print('=' * 74)
    print('   5 %%   %.3f s   ->  20 oscillations en %.1f s' % (q[0], 20 * q[0]))
    print('  25 %%   %.3f s                        %.1f s' % (q[1], 20 * q[1]))
    print('  50 %%   %.3f s                        %.1f s   <- mediane' % (q[2], 20 * q[2]))
    print('  75 %%   %.3f s                        %.1f s' % (q[3], 20 * q[3]))
    print('  95 %%   %.3f s                        %.1f s' % (q[4], 20 * q[4]))
    print()
    print('  Etendue a 90 %% : %.1f a %.1f s pour 20 oscillations, soit %+.0f / %+.0f %%'
          % (20 * q[0], 20 * q[4], 100 * (q[0] / T0 - 1), 100 * (q[4] / T0 - 1)))
    print()

    # ------------------------------------- sensibilite : un parametre a la fois
    print('=' * 74)
    print('  QUOI MESURER EN PREMIER  (chaque parametre balaye seul)')
    print('=' * 74)
    lignes = []
    for k, (lo, hi, pourquoi) in PLAGES.items():
        v = dict(nominal); v[k] = lo; tb = periode(v, profil)
        v = dict(nominal); v[k] = hi; th = periode(v, profil)
        lignes.append((abs(th - tb) / T0 * 100, k, tb, th, pourquoi))
    lignes.sort(reverse=True)
    print('  %-16s %8s %8s  %6s  %s' % ('parametre', 'T bas', 'T haut', 'effet', ''))
    print('  ' + '-' * 70)
    for eff, k, tb, th, pourquoi in lignes:
        barre = '#' * max(1, int(round(eff * 2.2)))
        print('  %-16s %7.3fs %7.3fs  %5.1f %%  %s' % (k, tb, th, eff, barre))
        print('  %-16s %s' % ('', pourquoi))
    print()
    tot = sum(l[0] for l in lignes)
    trois = sum(l[0] for l in lignes[:3])
    print('  Les trois premiers pesent %.0f %% de l incertitude totale.' % (100 * trois / tot))
    print('  Mesure-les, laisse le reste estime : le gain marginal est ailleurs.')
