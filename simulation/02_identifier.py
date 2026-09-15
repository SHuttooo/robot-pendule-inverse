# -*- coding: utf-8 -*-
"""
02 - Identifier le modele.

Un modele qui a l air juste ne sert a rien. Celui-ci doit reproduire des
chiffres. Ce script mesure, il ne regarde pas.

PHYSIQUE. Autour de l equilibre instable : theta'' = w^2 * theta.
Lache a theta0 sans vitesse -> theta(t) = theta0 * cosh(w*t).
En chronometrant le passage a theta1 :   w = arccosh(theta1/theta0) / t.

Un pendule inverse N OSCILLE PAS : il diverge. w n est pas une frequence,
c est l inverse d un temps de chute. On l exprime quand meme en Hz pour
pouvoir le comparer aux oscillations observees.

    python 02_identifier.py
    python 02_identifier.py --balayage        # effet de la hauteur de batterie
"""
import argparse
import math
import mujoco

G = 9.81
F_CARNET = 1.35          # Hz, oscillation relevee en boucle fermee sur le vrai robot


def charger(h_batterie=None):
    # Modele PLAN : mesurer un mode propre est un probleme a un seul degre de
    # liberte, autant ne pas payer les six de l articulation libre.
    import modele
    kw = {'h_bat_haut': h_batterie} if h_batterie is not None else {}
    return modele.construire(dimension='plan', **kw)


def geometrie_des_masses(m):
    """Ce que MuJoCo a deduit des geoms. On ne saisit jamais une inertie a la
    main : on saisit des formes et des masses, le compilateur fait le reste."""
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    m_ch = m.body_mass[ch]
    z_com = m.body_ipos[ch][2]        # CdM dans le repere du corps -> origine = essieu
    I_com = m.body_inertia[ch][1]     # inertie principale, axe de tangage = y
    I_essieu = I_com + m_ch * z_com**2                 # Huygens
    L_eq = I_essieu / (m_ch * z_com)                   # pendule simple equivalent

    print('  masse chassis              %8.3f kg' % m_ch)
    print('  masse totale               %8.3f kg' % m.body_mass.sum())
    print('  hauteur du CdM / essieu    %8.1f mm' % (z_com * 1000))
    print('  inertie au CdM (axe y)     %8.5f kg.m2' % I_com)
    print('  inertie a l essieu         %8.5f kg.m2' % I_essieu)
    print('  L_eq = I/(m*d)             %8.1f mm' % (L_eq * 1000))
    print()
    w = math.sqrt(m_ch * G * z_com / I_essieu)
    print('  >>> PREDICTION DE LA MESURE D ETABLI (MESURES.md, section 1)')
    print('      Robot suspendu par son essieu, roues bloquees, ecart de 5 a 10 deg :')
    print('        periode                 T = %.3f s   -> 20 oscillations en %.1f s'
          % (2 * math.pi / w, 40 * math.pi / w))
    print('      Retourne, il diverge a w = 2*pi/T = %.2f rad/s. Meme groupement' % w)
    print('      m*g*d/I : la periode suspendue DONNE le taux de divergence.')
    print()
    print('  Hauteur du CdM et L_eq ne sont pas la meme grandeur. Le carnet')
    print('  deduit L = g/(2*pi*f)^2 = 136 mm : cette formule donne L_eq, la')
    print('  longueur du pendule PONCTUEL equivalent. Pour un corps reel')
    print('  L_eq est toujours plus grand que la hauteur du CdM.')


def mesurer_w(m, mode, theta0=0.5, theta1=10.0):
    """Chronometre la chute de theta0 a theta1 (deg). Retourne w en rad/s.

    mode 'libre'  : couple moteur nul. C est le pole instable que la commande
                    doit stabiliser -- le chiffre qui compte.
    mode 'bloque' : roues solidaires du chassis. Le robot bascule en bloc,
                    en roulant. Plus lent : la translation ajoute de l inertie.
    """
    if mode == 'libre':
        import modele
        m = modele.construire(dimension='plan')            # copie propre
    d = mujoco.MjData(m)
    if mode == 'libre':
        m.actuator_gainprm[:, :] = 0      # actionneurs neutralises : couple nul
        m.actuator_biasprm[:, :] = 0
    else:
        d.eq_active[:] = 1                # les deux freins

    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tangage')
    adr = m.jnt_qposadr[j]
    d.qpos[adr] = math.radians(theta0)
    mujoco.mj_forward(m, d)

    while d.time < 5.0:
        mujoco.mj_step(m, d)
        if d.qpos[adr] >= math.radians(theta1):
            return math.acosh(theta1 / theta0) / d.time
    return float('nan')


def ligne(nom, w):
    print('  %-30s  w = %5.2f rad/s   soit %5.2f Hz   chute 1->10 deg en %4.0f ms'
          % (nom, w, w / (2 * math.pi), 1000 * math.acosh(10.0) / w))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--h-batterie', type=float, default=None)
    ap.add_argument('--balayage', action='store_true')
    a = ap.parse_args()

    if a.balayage:
        print('  h batterie      w libre        w bloque')
        for h in (0.06, 0.10, 0.14, 0.168, 0.20, 0.24, 0.30):
            mm = charger(h)
            print('   %5.0f mm      %6.2f rad/s    %6.2f rad/s'
                  % (h * 1000, mesurer_w(mm, 'libre'), mesurer_w(mm, 'bloque')))
        print()
        print('  Monter la batterie ralentit la chute. C est exactement la')
        print('  recommandation "monter le centre de masse" du carnet, chiffree.')
        raise SystemExit

    m = charger(a.h_batterie)

    print('=' * 72)
    print('GEOMETRIE DES MASSES   (deduite des geoms par le compilateur MuJoCo)')
    print('=' * 72)
    geometrie_des_masses(m)
    print()
    print('=' * 72)
    print('MODE PROPRE   (chronometrage de la chute, 0,5 deg -> 10 deg)')
    print('=' * 72)
    w_libre = mesurer_w(m, 'libre')
    ligne('roues libres (couple nul)', w_libre)
    ligne('roues bloquees', mesurer_w(m, 'bloque'))
    print()
    print('  Ces deux-la ne sont pas la prediction ci-dessus : ici le robot est')
    print('  pose au sol, il peut translater. Le pivot suspendu est l essieu,')
    print('  le pivot au sol se deplace. Trois grandeurs, trois montages.')
    print()
    print('=' * 72)
    print('CONFRONTATION AU CARNET')
    print('=' * 72)
    print("""
  Le carnet donne 1,35 Hz, mesure sur les passages par zero de e PENDANT
  que le robot equilibrait. C est une oscillation en BOUCLE FERMEE : sa
  frequence est fixee par le regulateur autant que par la mecanique. La
  convertir en longueur avec L = g/(2*pi*f)^2 est une estimation, pas une
  mesure -- utile pour dimensionner, insuffisante pour valider un modele.

  Le modele annonce %.1f rad/s roues libres, soit %.2f Hz. L ecart avec 1,35
  n est donc pas forcement une erreur de modele.

  POUR TRANCHER, sur le vrai robot -- l essai coute deux minutes :
    1. desarmer le PID          -> commande S
    2. caler le robot a la main a environ 1 deg de l equilibre
    3. lacher, et relever dans le log l instant ou P passe par 5 puis 10 deg
    4. w = arccosh(10/1) / t(10 deg)     [~%.0f ms attendus ici]
  Le log est deja horodate a la milliseconde par serial_monitor.ps1, et le
  champ P suffit. Ce chiffre-la est le pole instable, il ne depend d aucun
  gain, et c est lui qui valide ou invalide ce modele.

  Ensuite : python 02_identifier.py --balayage pour trouver la hauteur de
  batterie qui reproduit ta mesure.
""" % (w_libre, w_libre / (2 * math.pi), 1000 * math.acosh(10.0) / w_libre))
