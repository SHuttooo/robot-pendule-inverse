# -*- coding: utf-8 -*-
"""
05 - Bac a sable : c est toi qui pousses.

Deux facons, complementaires.

  A LA SOURIS -- tu choisis OU, au pixel pres.
      double-clic sur une piece pour la selectionner,
      puis Ctrl + clic DROIT + glisser pour tirer dessus.
      Le point d application est celui que tu as clique, et MuJoCo dessine
      la fleche de force. Ctrl + clic GAUCHE applique un couple.

  AU CLAVIER -- tu choisis QUAND et COMBIEN, de facon reproductible.
      P / M      pousser vers l avant / l arriere
      O / L      augmenter / diminuer la force
      H          changer le point d application (haut, milieu, essieu)
      D          changer la duree de la poussee
      G / K      angle seul / cascade complete   (comme tes commandes serie)
      R          remettre le robot debout
      N          couper ou remettre le bruit des capteurs
      ?          reafficher cette aide

Chaque poussee au clavier est CHRONOMETREE : angle maximum atteint, temps de
retour sous 0,5 deg, derive. C est le meme protocole que 04_pousser.py, donc
les chiffres sont comparables.

Attention : le viewer PASSIF expose la perturbation souris mais ne l applique
pas lui-meme. C est mjv_applyPerturbForce, dans la boucle ci-dessous, qui la
transforme en force. Sans cet appel, tirer a la souris ne fait rien.
"""
import math
import time
import numpy as np
import mujoco
import mujoco.viewer
import modele
import etat
from firmware import Firmware, RAD_PAR_PAS

FORCES = [0.1, 0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0]
HAUTEURS = [('haut', 0.210), ('milieu', 0.120), ('essieu', 0.0)]
DUREES = [0.05, 0.10, 0.20, 0.50]

m = mujoco.MjModel.from_xml_path('modeles/balancier.xml')
d = mujoco.MjData(m)

B_CHASSIS = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
DT_CTRL = 1.0 / 200
N_PHYS = round(DT_CTRL / m.opt.timestep)
NUL = np.zeros(3)

# ----------------------------------------------------------------- etat
E = {
    'i_force': 4,          # 0,8 N -- la limite mesuree par 04_pousser.py
    'i_haut': 0,
    'i_duree': 1,          # 100 ms
    'sens': 1.0,
    'fin_poussee': -1.0,   # date de fin de la poussee en cours
    'bruit': True,
    'mesure': None,        # (t0, angle_max, t_retour, x0)
}
fw = Firmware(dual=True)
alea = np.random.default_rng(1)


def aide():
    print()
    print('  ' + '-' * 66)
    print('  SOURIS   double-clic pour selectionner, Ctrl + clic DROIT + glisser')
    print('           pour tirer au point de ton choix. Ctrl + clic GAUCHE = couple.')
    print('  CLAVIER  P/M pousser avant/arriere   O/L force   H point   D duree')
    print('           G/K mode   R redresser   N bruit   ? aide')
    print('  ' + '-' * 66)
    reglage()


def reglage():
    print('  force %.1f N   |   point : %s (%.0f mm)   |   duree %.0f ms   |'
          '   mode %s   |   bruit %s'
          % (FORCES[E['i_force']], HAUTEURS[E['i_haut']][0],
             HAUTEURS[E['i_haut']][1] * 1000, DUREES[E['i_duree']] * 1000,
             'K cascade' if fw.dual else 'G angle seul',
             'oui' if E['bruit'] else 'non'))


def redresser():
    # etat.poser appelle mj_resetData, qui remet d.time a ZERO. Si une poussee etait programmee jusqu a
    # t = 12 s, la condition "d.time < fin_poussee" redevient vraie et la force
    # est reappliquee pendant 12 secondes : le robot repart aussitot. Il faut
    # donc annuler la poussee en cours, pas seulement replacer le robot.
    E['fin_poussee'] = -1.0
    E['mesure'] = None
    d.qfrc_applied[:] = 0
    d.xfrc_applied[:] = 0
    etat.poser(m, d, tangage_deg=0.3)
    fw.__init__(dual=fw.dual)
    fw.pitch = 0.3
    print('  robot redresse.')


def pousser(sens):
    E['sens'] = sens
    E['fin_poussee'] = d.time + DUREES[E['i_duree']]
    E['mesure'] = [d.time, 0.0, None, etat.avance(m, d)]
    print('  >>> %+.1f N a %s pendant %.0f ms   (impulsion %.3f N.s)'
          % (sens * FORCES[E['i_force']], HAUTEURS[E['i_haut']][0],
             DUREES[E['i_duree']] * 1000,
             FORCES[E['i_force']] * DUREES[E['i_duree']]))


def touche(code):
    # Piege : une touche non imprimable (Ctrl, Maj, fleche, F1...) donnait
    # c = '' et "'' in 'GK'" vaut True en Python -- la chaine vide est incluse
    # dans toute chaine. Un simple appui sur Maj basculait donc en mode G et
    # coupait la boucle externe sans rien dire. D ou le tuple, pas la chaine.
    if not (32 <= code < 127):
        return
    c = chr(code)
    if c == 'P':
        pousser(+1.0)
    elif c == 'M':
        pousser(-1.0)
    elif c == 'O':
        E['i_force'] = min(len(FORCES) - 1, E['i_force'] + 1); reglage()
    elif c == 'L':
        E['i_force'] = max(0, E['i_force'] - 1); reglage()
    elif c == 'H':
        E['i_haut'] = (E['i_haut'] + 1) % len(HAUTEURS); reglage()
    elif c == 'D':
        E['i_duree'] = (E['i_duree'] + 1) % len(DUREES); reglage()
    elif c in ('G', 'K'):
        fw.dual = (c == 'K'); reglage()
    elif c == 'R':
        redresser()
    elif c == 'N':
        E['bruit'] = not E['bruit']; reglage()
    elif c == '/':
        aide()


redresser()
aide()
print()

tick = 0
with mujoco.viewer.launch_passive(m, d, key_callback=touche) as vue:
    while vue.is_running():
        debut = time.time()

        bg = modele.V['bruit_gyro'] if E['bruit'] else 0.0
        ba = modele.V['bruit_accel'] if E['bruit'] else 0.0
        fw.imu(etat.gyro(m, d) + alea.normal(0, bg, 3),
               etat.accel(m, d) + alea.normal(0, ba, 3), DT_CTRL)
        fw.inner(DT_CTRL)
        if tick % 5 == 0:                       # boucle externe a 40 Hz
            fw.outer(DT_CTRL * 5, etat.odometrie_pas(m, d, RAD_PAR_PAS))
        d.ctrl[:] = round(fw.wheel_sps) * RAD_PAR_PAS

        # ---- les deux sources de perturbation, remises a zero a chaque pas
        d.qfrc_applied[:] = 0
        d.xfrc_applied[:] = 0

        # La souris. Sans cet appel, tirer ne produit AUCUNE force -- mais
        # mjv_applyPerturbForce est un RESSORT vers une position de reference :
        # si 'active' reste arme apres un relacher, il tire indefiniment et le
        # robot part tout seul. D ou la double condition, et la trace.
        souris = bool(vue.perturb.active) and vue.perturb.select > 0
        if souris:
            mujoco.mjv_applyPerturbForce(m, d, vue.perturb)

        # le clavier : force horizontale au point choisi
        if d.time < E['fin_poussee']:
            point = d.xpos[B_CHASSIS] + np.array([0.0, 0.0, HAUTEURS[E['i_haut']][1]])
            f = np.array([E['sens'] * FORCES[E['i_force']], 0.0, 0.0])
            mujoco.mj_applyFT(m, d, f, NUL, point, B_CHASSIS, d.qfrc_applied)

        for _ in range(N_PHYS):
            mujoco.mj_step(m, d)

        # ---- chronometrage de la poussee en cours
        ang = etat.tangage(m, d)
        if E['mesure'] is not None:
            t0, amax, tret, x0 = E['mesure']
            amax = max(amax, abs(ang))
            if tret is None and d.time > E['fin_poussee'] and abs(ang) < 0.5:
                tret = d.time - t0
                print('      angle max %5.2f deg   retour en %.2f s   derive %+.0f mm'
                      % (amax, tret, (etat.avance(m, d) - x0) * 1000))
            E['mesure'] = [t0, amax, tret, x0]
            if abs(ang) > 30.0:
                print('      IL TOMBE  (angle max %.1f deg)   -- R pour le redresser' % amax)
                E['mesure'] = None

        if tick % 100 == 0:                     # une ligne toutes les 0,5 s
            print('  t %6.1f s   x %+7.1f mm   angle %+6.2f d   sps %+6.0f'
                  '   corr %+5.2f d   [%s]%s'
                  % (d.time, etat.avance(m, d) * 1000, ang, fw.wheel_sps,
                     fw.speed_angle_corr, 'K' if fw.dual else 'G ANGLE SEUL',
                     '   << SOURIS ACTIVE' if souris else ''))
        tick += 1
        vue.sync()
        retard = DT_CTRL - (time.time() - debut)
        if retard > 0:
            time.sleep(retard)

print('  fin.')
