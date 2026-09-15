# -*- coding: utf-8 -*-
"""
01 - Regarder.

Le minimum vital de MuJoCo. Trois objets, jamais plus :

    MjModel   ce qui ne change pas : masses, geometries, actionneurs.
              Compile depuis le XML une fois pour toutes.
    MjData    ce qui change : qpos, qvel, ctrl, time, capteurs.
    mj_step   avance la physique d un timestep (ici 0,5 ms).

    python 01_regarder.py              # il tombe, personne ne le rattrape
    python 01_regarder.py --pousser    # il tombe apres une poussee
"""
import argparse
import time
import mujoco
import mujoco.viewer

ap = argparse.ArgumentParser()
ap.add_argument('--pousser', action='store_true')
ap.add_argument('--mesh', action='store_true',
                help='superpose les STL de l assemblage (decoratif). Touche 0/1/2'
                     ' dans le viewer pour basculer entre les groupes de geoms.')
a = ap.parse_args()

if a.mesh:
    import modele
    m = modele.construire(mesh=True)
else:
    m = mujoco.MjModel.from_xml_path('modeles/balancier.xml')
d = mujoco.MjData(m)

# Les index se cherchent par nom une seule fois, jamais dans la boucle.
J_TANGAGE = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'tangage')]

d.qpos[J_TANGAGE] = 0.02          # rad : on le pose presque droit

print(__doc__)
print('  souris gauche  : tourner la camera')
print('  molette        : zoomer')
print('  ctrl + gauche  : pousser le robot')
print('  espace         : pause     |     barre de droite : capteurs, contacts...')
if a.mesh:
    print()
    print('  MODE MESH. Les STL sont dans le groupe 2, les primitives dans le 0.')
    print('  Touches 0 et 2 : afficher / masquer chaque groupe, pour verifier')
    print('  que les boites tombent bien dans le maillage.')
    print('  Recaler : editer MAILLAGES dans modele.py, relancer.')
print()

with mujoco.viewer.launch_passive(m, d) as vue:
    t0 = time.time()
    while vue.is_running():
        debut = time.time()

        if a.pousser and 1.0 < d.time < 1.05:
            d.qvel[J_TANGAGE] += 0.5      # coup de pouce

        mujoco.mj_step(m, d)
        vue.sync()

        # On cale la simulation sur le temps reel, sinon elle defile a 50x.
        retard = m.opt.timestep - (time.time() - debut)
        if retard > 0:
            time.sleep(retard)

print('  duree simulee : %.2f s' % d.time)
