# -*- coding: utf-8 -*-
"""
Rendu hors-ecran : des images et des videos, sans ouvrir de fenetre.

Le viewer interactif (01_regarder.py, 03_pid.py --vue) sert a manipuler.
Celui-ci sert a GARDER une trace : verifier un placement, comparer deux
reglages, illustrer un resultat.

    python rendu.py                        4 vues du modele -> rendus/
    python rendu.py --vues face profil     seulement celles-la
    python rendu.py --pid 8                8 s de la cascade en video
    python rendu.py --chute 1.2            chute libre, PID coupe

Vues disponibles : face profil dessus troisquarts detail_roue
"""
import argparse
import math
import os
import numpy as np
import mujoco
import etat
from PIL import Image

DOSSIER = 'rendus'

#  nom            azimut  elevation  distance  point vise (x, y, z)
VUES = {
    'face':        (  0, -8,  0.62, (0, 0, 0.10)),
    'profil':      ( 90, -8,  0.62, (0, 0, 0.10)),
    'dessus':      ( 90, -78, 0.55, (0, 0, 0.08)),
    'troisquarts': (135, -18, 0.68, (0, 0, 0.10)),
    'detail_roue': (110, -12, 0.22, (0, 0.069, 0.0)),
    # vue de cinema : trois quarts leger, le tangage reste lisible
    'film':        (104, -10, 0.56, (0, 0, 0.115)),
    # serree, pour le montage cote a cote avec la prise reelle : le robot doit
    # y occuper la meme place que sur la video, sinon l oeil compare deux
    # tailles au lieu de comparer deux comportements.
    'duo':         (104, -6, 0.40, (0, 0, 0.118)),
}


def camera(vue, suivre=None):
    """suivre : position a viser. Sans elle la camera reste fixe et le robot
    sort du cadre des qu il avance -- ce qui est exactement ce qu on veut
    filmer. On garde la hauteur de visee du reglage, on ne suit qu en x et y."""
    az, el, di, cible = VUES[vue]
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.azimuth, c.elevation, c.distance = az, el, di
    if suivre is None:
        c.lookat[:] = cible
    else:
        c.lookat[:] = [suivre[0], suivre[1], cible[2]]
    return c


def rendre(r, d, vue, options=None, suivre=None):
    r.update_scene(d, camera=camera(vue, suivre), scene_option=options)
    return r.render()


def bandeau(img, texte):
    """Une legende lisible, incrustee en bas a gauche."""
    from PIL import ImageDraw
    im = Image.fromarray(img)
    g = ImageDraw.Draw(im)
    g.rectangle([0, im.height - 26, im.width, im.height], fill=(16, 21, 27))
    g.text((10, im.height - 19), texte, fill=(200, 212, 222))
    return np.array(im)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--vues', nargs='*', default=list(VUES))
    ap.add_argument('--pid', type=float, default=0.0, help='duree en s, cascade active')
    ap.add_argument('--chute', type=float, default=0.0, help='duree en s, PID coupe')
    ap.add_argument('--largeur', type=int, default=1100)
    ap.add_argument('--hauteur', type=int, default=800)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--transparent', action='store_true',
                    help='affiche aussi les geoms de collision par-dessus')
    a = ap.parse_args()

    os.makedirs(DOSSIER, exist_ok=True)
    m = mujoco.MjModel.from_xml_path('modeles/balancier.xml')
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, height=a.hauteur, width=a.largeur)

    opt = None
    if a.transparent:
        opt = mujoco.MjvOption()
        opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = True

    # ------------------------------------------------------------ instantanes
    if not a.pid and not a.chute:
        etat.poser(m, d, tangage_deg=0.0)
        for v in a.vues:
            if v not in VUES:
                print('  vue inconnue : %s' % v)
                continue
            chemin = os.path.join(DOSSIER, '%s.png' % v)
            Image.fromarray(rendre(r, d, v, opt)).save(chemin)
            print('  %s' % chemin)
        raise SystemExit

    # -------------------------------------------------------------- sequence
    duree = a.pid or a.chute
    nom = 'pid' if a.pid else 'chute'
    DT_CTRL = 1.0 / 200
    N_PHYS = round(DT_CTRL / m.opt.timestep)

    fw = None
    if a.pid:
        from firmware import Firmware
        fw = Firmware(dual=True)

    from firmware import RAD_PAR_PAS

    etat.poser(m, d, tangage_deg=(0.3 if a.pid else 2.0))
    if fw:
        fw.pitch = 0.3 if a.pid else 2.0

    images, tick, prochaine = [], 0, 0.0
    while d.time < duree:
        if fw:
            fw.imu(etat.gyro(m, d), etat.accel(m, d), DT_CTRL)
            fw.inner(DT_CTRL)
            if tick % 5 == 0:
                fw.outer(DT_CTRL * 5, etat.odometrie_pas(m, d, RAD_PAR_PAS))
            d.ctrl[:] = fw.wheel_sps * RAD_PAR_PAS
        for _ in range(N_PHYS):
            mujoco.mj_step(m, d)
        if d.time >= prochaine:
            img = rendre(r, d, 'profil', opt,
                         suivre=d.xpos[mujoco.mj_name2id(
                             m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')])
            leg = 't = %5.2f s   tangage %+6.2f deg' % (d.time, etat.tangage(m, d))
            if fw:
                leg += '   sps %+6.0f' % fw.wheel_sps
            images.append(bandeau(img, leg))
            prochaine += 1.0 / a.fps
        tick += 1

    gif = os.path.join(DOSSIER, '%s.gif' % nom)
    Image.fromarray(images[0]).save(
        gif, save_all=True, append_images=[Image.fromarray(i) for i in images[1:]],
        duration=int(1000 / a.fps), loop=0, optimize=True)
    print('  %s   (%d images, %.1f s)' % (gif, len(images), duree))
    for i, k in enumerate((0, len(images) // 2, len(images) - 1)):
        c = os.path.join(DOSSIER, '%s_%d.png' % (nom, i))
        Image.fromarray(images[k]).save(c)
        print('  %s' % c)
