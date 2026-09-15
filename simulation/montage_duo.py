"""Simulation et robot reel cote a cote, pour le post LinkedIn.

    python montage_duo.py

Le haut est la simulation MuJoCo avec le moteur mesure (ressort magnetique,
resonance a 71 Hz). Le bas est la prise du 10 septembre 2026. Meme robot, meme
politique de 1 634 poids, meme geste : on pousse et il se rattrape.

Les deux bandeaux de telemetrie sont construits de la meme facon et affichent la
meme grandeur, l erreur d angle. Celui du bas vient de logs/robot.log,
synchronise a l image ; celui du haut vient de la simulation. C est la
comparaison que le post raconte, et elle n a de valeur que si les deux colonnes
sont mesurees et non redessinees.

Sortie 1080x1920, muet, pret a publier.
"""
import io
import os
import re
import math
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import film
import montage_reel as MR

SORTIE = os.path.join('rendus', 'v3_moteur_fidele')
TRAVAIL = os.path.join(SORTIE, '_duo')

L, H = 1080, 800           # chaque panneau video
BANDE = 160                # chaque bandeau de telemetrie
FPS = 30
DUREE = 14.0               # s

VIDEO = MR.VIDEO
# On reprend le meilleur passage du reel : les trois poussees au doigt.
REEL_DEBUT = 33.5
# Le robot occupe 900 px de haut dans la prise. Le panneau n en fait que 800 :
# on recadre a la bonne hauteur PUIS on reduit, plutot que de lui couper la
# tete. Les 60 px perdus de chaque cote sont du fond noir, on n y perd rien.
CROP_REEL = (1080, 880, 0, 530)

BLANC = MR.BLANC
GRIS = MR.GRIS
VERT = MR.VERT
OCRE = MR.OCRE
ACCENT = MR.ACCENT


def bandeau(erreurs, i, titre, coul, sur_poussee):
    """Bandeau compact : le titre, la courbe glissante, la valeur."""
    im = Image.new('RGB', (L, BANDE), (14, 16, 20))
    d = ImageDraw.Draw(im)
    f_t = MR.police(26, True)
    f_p = MR.police(17)
    f_g = MR.police(44, True)
    d.line([0, 0, L, 0], fill=(38, 42, 52))
    d.text((34, 20), titre, font=f_t, fill=coul)

    x0, y0, larg, haut = 34, 62, 560, 82
    d.rectangle([x0, y0, x0 + larg, y0 + haut], outline=(38, 42, 52))
    d.line([x0, y0 + haut / 2, x0 + larg, y0 + haut / 2], fill=(34, 38, 48))
    d.text((x0 + larg - 108, 44), 'tilt error, +/- 12 deg', font=f_p,
           fill=(70, 76, 90))

    n = int(4.0 * FPS)                       # 4 s de courbe
    deb = max(0, i - n)
    tranche = erreurs[deb:i + 1]
    if len(tranche) > 2:
        pts = []
        for k, v in enumerate(tranche):
            px = x0 + larg * (k + (n - len(tranche) + 1)) / n
            py = y0 + haut / 2 - max(-12.0, min(12.0, v)) * (haut / 2) / 12.0
            pts.append((px, py))
        d.line(pts, fill=OCRE if sur_poussee else VERT, width=3)

    v = erreurs[min(i, len(erreurs) - 1)] if len(erreurs) else 0.0
    d.text((x0 + larg + 64, 52), '%+.2f' % v, font=f_g, fill=BLANC)
    d.text((x0 + larg + 236, 86), 'deg', font=f_p, fill=GRIS)
    return im


def panneau_simulation():
    """Rend la simulation et renvoie (images PIL, erreurs d angle)."""
    film.ACTIONNEUR = 'couple'
    pilote = film.charger_agent(os.path.join('agents', 'balancier_final.zip'))
    film.VUE = 'duo'
    brutes, chute = film.jouer(pilote, difficulte=0.0, largeur=L, hauteur=H)
    print('  simulation : %d images, %s'
          % (len(brutes), 'chute' if chute else 'tient'))
    ims = [Image.fromarray(im) for im, _ in brutes]
    err = [float(m['ang']) for _, m in brutes]
    return ims, err


def panneau_reel():
    """Extrait le passage utile de la prise, recadre, en images PIL."""
    os.makedirs(TRAVAIL, exist_ok=True)
    motif = os.path.join(TRAVAIL, 'r%05d.png')
    l, h, x, y = CROP_REEL
    subprocess.run(['ffmpeg', '-y', '-v', 'error',
                    '-ss', str(REEL_DEBUT), '-t', str(DUREE), '-i', VIDEO,
                    '-vf', 'crop=%d:%d:%d:%d,scale=960:%d,'
                    'pad=%d:%d:(ow-iw)/2:0:color=0x0e1014,fps=%d'
                    % (l, h, x, y, H, L, H, FPS),
                    motif], check=True)
    fics = sorted(f for f in os.listdir(TRAVAIL) if f.startswith('r'))
    print('  reel : %d images' % len(fics))
    T, E, S = MR.lire_journal()
    err = []
    for k in range(len(fics)):
        tv = REEL_DEBUT + k / FPS
        err.append(float(E[int(np.argmin(np.abs(T - tv)))]) if len(T) else 0.0)
    return [os.path.join(TRAVAIL, f) for f in fics], err


def main():
    os.makedirs(TRAVAIL, exist_ok=True)
    ims_sim, err_sim = panneau_simulation()
    fics_reel, err_reel = panneau_reel()

    n = min(len(ims_sim), len(fics_reel), int(DUREE * FPS))
    print('  montage sur %d images (%.1f s)' % (n, n / FPS))

    # Les instants de poussee, pour colorer la courbe.
    pics_reel = [36.3, 38.5, 42.0]
    pics_sim = [t for t, k, _, _ in film.SCENARIO if k == 'poussee']

    dossier = os.path.join(TRAVAIL, 'assemble')
    os.makedirs(dossier, exist_ok=True)
    haut_total = H + BANDE + H + BANDE
    for k in range(n):
        toile = Image.new('RGB', (L, haut_total), (14, 16, 20))
        toile.paste(ims_sim[k], (0, 0))
        tsim = k / FPS
        toile.paste(bandeau(err_sim, k, 'SIMULATION  ·  MuJoCo, measured motor',
                            ACCENT, min(abs(tsim - p) for p in pics_sim) < 0.4),
                    (0, H))
        toile.paste(Image.open(fics_reel[k]).convert('RGB'), (0, H + BANDE))
        trl = REEL_DEBUT + k / FPS
        toile.paste(bandeau(err_reel, k, 'REAL ROBOT  ·  same 1 634-weight policy',
                            OCRE, min(abs(trl - p) for p in pics_reel) < 0.4),
                    (0, H + BANDE + H))
        toile.save(os.path.join(dossier, '%05d.png' % k))
    print('  images assemblees')

    sortie = os.path.join(SORTIE, 'duo_sim_reel.mp4')
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-framerate', str(FPS),
                    '-i', os.path.join(dossier, '%05d.png'),
                    '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
                    '-vf', 'scale=1080:-2', sortie], check=True)
    print()
    print('  ->  %s' % sortie)


if __name__ == '__main__':
    main()
