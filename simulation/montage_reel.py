"""Montage de la prise reelle du 10 septembre 2026, pour publication.

    python montage_reel.py

Ce que ca fait, et pourquoi.

AUCUN RECADRAGE. Deux versions rejetees avant celle-ci : un carre serre sur le
robot ("tu as rogne beaucoup trop"), puis un rognage leger du bas. Verdict :
"ne coupe pas, juste rogne le temps". L image sort donc telle quelle, en
1080x1920.

Le pupitre visible a l ecran EST la telemetrie, en direct : c est plus
convaincant qu un bandeau ajoute apres coup, et ca montre que la scene est
reelle.

SON. Coupe. Sur LinkedIn la video demarre muette de toute facon, et un mauvais
son ne se remarque que chez ceux qui l activent.

PAS DE BANDEAU. Une courbe et des chiffres regeneres depuis le journal avaient
ete ajoutes sous l image. Rejetes : mal places, et pas assez justes pour etre
montres.

Le journal sert encore, mais seulement a CHOISIR ou couper : les pics de gyro
donnent l instant exact de chaque poussee, ce que l oeil ne sait pas faire en
faisant defiler une timeline.
"""
import io
import os
import re
import math
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# La prise brute (telephone, 10 sept 2026) n est pas versionnee : a deposer ici.
VIDEO = os.path.join('rendus', 'source', '20260910_210651.mp4')
# Le journal de la soiree est dans donnees/logs_robot_*.zip : le decompresser
# dans logs/ a la racine du depot, la ou le moniteur serie ecrit.
LOG = os.path.join('..', 'logs', 'robot.log')
SORTIE = os.path.join('rendus', 'v3_moteur_fidele')

DEBUT_LOG = 21 * 3600 + 6 * 60 + 51.0      # heure de debut, tiree du nom du fichier
FPS = 30
COTE = 1080
# Aucun recadrage. L image reste en 1080x1920.

# UN SEUL segment continu : pas de saut au milieu. Il commence juste avant la
# premiere poussee au doigt (pic de gyro a 36,3 s) et va jusqu au bout.
SEGMENT = (33.0, 62.6)

# Le bandeau est AJOUTE SOUS l image, pas pose dessus. Pose dessus, il
# masquait les roues -- or les roues sont precisement ce qu on veut voir quand
# le robot encaisse une poussee.
BANDE = 280                                 # hauteur du bandeau de telemetrie
FENETRE = 4.0                               # s de courbe affichee
ACCENT = (90, 169, 230)
VERT = (90, 224, 138)
OCRE = (224, 164, 88)
GRIS = (138, 144, 160)
BLANC = (232, 236, 242)

LIGNE = re.compile(
    r'^(\d\d):(\d\d):([\d.]+) P:(-?[\d.]+) tgt:(-?[\d.]+) e:(-?[\d.]+) '
    r'g:(-?[\d.]+) (PID|AGENT) (DUAL|ANG|OFF) sps:(-?\d+) ')


def police(taille, gras=False):
    for nom in (('consolab.ttf' if gras else 'consola.ttf'),
                ('arialbd.ttf' if gras else 'arial.ttf')):
        try:
            return ImageFont.truetype(nom, taille)
        except OSError:
            pass
    return ImageFont.load_default()


def lire_journal():
    """Renvoie t (s depuis le debut de la video), erreur d angle, commande."""
    brut = io.open(LOG, 'rb').read().replace(b'\x00', b'')
    t, e, s = [], [], []
    for l in brut.decode('utf-8', 'replace').splitlines():
        m = LIGNE.match(l)
        if not m:
            continue
        ts = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3))
        u = ts - DEBUT_LOG
        if -3.0 <= u <= 70.0:
            t.append(u)
            e.append(float(m.group(6)))
            s.append(float(m.group(10)))
    return np.array(t), np.array(e), np.array(s)


def bandeau(t_video, T, E, S, sur_poussee):
    """Une image RGB de BANDE pixels de haut, collee SOUS la video."""
    im = Image.new('RGB', (COTE, BANDE), (14, 16, 20))
    d = ImageDraw.Draw(im)
    f_pet = police(20)
    f_min = police(17)
    f_moy = police(34, True)
    f_gro = police(56, True)

    d.line([0, 0, COTE, 0], fill=(38, 42, 52))

    # ---- la courbe, 4 dernieres secondes
    x0, y0, larg, haut = 36, 62, 560, 168
    d.rectangle([x0, y0, x0 + larg, y0 + haut], outline=(38, 42, 52))
    d.line([x0, y0 + haut / 2, x0 + larg, y0 + haut / 2], fill=(34, 38, 48))
    d.text((x0, 28), 'TILT ERROR', font=f_pet, fill=GRIS)
    d.text((x0 + larg - 110, 30), '+/- 4 deg', font=f_min, fill=(70, 76, 90))

    sel = (T >= t_video - FENETRE) & (T <= t_video)
    if sel.sum() > 2:
        tt, ee = T[sel], E[sel]
        pts = []
        for a, b in zip(tt, ee):
            px = x0 + larg * (a - (t_video - FENETRE)) / FENETRE
            py = y0 + haut / 2 - max(-4.0, min(4.0, b)) * (haut / 2) / 4.0
            pts.append((px, py))
        d.line(pts, fill=OCRE if sur_poussee else VERT, width=4)

    # ---- les chiffres
    i = int(np.argmin(np.abs(T - t_video))) if len(T) else 0
    err = E[i] if len(E) else 0.0
    cmd = S[i] if len(S) else 0.0
    cx = x0 + larg + 72
    d.text((cx, 28), 'TILT', font=f_pet, fill=GRIS)
    d.text((cx, 52), '%+.2f' % err, font=f_gro, fill=BLANC)
    d.text((cx + 200, 82), 'deg', font=f_pet, fill=GRIS)
    d.text((cx, 140), 'WHEEL COMMAND', font=f_pet, fill=GRIS)
    d.text((cx, 166), '%+5d' % cmd, font=f_moy, fill=ACCENT)
    d.text((cx + 200, 178), 'steps/s', font=f_pet, fill=GRIS)
    return im


def coin(t_video, texte):
    im = Image.new('RGBA', (COTE, 96), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.text((34, 26), texte, font=police(34, True), fill=BLANC)
    return im


def main():
    os.makedirs(SORTIE, exist_ok=True)
    a, b = SEGMENT
    sortie = os.path.join(SORTIE, 'reel.mp4')
    subprocess.run(['ffmpeg', '-y', '-v', 'error',
                    '-ss', '%.3f' % a, '-to', '%.3f' % b, '-i', VIDEO,
                    '-an', '-c:v', 'libx264', '-crf', '16',
                    '-pix_fmt', 'yuv420p', sortie], check=True)
    print('  segment   %.1f -> %.1f s   (%.1f s)' % (a, b, b - a))
    print('  image     1080x1920, intacte')
    print('  son       coupe')
    print()
    print('  ->  %s' % sortie)


if __name__ == '__main__':
    main()
