# -*- coding: utf-8 -*-
"""
Rendu soigne : des videos presentables, pas des captures de debug.

    python film.py --duel                 agent contre cascade, cote a cote
    python film.py --progression          l apprentissage, du premier jalon au dernier
    python film.py --demo                 une demo simple de l agent final
    python film.py --format carre         1080x1080  (defaut : 16/9 en 1280x720)
    python film.py --format portrait      1080x1350, le format qui marche le mieux

Camera asservie au robot, incrustations lisibles, sortie MP4 par ffmpeg.
Tout est reproductible : meme graine, memes poussees, meme sequence.
"""
import argparse
import glob
import math
import os
import re
import subprocess

import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont

import etat
import rendu as R
from balancier_env import (Balancier, MAX_ACCEL, HZ_POLITIQUE, RAD_PAR_PAS,
                           RAYON, V_MAX_CONSIGNE)
from firmware import Firmware

SORTIE = os.path.join('rendus', 'v3_moteur_fidele')

# Le moteur fidele est desormais le defaut pour TOUT rendu : filmer un agent
# sur l actionneur en vitesse revient a le filmer sur un robot qui n existe
# pas. C est exactement ce qui avait donne des videos flatteuses le 5 septembre
# et un robot qui tremble le 9.
ACTIONNEUR = 'couple'
# La vue de rendu. 'duo' est plus serree, pour le montage cote a cote ou le
# robot simule doit occuper la meme place que le robot filme.
VUE = 'film'
FPS = 30
AUTEUR = 'Matthieu Vinet'
MENTION = 'MuJoCo + PPO  |  Matthieu Vinet  |  2026'

# --- palette, la meme que les documents du projet
FOND = (16, 21, 27)
ENCRE = (227, 233, 238)
ENCRE2 = (140, 152, 163)
ACCENT = (78, 184, 216)          # l agent
OCRE = (224, 160, 85)            # la cascade, et les alertes
VERT = (95, 196, 162)
TRAIT = (45, 55, 64)

FORMATS = {'large': (1280, 720), 'carre': (1080, 1080), 'portrait': (1080, 1350)}
POLICES = 'C:/Windows/Fonts/'


def police(nom, taille):
    try:
        return ImageFont.truetype(POLICES + nom, taille)
    except OSError:
        return ImageFont.load_default()


# ====================================================================== pilotes
def charger_agent(chemin):
    from stable_baselines3 import PPO
    mdl = PPO.load(chemin, device='cpu')
    return lambda o, e: mdl.predict(o, deterministic=True)[0]


def pilote_cascade():
    fw = Firmware(dual=True)

    def f(o, e):
        fw.pitch = float(o[0]) * 10.0
        fw.gyroFilt = float(o[1]) * 100.0
        fw.wheel_sps = float(np.mean(e.sps))
        corr = fw.Kp_spd * (fw.wheel_sps - e.consigne[0] / (RAD_PAR_PAS * RAYON))
        corr += fw.Ki_spd * (float(o[5]) * 0.20 / (RAD_PAR_PAS * RAYON))
        cible = -max(-6.0, min(6.0, corr))
        accel = fw.Kp_a * (fw.pitch - cible) + fw.Kd_a * fw.gyroFilt
        return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1.0, 1.0)
    return f


# ===================================================================== scenario
#  Une sequence ECRITE, identique pour tous les pilotes et tous les jalons :
#  c est ce qui rend les videos comparables entre elles. Les libelles sont en
#  anglais -- ces videos sont faites pour etre publiees.
SCENARIO = [
    (0.0,  'consigne', (0.00, 0.0),  'hold position'),
    (1.8,  'poussee',  (+0.28, 0.21), None),      # les deux tiennent, verifie
    (4.0,  'poussee',  (-0.28, 0.21), None),
    (6.0,  'consigne', (0.12, 0.0),  'drive forward'),
    (9.0,  'consigne', (-0.10, 0.0), 'drive backward'),
    (12.0, 'consigne', (0.00, 45.0), 'turn in place'),
    (14.5, 'consigne', (0.00, 0.0),  'hold position'),
    (16.0, 'poussee',  (+0.50, 0.21), None),      # sous la limite des deux
    (19.0, 'fin',      None,         ''),
    # ------------------------------------------------------------------
    #  Forces recalibrees le 9 sept 2026 sur le MOTEUR FIDELE. Les anciennes
    #  (0,45 puis 0,64 N) venaient de l actionneur en vitesse et etaient
    #  au-dessus de ce que quiconque encaisse desormais : l agent de 9 M
    #  tombait dans la video de progression.
    #
    #  Limites mesurees (17_pousser_agents.py, poussee isolee de 150 ms) :
    #      cascade 0,47 N   nouvel agent 0,53 N   ancien agent 0,38 N
    #
    #  Mais dans un ENCHAINEMENT l avantage de 13 % du nouvel agent ne tient
    #  pas : a 0,55 N les deux tombent, a 0,50 les deux tiennent. On ne
    #  fabrique donc pas une difference qui n existe pas -- le scenario reste
    #  sous la limite des deux, et ce qui se voit est la vraie difference :
    #  l agent tremble 19 fois moins (commande d ecart-type 47 pas/s contre
    #  86 pour la cascade et 873 pour l ancien agent).
    # ------------------------------------------------------------------
]
DUREE_POUSSEE = 0.15


def jouer(pilote, difficulte=0.0, graine=7, largeur=760, hauteur=560):
    """Deroule le scenario et rend les images brutes + la telemetrie."""
    env = Balancier(difficulte=difficulte, avec_consignes=False, graine=graine,
                    actionneur=ACTIONNEUR)
    o, _ = env.reset(seed=graine)
    env.t_poussee = 1e9
    r = mujoco.Renderer(env.m, height=hauteur, width=largeur)
    b = mujoco.mj_name2id(env.m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')

    images = []
    prochaine, i_evt, phase, fin_p, force_p, haut_p = 0.0, 0, 'hold position', -1.0, 0.0, 0.21
    chute_a = None
    from collections import deque
    hist = deque([0.0] * 400, maxlen=400)        # 4 s d angle a 100 Hz

    while env.d.time < SCENARIO[-1][0]:
        while i_evt < len(SCENARIO) and env.d.time >= SCENARIO[i_evt][0]:
            t, genre, arg, texte = SCENARIO[i_evt]
            if genre == 'consigne':
                env.consigne[0], env.consigne[1] = arg[0], math.radians(arg[1])
                phase = texte
            elif genre == 'poussee':
                force_p, haut_p = arg
                env.hauteur_externe = haut_p
                fin_p = env.d.time + DUREE_POUSSEE
            i_evt += 1

        env.force_externe = force_p if env.d.time < fin_p else 0.0
        o, rec, tombe, fini, info = env.step(pilote(o, env))
        hist.append(info['tangage'])
        if tombe and chute_a is None:
            chute_a = env.d.time

        if env.d.time >= prochaine:
            images.append((R.rendre(r, env.d, VUE, suivre=env.d.xpos[b]),
                           dict(t=env.d.time, ang=info['tangage'], v=info['v'],
                                vc=env.consigne[0], w=math.degrees(info['w']),
                                wc=math.degrees(env.consigne[1]),
                                f=env.force_externe, h=haut_p, phase=phase,
                                tombe=chute_a is not None, hist=list(hist))))
            prochaine += 1.0 / FPS
    return images, chute_a


# ================================================================ incrustations
def habiller(img, m, titre, sous_titre, L, H, coul=ACCENT):
    """Habillage complet, en anglais. Trois choses doivent sauter aux yeux :
    ce qu on demande au robot, quand on le pousse, et ce qu il fait."""
    im = Image.fromarray(img).convert('RGB')
    if im.size != (L, H):
        im = im.resize((L, H), Image.LANCZOS)
    g = ImageDraw.Draw(im, 'RGBA')
    u = L / 1280.0                                    # echelle typographique
    f_t = police('seguisb.ttf', int(26 * u))
    f_s = police('segoeui.ttf', int(18 * u))
    f_e = police('seguisb.ttf', int(12 * u))          # etiquettes
    f_m = police('consola.ttf', int(17 * u))
    f_g = police('seguisb.ttf', int(34 * u))          # la banniere de poussee
    marge = int(34 * u)

    # ---------------------------------------------------- en-tete
    haut = int(64 * u)
    g.rectangle([0, 0, L, haut], fill=(*FOND, 215))
    g.text((marge, int(19 * u)), titre, font=f_t, fill=ENCRE)
    if sous_titre:
        g.text((marge + g.textlength(titre, font=f_t) + int(18 * u),
                int(25 * u)), sous_titre, font=f_s, fill=coul)
    ht = '%4.1f s' % m['t']
    g.text((L - marge - g.textlength(ht, font=f_m), int(25 * u)), ht,
           font=f_m, fill=ENCRE2)

    # ------------------------------------- la CONSIGNE, toujours affichee
    y = haut + int(16 * u)
    lignes = [('COMMAND', None),
              ('forward', '%+.2f m/s' % m['vc']),
              ('turn', '%+.0f deg/s' % m['wc'])]
    lc = int(200 * u)
    g.rounded_rectangle([marge, y, marge + lc, y + int(74 * u)],
                        radius=int(5 * u), fill=(*FOND, 195))
    g.text((marge + int(12 * u), y + int(8 * u)), 'COMMAND', font=f_e, fill=coul)
    for i, (k, v) in enumerate(lignes[1:]):
        yy = y + int((28 + 21 * i) * u)
        g.text((marge + int(12 * u), yy), k, font=f_e, fill=ENCRE2)
        g.text((marge + lc - int(12 * u) - g.textlength(v, font=f_m), yy - int(3 * u)),
               v, font=f_m, fill=ENCRE)
    g.text((marge, y + int(84 * u)), m['phase'].upper(), font=f_e, fill=ENCRE2)

    # ------------------------------------------- la POUSSEE, impossible a rater
    if m['f']:
        fleche = '<<<' if m['f'] < 0 else '>>>'
        txt = ('%s   PUSH  %.1f N  @  %.0f mm   %s'
               % (fleche, abs(m['f']), m['h'] * 1000, fleche))
        w = g.textlength(txt, font=f_g)
        x, yy = (L - w) / 2, haut + int(20 * u)
        g.rounded_rectangle([x - int(26 * u), yy - int(12 * u),
                             x + w + int(26 * u), yy + f_g.size + int(12 * u)],
                            radius=int(6 * u), fill=OCRE)
        g.text((x, yy), txt, font=f_g, fill=FOND)
        g.rectangle([0, 0, L - 1, H - 1], outline=OCRE, width=int(6 * u))

    if m['tombe']:
        txt = 'FALLEN'
        w = g.textlength(txt, font=f_g)
        g.rounded_rectangle([(L - w) / 2 - int(26 * u), H // 2 - int(30 * u),
                             (L + w) / 2 + int(26 * u), H // 2 + int(30 * u)],
                            radius=int(6 * u), fill=(200, 70, 60))
        g.text(((L - w) / 2, H // 2 - int(20 * u)), txt, font=f_g, fill=(255, 255, 255))

    # ---------------------------------------------------- jauges du bas
    hb = int(142 * u)
    g.rectangle([0, H - hb, L, H], fill=(*FOND, 228))
    larg = L - 2 * marge

    def jauge(yj, valeur, plage, etiq, texte, couleur, marque=None):
        g.text((marge, yj - int(20 * u)), etiq, font=f_e, fill=ENCRE2)
        g.text((marge + larg - g.textlength(texte, font=f_m), yj - int(22 * u)),
               texte, font=f_m, fill=ENCRE)
        g.rectangle([marge, yj, marge + larg, yj + int(4 * u)], fill=TRAIT)
        cx = marge + larg / 2
        g.rectangle([cx - 1, yj - int(5 * u), cx + 1, yj + int(9 * u)], fill=(70, 82, 92))
        x = cx + max(-larg / 2, min(larg / 2, valeur / plage * larg / 2))
        a, bx = (cx, x) if x >= cx else (x, cx)
        g.rectangle([a, yj, bx, yj + int(4 * u)], fill=couleur)
        g.ellipse([x - int(6 * u), yj - int(4 * u), x + int(6 * u), yj + int(8 * u)],
                  fill=couleur)
        if marque is not None:
            xm = cx + max(-larg / 2, min(larg / 2, marque / plage * larg / 2))
            g.polygon([(xm, yj - int(8 * u)), (xm - int(6 * u), yj - int(19 * u)),
                       (xm + int(6 * u), yj - int(19 * u))], fill=OCRE)

    # --- TRACE de l angle sur les 4 dernieres secondes
    yt, ht = H - hb + int(14 * u), int(52 * u)
    g.text((marge, yt - int(2 * u)), 'TILT   last 4 s,  +-6 deg', font=f_e, fill=ENCRE2)
    txt = '%+.2f deg' % m['ang']
    g.text((marge + larg - g.textlength(txt, font=f_m), yt - int(4 * u)),
           txt, font=f_m, fill=ENCRE)
    y0 = yt + int(16 * u) + ht / 2
    g.line([marge, y0, marge + larg, y0], fill=TRAIT)
    h = m.get('hist') or [0.0]
    pts = []
    for i, v in enumerate(h):
        pts += [marge + i * larg / len(h),
                y0 - max(-ht / 2, min(ht / 2, v / 6.0 * ht / 2))]
    if len(pts) >= 4:
        g.line(pts, fill=coul, width=max(2, int(2 * u)))

    jauge(H - hb + int(104 * u), m['v'], V_MAX_CONSIGNE,
          'SPEED   triangle = command', '%+.3f m/s' % m['v'], VERT, marque=m['vc'])

    # ---------------------------------------------------- signature
    #  Deux fois : une mention lisible dans le bandeau, et une marque discrete
    #  DANS l image -- celle-la ne part pas si quelqu un recadre les bandeaux.
    f_c = police('segoeui.ttf', int(13 * u))
    g.text((L - marge - g.textlength(MENTION, font=f_c), H - int(20 * u)),
           MENTION, font=f_c, fill=(90, 102, 112))
    f_w = police('seguisb.ttf', int(15 * u))
    g.text((L - marge - g.textlength(AUTEUR, font=f_w), int(H * 0.42)),
           AUTEUR, font=f_w, fill=(226, 232, 238, 40))
    return np.array(im)


def ecrire(images, nom, L, H):
    os.makedirs(SORTIE, exist_ok=True)
    chemin = os.path.join(SORTIE, nom + '.mp4')
    p = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo',
         '-pix_fmt', 'rgb24', '-s', '%dx%d' % (L, H), '-r', str(FPS), '-i', '-',
         '-c:v', 'libx264', '-preset', 'slow', '-crf', '19',
         '-pix_fmt', 'yuv420p', '-movflags', '+faststart', chemin],
        stdin=subprocess.PIPE)
    for im in images:
        p.stdin.write(np.ascontiguousarray(im, dtype=np.uint8).tobytes())
    p.stdin.close()
    p.wait()
    print('  %-50s %6.0f ko  %5.1f s'
          % (chemin, os.path.getsize(chemin) / 1024, len(images) / FPS))
    return chemin


def jalons():
    f = glob.glob(os.path.join('agents', 'balancier_*_pas.zip'))
    return sorted(f, key=lambda x: int(re.search(r'_(\d+)_pas', x).group(1)))


# ========================================================================= main
def _clip(pilote, titre, sous_titre, L, H, coul=ACCENT, difficulte=0.0):
    brutes, chute = jouer(pilote, difficulte=difficulte,
                          largeur=int(L * 1.0), hauteur=int(H * 1.0))
    return [habiller(im, m, titre, sous_titre, L, H, coul) for im, m in brutes], chute


def cote_a_cote(ga, dr, L, H):
    n = min(len(ga), len(dr))
    return [np.concatenate([ga[i], dr[i]], axis=1) for i in range(n)]


def empile(ha, ba):
    """Pour les formats carre et portrait : cote a cote donnerait deux
    vignettes trop etroites, on superpose."""
    n = min(len(ha), len(ba))
    return [np.concatenate([ha[i], ba[i]], axis=0) for i in range(n)]


# ====================================================================== mosaique
def vignette(img, m, n_pas, L, H):
    """Habillage minimal d une case : le compteur de pas et l etat."""
    im = Image.fromarray(img).convert('RGB')
    if im.size != (L, H):
        im = im.resize((L, H), Image.LANCZOS)
    g = ImageDraw.Draw(im, 'RGBA')
    u = L / 360.0
    f_n = police('seguisb.ttf', int(19 * u))
    f_e = police('segoeui.ttf', int(11 * u))
    etq = format(n_pas, ',').replace(',', ' ')
    g.rectangle([0, 0, L, int(30 * u)], fill=(*FOND, 210))
    g.text((int(9 * u), int(6 * u)), etq, font=f_n, fill=ENCRE)
    g.text((int(9 * u) + g.textlength(etq, font=f_n) + int(6 * u), int(12 * u)),
           'steps', font=f_e, fill=ENCRE2)
    if m['tombe']:
        g.rectangle([0, 0, L, H], fill=(190, 70, 60, 34))
        g.rectangle([0, 0, L - 1, H - 1], outline=(190, 70, 60), width=int(2 * u))
        t = 'FALLEN'
        g.text((L - int(9 * u) - g.textlength(t, font=f_e), int(12 * u)),
               t, font=f_e, fill=(232, 130, 120))
    else:
        g.rectangle([0, 0, L - 1, H - 1], outline=(*TRAIT, 200), width=1)
        g.ellipse([L - int(20 * u), int(11 * u), L - int(12 * u), int(19 * u)],
                  fill=VERT)
    return np.array(im)


def mosaique(L, H, cols, lignes):
    fichiers = jalons()[:cols * lignes]
    haut, bas = int(H * 0.09), int(H * 0.05)
    lc, hc = L // cols, (H - haut - bas) // lignes
    bandes, etiquettes = [], []
    for f in fichiers:
        n = int(re.search(r'_(\d+)_pas', f).group(1))
        brutes, _ = jouer(charger_agent(f), largeur=lc, hauteur=hc)
        bandes.append([vignette(im, m, n, lc, hc) for im, m in brutes])
        etiquettes.append((n, brutes))
        print('     %10s pas' % format(n, ','))
    n_img = min(len(b) for b in bandes)

    f_t = police('seguisb.ttf', int(H / 34))
    f_s = police('segoeui.ttf', int(H / 52))
    f_m = police('consola.ttf', int(H / 56))
    sorties = []
    for k in range(n_img):
        page = Image.new('RGB', (L, H), FOND)
        for i, b in enumerate(bandes):
            page.paste(Image.fromarray(b[k]),
                       ((i % cols) * lc, haut + (i // cols) * hc))
        g = ImageDraw.Draw(page)
        # En portrait, le sous-titre ne tient pas a cote du titre : il sortait
        # du cadre et se superposait a l horodatage. On l empile dessous.
        portrait = H > L
        titre = 'Learning to balance'
        sous = 'same robot, same pushes, %d training checkpoints' % len(bandes)
        g.text((int(L * 0.022), int(haut * (0.16 if portrait else 0.26))),
               titre, font=f_t, fill=ENCRE)
        if portrait:
            g.text((int(L * 0.022), int(haut * 0.52)), sous, font=f_s, fill=ACCENT)
        else:
            g.text((int(L * 0.022) + g.textlength(titre, font=f_t)
                    + int(L * 0.016), int(haut * 0.38)),
                   sous, font=f_s, fill=ACCENT)
        m0 = etiquettes[0][1][k][1]
        d = 'PUSH %.2f N' % abs(m0['f']) if m0['f'] else m0['phase']
        yd = int(haut * (0.16 if portrait else 0.30))
        g.text((L - int(L * 0.022) - g.textlength(d, font=f_m), yd),
               d.upper(), font=f_m, fill=OCRE if m0['f'] else ENCRE2)
        ht = '%4.1f s' % m0['t']
        g.text((L - int(L * 0.022) - g.textlength(ht, font=f_m),
                yd + int(haut * 0.20)), ht, font=f_m, fill=ENCRE2)
        g.text((int(L * 0.022), H - int(bas * 0.68)), MENTION, font=f_m,
               fill=(90, 102, 112))
        sorties.append(np.array(page))
    return sorties


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--duel', action='store_true')
    ap.add_argument('--progression', action='store_true')
    ap.add_argument('--demo', action='store_true')
    ap.add_argument('--mosaique', action='store_true')
    ap.add_argument('--format', choices=list(FORMATS), default='large')
    ap.add_argument('--difficulte', type=float, default=0.0)
    ap.add_argument('--actionneur', choices=('vitesse', 'couple'), default='couple')
    ap.add_argument('--comparer', action='store_true',
                    help='ancien agent contre nouveau, sur le meme moteur fidele')
    ap.add_argument('--ancien', type=str,
                    default=os.path.join('agents_v1_vitesse', 'balancier_9000000_pas.zip'))
    a = ap.parse_args()
    ACTIONNEUR = a.actionneur
    L, H = FORMATS[a.format]
    tout = not (a.duel or a.progression or a.demo or a.mosaique or a.comparer)

    fichiers = jalons()
    final = fichiers[-1] if fichiers else 'agents/balancier_final.zip'

    if a.mosaique:
        print('  MOSAIQUE  (12 jalons simultanes)')
        # 16 jalons -> grille 4 x 4. En 16/9 les cases seraient deux fois
        # plus larges que hautes, ce qui va mal a un robot debout : on
        # rend la mosaique en 4/3 ou en carre.
        Lm, Hm = (1280, 960) if a.format == 'large' else (1080, 1080)
        ecrire(mosaique(Lm, Hm, 4, 4), 'mosaique_%s' % a.format, Lm, Hm)

    if a.comparer or tout:
        # Le rendu qui raconte la soiree du 9 septembre : le MEME agent qui
        # paraissait excellent sur l actionneur en vitesse, remis sur le moteur
        # fidele a cote de celui qui y a ete entraine. L ecart de tremblement
        # se voit sans avoir besoin d une courbe.
        print('  COMPARAISON  (ancien contre nouveau, meme moteur fidele)')
        if a.format == 'large':
            Lc, Hc = L // 2, H
        else:
            Lc, Hc = L, H // 2
        g, _ = _clip(charger_agent(a.ancien), 'Trained on an ideal wheel',
                     'shakes on the real motor', Lc, Hc, OCRE, a.difficulte)
        d, _ = _clip(charger_agent(final), 'Trained on the measured motor',
                     'magnetic spring, 71 Hz resonance', Lc, Hc, ACCENT,
                     a.difficulte)
        im = (cote_a_cote(g, d, Lc, Hc) if a.format == 'large' else empile(g, d))
        ecrire(im, 'comparaison_%s' % a.format, L if a.format == 'large' else Lc,
               H if a.format == 'large' else H)

    if a.demo or tout:
        print('  DEMO')
        im, ch = _clip(charger_agent(final), 'Two-wheel balancing robot',
                       'reinforcement learning policy', L, H)
        ecrire(im, 'demo_%s' % a.format, L, H)

    if a.duel or tout:
        print('  DUEL  (meme robot, meme scenario, memes poussees)')
        if a.format == 'large':
            Ld, Hd = L // 2, H
        else:                                  # carre / portrait : on empile
            Ld, Hd = L, H // 2
        g, _ = _clip(charger_agent(final), 'RL policy', '1 634 weights', Ld, Hd,
                     ACCENT, a.difficulte)
        d, _ = _clip(pilote_cascade(), 'Hand-tuned cascade', 'PID, 3 gains', Ld, Hd,
                     OCRE, a.difficulte)
        im = (cote_a_cote(g, d, Ld, Hd) if a.format == 'large'
              else empile(g, d))
        ecrire(im, 'duel_%s' % a.format, Ld * (2 if a.format == 'large' else 1),
               Hd * (1 if a.format == 'large' else 2))

    if a.progression or tout:
        print('  PROGRESSION  (%d jalons)' % len(fichiers))
        images = []
        for f in fichiers:
            n = int(re.search(r'_(\d+)_pas', f).group(1))
            im, ch = _clip(charger_agent(f), 'Learning to balance',
                           '%s training steps' % format(n, ',').replace(',', ' '),
                           L, H)
            # inutile de filmer 19 s d un agent qui tombe a la premiere seconde
            garde = int((ch + 1.2) * FPS) if ch else len(im)
            images += im[:max(FPS * 2, garde)]
            print('     %10s pas  ->  %s' % (format(n, ','),
                                             'chute a %.1f s' % ch if ch else 'tient'))
        ecrire(images, 'progression_%s' % a.format, L, H)


