"""La video du post : une seule piece, verticale, muette.

    python montage_post.py

STRUCTURE
    0 - 1 s    une image FIXE de couverture
    1 - 8 s    le robot reel, plein cadre, premiere poussee
    7 - 20 s   le meme plan, avec la simulation EN MEDAILLON
   20 - 39 s   la mosaique des 16 jalons d apprentissage
   39 - 44 s   retour au reel, signature

La mosaique a d abord dure 8 s, puis 14. Toujours trop court : les avant
derniers jalons tombent entre 15 et 17 s, et c est exactement ce qu il faut
voir -- l apprentissage se lit dans QUAND ils tombent, pas dans le fait qu ils
tombent. Elle passe donc en entier, 19 s.

POURQUOI UN MEDAILLON ET PAS DEUX PANNEAUX EMPILES
Empiler la simulation au-dessus du reel oblige a recadrer la prise, et Matthieu
a tranche : "ne coupe pas". Le medaillon se pose sur le tapis noir, qui est
vide, et laisse l image d origine intacte. On garde la comparaison sans payer
le recadrage.

LA COUVERTURE
LinkedIn prend la PREMIERE IMAGE comme vignette du post. Sans couverture, c
etait une main floue au milieu d une poussee. Une seconde d image fixe suffit a
choisir ce que verra quelqu un qui ne lance pas la lecture -- c est-a-dire la
majorite.

PAS DE SON
92 % de l energie de la bande d origine est entre 120 et 1200 Hz : de la voix et
du bruit de piece. Le sifflement moteur, seule chose qui aurait valu la peine,
pese 4,6 %. Et sur LinkedIn la lecture demarre muette de toute facon.
"""
import os
import subprocess
from PIL import Image, ImageDraw

import film
import montage_reel as MR

SORTIE = os.path.join('rendus', 'v3_moteur_fidele')
TRAVAIL = os.path.join(SORTIE, '_post')
L, H = 1080, 1920
FPS = 30

REEL = MR.VIDEO
REEL_DEBUT = 33.0            # juste avant la premiere poussee au doigt

# medaillon : 420 de large, pose sur le tapis noir en bas a gauche
# Hauteur PAIRE : x264 en yuv420p refuse une dimension impaire.
MED_L, MED_H = 560, 420
MED_X, MED_Y = 44, H - MED_H - 300

BLANC = (238, 241, 246)
GRIS = (150, 156, 172)
ACCENT = (90, 169, 230)
OCRE = (224, 164, 88)


def carte(lignes, y0=140, voile=True):
    """Un calque de texte transparent, plein cadre.

    Le VOILE compte : le haut de l image est occupe par deux ecrans allumes,
    fond clair et charge. Un lisere ne suffit pas, il faut assombrir dessous.
    """
    im = Image.new('RGBA', (L, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    haut = sum(int(t * 1.45) for _, t, _, _ in lignes) + 70
    if voile:
        for k in range(haut):
            a = int(190 * (1.0 - (k / haut) ** 2.2))
            d.line([0, y0 - 44 + k, L, y0 - 44 + k], fill=(8, 10, 14, a))
    y = y0
    for texte, taille, coul, gras in lignes:
        # Ajustement automatique : une ligne trop longue etait coupee par le
        # bord droit ("200 corrections per seco..."). On reduit jusqu a ce
        # qu elle rentre, plutot que de se fier a un comptage de caracteres.
        f = MR.police(taille, gras)
        while taille > 20 and d.textlength(texte, font=f) > L - 112:
            taille -= 2
            f = MR.police(taille, gras)
        x = 56
        # liseré sombre : le texte doit rester lisible sur un fond clair
        for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2)):
            d.text((x + dx, y + dy), texte, font=f, fill=(8, 10, 14, 220))
        d.text((x, y), texte, font=f, fill=coul)
        y += int(taille * 1.45)
    return im


def cartes():
    os.makedirs(TRAVAIL, exist_ok=True)
    jeux = {
        'c1': [('Two wheels.', 78, BLANC, True),
               ('200 corrections per second.', 78, BLANC, True),
               ('', 20, GRIS, False),
               ('ESP32  -  MPU-6050  -  2x NEMA17', 38, GRIS, False)],
        'c2': [('Trained in simulation.', 72, ACCENT, True),
               ('Running on the robot.', 72, BLANC, True),
               ('', 16, GRIS, False),
               ('1 634 weights · 250 µs to decide, 5 ms available', 34, GRIS, False)],
        # Reduit pour tenir dans les 240 px au-dessus de la mosaique centree.
        'c3': [('16 checkpoints  -  9 000 000 steps', 44, OCRE, True),
               ('', 10, GRIS, False),
               ('15.7 minutes on CPU', 28, GRIS, False)],
        'c4': [('Matthieu Vinet', 68, BLANC, True),
               ('robot learning  -  Polytech Sorbonne', 38, GRIS, False)],
    }
    for nom, lignes in jeux.items():
        # c3 va dans la bande noire au-dessus de la mosaique centree : elle
        # commence a y=240, le texte doit donc tenir dans ces 240 px.
        y = {'c3': 66, 'c4': H - 400}.get(nom, 150)
        carte(lignes, y0=y, voile=(nom != 'c3')).save(
            os.path.join(TRAVAIL, nom + '.png'))

    # L etiquette du medaillon. Sans elle l incrustation ne veut rien dire : on
    # voit une petite image grise sans savoir que c est le MEME robot, dans le
    # simulateur, pilote par le MEME reseau.
    lab = Image.new('RGBA', (L, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(lab)
    f = MR.police(30, True)
    y = MED_Y - 48
    txt = 'MuJoCo simulation, same policy'
    for dx, dy in ((-2, 0), (2, 0), (0, -2), (0, 2)):
        d.text((MED_X + 6 + dx, y + dy), txt, font=f, fill=(8, 10, 14, 235))
    d.text((MED_X + 6, y), txt, font=f, fill=ACCENT)
    lab.save(os.path.join(TRAVAIL, 'lab.png'))
    noms = list(jeux) + ['lab']
    return {n: os.path.join(TRAVAIL, n + '.png') for n in noms}


def clip_simulation():
    """Un plan de simulation propre, sans habillage, pour le medaillon."""
    film.ACTIONNEUR = 'couple'
    film.VUE = 'duo'
    pilote = film.charger_agent(os.path.join('agents', 'balancier_final.zip'))
    brutes, _ = film.jouer(pilote, difficulte=0.0,
                           largeur=MED_L, hauteur=MED_H)
    d = os.path.join(TRAVAIL, 'sim')
    os.makedirs(d, exist_ok=True)
    for k, (im, _) in enumerate(brutes):
        Image.fromarray(im).save(os.path.join(d, '%05d.png' % k))
    print('  simulation : %d images' % len(brutes))
    sortie = os.path.join(TRAVAIL, 'sim.mp4')
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-framerate', str(FPS),
                    '-i', os.path.join(d, '%05d.png'), '-c:v', 'libx264',
                    '-crf', '16', '-pix_fmt', 'yuv420p', sortie], check=True)
    return sortie


def couverture():
    """L image fixe du debut, celle qui sert de vignette dans le fil."""
    import subprocess as sp
    brut = os.path.join(TRAVAIL, 'fond.png')
    sp.run(['ffmpeg', '-y', '-v', 'error', '-ss', '58.0', '-i', REEL,
            '-frames:v', '1', brut], check=True)
    im = Image.open(brut).convert('RGB')

    # Voile du haut : le texte doit rester lisible par-dessus deux ecrans
    # allumes, et la vignette doit se lire a la taille d un timbre.
    d = ImageDraw.Draw(im, 'RGBA')
    for k in range(760):
        d.line([0, k, L, k], fill=(8, 10, 14, int(225 * (1.0 - (k / 760) ** 1.9))))
    d.rectangle([0, H - 250, L, H], fill=(8, 10, 14, 200))

    f1 = MR.police(84, True)
    f2 = MR.police(84, True)
    f3 = MR.police(38)
    f4 = MR.police(34, True)
    d.text((56, 150), 'Trained', font=f1, fill=ACCENT)
    d.text((56, 250), 'in simulation.', font=f1, fill=ACCENT)
    d.text((56, 380), 'Running', font=f2, fill=BLANC)
    d.text((56, 480), 'on the robot.', font=f2, fill=BLANC)
    d.text((56, 620), 'MuJoCo  ·  PPO  ·  ESP32', font=f3, fill=GRIS)
    d.text((56, H - 180), 'Matthieu Vinet', font=f4, fill=BLANC)
    d.text((56, H - 132), 'robot learning  ·  Polytech Sorbonne',
           font=MR.police(28), fill=GRIS)

    # La simulation en medaillon : la vignette montre les DEUX mondes d un coup.
    sim = os.path.join(TRAVAIL, 'sim', '00030.png')
    if os.path.exists(sim):
        v = Image.open(sim).convert('RGB').resize((MED_L, MED_H))
        cadre = Image.new('RGB', (MED_L + 8, MED_H + 8), ACCENT)
        cadre.paste(v, (4, 4))
        im.paste(cadre, (MED_X, MED_Y))
        d.text((MED_X + 6, MED_Y - 46), 'MuJoCo simulation, same policy',
               font=MR.police(30, True), fill=ACCENT)

    chemin = os.path.join(TRAVAIL, 'couverture.png')
    im.save(chemin)
    return chemin


def main():
    os.makedirs(TRAVAIL, exist_ok=True)
    c = cartes()
    sim = clip_simulation()
    couv = couverture()
    mosaique = os.path.join(SORTIE, 'mosaique_verticale.mp4')

    sortie = os.path.join(SORTIE, 'post_sim_vs_real.mp4')
    f = [
        '[3:v]loop=loop=-1:size=1,trim=0:1.0,setpts=PTS-STARTPTS,fps=%d[cov]' % FPS,
        '[0:v]trim=%.2f:%.2f,setpts=PTS-STARTPTS,fps=%d[reel]'
        % (REEL_DEBUT, REEL_DEBUT + 25.0, FPS),
        '[reel]split=3[r1][r2][r3]',
        '[r1]trim=0:7,setpts=PTS-STARTPTS[a]',
        '[r2]trim=7:20,setpts=PTS-STARTPTS[b]',
        '[r3]trim=20:25,setpts=PTS-STARTPTS[d]',
        '[1:v]fps=%d,trim=0:13,setpts=PTS-STARTPTS,'
        'pad=%d:%d:4:4:color=0x5aa9e6[med]' % (FPS, MED_L + 8, MED_H + 8),
        '[b][med]overlay=%d:%d:shortest=1[b2]' % (MED_X, MED_Y),
        # La mosaique ne remplit PAS le cadre : 1080x1440 posee a y=400, ce
        # et elle est CENTREE : 240 px de noir en haut et en bas. Le carton ne doit
        # jamais passer par-dessus les cases, sinon plus rien n est lisible.
        '[2:v]fps=%d,trim=0:19,setpts=PTS-STARTPTS,scale=1080:1440,'
        'pad=%d:%d:0:240:color=0x0e1014[mos]' % (FPS, L, H),
        '[cov][a][b2][mos][d]concat=n=5:v=1:a=0[v]',
        # les cartons, decales d une seconde par la couverture
        "[v][4:v]overlay=0:0:enable='between(t,1.6,6.4)'[v1]",
        "[v1][5:v]overlay=0:0:enable='between(t,8.6,13.4)'[v2]",
        # Le carton reste tant que la mosaique est a l ecran. Il s effacait a
        # 30 s alors qu elle allait jusqu a 40 : un reste de l ancien minutage.
        "[v2][6:v]overlay=0:0:enable='between(t,21.4,39.8)'[v3]",
        "[v3][7:v]overlay=0:0:enable='between(t,40.4,45.0)'[v4]",
        "[v4][8:v]overlay=0:0:enable='between(t,8.2,20.8)'[out]",
    ]
    subprocess.run([
        'ffmpeg', '-y', '-v', 'error',
        '-i', REEL, '-i', sim, '-i', mosaique, '-i', couv,
        '-i', c['c1'], '-i', c['c2'], '-i', c['c3'], '-i', c['c4'],
        '-i', c['lab'],
        '-filter_complex', ';'.join(f), '-map', '[out]', '-an',
        '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', sortie], check=True)
    print()
    print('  ->  %s' % sortie)


if __name__ == '__main__':
    main()
