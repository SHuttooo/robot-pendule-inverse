"""La mosaique des jalons, au format telephone.

    python 19_mosaique_verticale.py

Trois reglages, tous corriges apres coup.

FORMAT. La version d origine est en 4/3 : posee dans une video verticale elle
laissait deux enormes bandes noires. Mais la remplir sur toute la hauteur
etouffe : "c est illisible et bizarre". Compromis retenu, 1080x1440, qu on pose
ensuite dans le cadre 1920 en laissant une bande en haut POUR LE TEXTE, qui ne
doit jamais passer par-dessus les cases.

CAMERA. La vue serree du montage cote a cote colle trop aux robots une fois
mise en vignette. On reprend la vue large.

CASES. 4 colonnes sur 4 lignes, soit 270x310 : presque carre, ce qui laisse de
l air autour d un robot debout.
"""
import os
import film

SORTIE = os.path.join('rendus', 'v3_moteur_fidele')

if __name__ == '__main__':
    film.ACTIONNEUR = 'couple'
    film.VUE = 'film'          # large, pas la vue serree du cote a cote
    os.makedirs(SORTIE, exist_ok=True)
    film.SORTIE = SORTIE
    images = film.mosaique(1080, 1440, 4, 4)
    film.ecrire(images, 'mosaique_verticale', 1080, 1440)
