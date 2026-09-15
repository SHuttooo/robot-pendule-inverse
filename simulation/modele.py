# -*- coding: utf-8 -*-
"""
Le modele, et d ou viennent ses chiffres.

    python modele.py            regenere modeles/balancier.xml
    python modele.py --etat     liste les parametres et leur provenance

DEPUIS L EXPORT D ASSEMBLAGE, la geometrie ne se suppose plus : elle se lit.
Les 6 STL partagent le repere de l assemblage, donc les positions relatives
sont exactes. Les maillages PORTENT LA MASSE : MuJoCo calcule leur tenseur
d inertie sur la vraie forme, ce qui vaut infiniment mieux que mes boites.
Ils portent AUSSI la collision, mais MuJoCo les convexifie : il travaille sur
l enveloppe convexe de chaque piece, pas sur le cadre ajoure. C est faux dans le
detail et sans importance -- ca ne sert qu apres une chute, quand le robot a
deja perdu. Le contact qui compte, celui des pneus, reste un cylindre : plus
juste qu un maillage, et plus rapide. Cout de la collision des maillages :
-24 % de vitesse de simulation, soit 71x le temps reel au lieu de 94x.

Il ne reste donc a mesurer que des MASSES, plus aucune forme.

REPERE DE L ASSEMBLAGE (mm)          REPERE DU MODELE (m)
  x = entraxe des roues                x = avant du robot
  y = avant-arriere                    y = axe des roues
  z = vertical                         z = vertical
  origine quelconque                   origine = l axe des roues
L arbre du moteur 1 a ete localise dans le maillage (cylindre de 5,00 mm) :
il donne l axe a (74,10 ; 39,68 ; 1021,00). C est l origine du modele.
"""
import argparse
import os

# ==========================================================================
#  MESURE   mesure, fiche technique, ou lu dans l assemblage
#  DEDUIT   calcule a partir d une MESURE
#  SUPPOSE  invente pour que le modele tourne. A remplacer.
# ==========================================================================
P = {
    # ---------------------------------------------------------------- roues
    'rayon_roue':   (0.0325, 'MESURE',  'roues de 65 mm. Le carnet supposait 90 :'
                                        ' ses distances en mm sont 1,38x trop grandes.'),
    'largeur_roue': (0.013,  'MESURE',  'demi-epaisseur, roue de 26 mm.'),
    'voie':         (0.0691, 'DEDUIT',  'demi-entraxe. Arbres a 84,5 mm dans l assemblage,'
                                        ' roue de 26 mm montee en bout. Sans effet en plan.'),
    'masse_roue':   (0.030,  'ESTIME',  'roue plastique 65x26 mm avec moyeu. La resonance'
                                        ' mesuree a 71 Hz autorise jusqu a 110 g si le couple'
                                        ' est celui de la fiche. Une balance trancherait.'),
    'pas_par_tour': (1600.0, 'MESURE',  'tranche le 9 sept 2026 par balayage en frequence.'
                                        ' Le gain d actionnement mesure vaut -4,628 +/- 0,146 ;'
                                        ' a 1600 le modele en est a 12 %, a 3200 a 77 %.'
                                        ' Et le rapport gravite/actionnement, qui ne depend'
                                        ' d aucune masse, doit valoir g : 8,86 a 1600 contre'
                                        ' 4,43 a 3200.'),

    # -------------------------------------------------------------- moteurs
    'masse_moteur': (0.220,  'MESURE',  '17HS3401S, fiche technique. A CONFIRMER.'),
    'couple_max':   (0.20,   'MESURE',  'N.m. Couple de maintien 0,28 derate.'
                                        ' Au-dela, le pas-a-pas decroche.'),

    # ------------------------------- pieces imprimees, PLA
    # Masses ESTIMEES, pas devinees : le volume vient du STL, la densite
    # effective de l epaisseur caracteristique 2V/A. Une piece mince est
    # presque toute en perimetres (dense), une piece epaisse est surtout du
    # remplissage (legere). Modele : 2 perimetres de 0,4 mm par face pleins,
    # le reste a 20 %, PLA a 1,24 g/cm3.
    #          volume    2V/A    densite eff.
    # piece1   92,8 cm3  8,01 mm   0,45      41 g
    # piece2   65,2 cm3  4,15 mm   0,63      41 g
    # piece3  118,1 cm3  5,04 mm   0,56      67 g
    # piece4  103,5 cm3  6,17 mm   0,51      52 g
    'masse_piece1': (0.041,  'ESTIME',  '92,8 cm3 a 0,45 g/cm3. +-30 % selon le'
                                        ' remplissage reel.'),
    'masse_piece2': (0.041,  'ESTIME',  '65,2 cm3 a 0,63 g/cm3.'),
    'masse_piece3': (0.067,  'ESTIME',  '118,1 cm3 a 0,56 g/cm3.'),
    'masse_piece4': (0.052,  'ESTIME',  '103,5 cm3 a 0,51 g/cm3.'),

    # ------------------------------- absents de la CAO
    # 4 cellules lithium 3,7 V en serie (4S = 14,8 V, coherent avec le carnet).
    # 3 posees SUR la piece du haut, 1 collee SOUS elle. La piece4 va de +136
    # a +199 mm de l essieu, une 18650 fait 18 mm de diametre.
    'masse_bat_haut': (0.163, 'ESTIME',  '3 x 18650 CONFIRMEES : 46 g piece + 25 g de'
                                         ' supports et cablage.'),
    'h_bat_haut':     (0.208, 'DEDUIT',  'posees sur piece4 (sommet +199 mm) + 9 mm de rayon.'),
    'masse_bat_bas':  (0.054, 'ESTIME',  '1 x 18650 confirmee, 46 g + colle.'),
    'h_bat_bas':      (0.182, 'DEDUIT',  'collee sous le PLATEAU du haut, a l etage'
                                         ' superieur. Le plateau va de +191 a +199 mm ;'
                                         ' 191 - 9 mm de rayon.'),
    'masse_elec':     (0.070, 'ESTIME',  'petite Labdec + ESP32 + 2 A4988 + cablage.'),
    'h_elec':         (0.075, 'DEDUIT',  'posee sur l etagere du bas. Les surfaces'
                                         ' horizontales du maillage sont a +66, +141 et'
                                         ' +199 mm ; +66 + 9 mm de demi-epaisseur.'),

    # ------------------------------------------------------------- capteurs
    'h_mpu':        (0.060, 'SUPPOSE', 'sans effet sur la dynamique, seulement sur'
                                       ' l accelerometre en rotation rapide.'),
    'bruit_gyro':   (0.00108, 'MESURE', 'rad/s = 0,062 deg/s. Robot IMMOBILE, moteurs'
                                        ' muets, 6 s a 200 Hz, 9 sept 2026. Conforme a la'
                                        ' fiche du MPU-6050 sur 100 Hz de bande.'
                                        ' L ancienne valeur 0,019 (1,088 deg/s) etait la'
                                        ' VIBRATION des pas-a-pas, pas le capteur : 17x trop'
                                        ' grande. La vibration doit venir de moteur.py, pas'
                                        ' d un terme de bruit -- elle est correlee a'
                                        ' l activite moteur, un bruit blanc ne l est pas.'),
    'bruit_accel':  (0.205, 'MESURE',  'm/s2. Mesure le 9 sept 2026, moteurs actifs, donc'
                                       ' borne haute : le chiffre contient la vibration.'),
    'vibration_gyro': (0.034, 'MESURE', 'rad/s = 1,95 deg/s. Ce que le gyro voit EN PLUS du'
                                        ' bruit quand les moteurs tournent. A reproduire par'
                                        ' moteur.py, pas a injecter.'),
    'resonance_moteur': (71.0, 'MESURE', 'Hz. Resonance du rotor+roue contre le ressort'
                                        ' magnetique, 9 sept 2026 : 60 % de la puissance du'
                                        ' gyro entre 60 et 80 Hz moteurs tournants contre'
                                        ' 6 % a l arret, pic fixe alors que la vitesse varie'
                                        ' d un facteur 9. Le modele en predisait 113.'
                                        ' Impose p*C0/J = (2 pi 71)^2, soit 0,395 fois la'
                                        ' valeur supposee -- couple plus faible, roues plus'
                                        ' lourdes, ou les deux.'),

    # ---------------------------------------------------------------- monde
    'frottement':   (1.0,   'SUPPOSE', 'pneu/sol. Se mesure sur un plan incline :'
                                       ' mu = tan(angle de glissement).'),
    'timestep':     (0.0005,'CHOISI',  's. 10 pas de physique par pas de commande'
                                       ' (la boucle tourne a 200 Hz).'),
}

V = {k: v[0] for k, v in P.items()}

# ==========================================================================
#  L assemblage. Origine du modele = l axe des roues, localise dans le
#  maillage du moteur 1 (l arbre de 5,00 mm).
# ==========================================================================
# Les STL vivent dans hardware/, pas ici. Chemin ABSOLU calcule a l execution :
# from_xml_string resout un meshdir relatif depuis le dossier courant, pas
# depuis ce fichier, et un script lance d ailleurs ne trouverait plus rien.
MESHDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       '..', 'hardware', 'cao', 'stl_assemblage')).replace('\\', '/')
# Dans modeles/balancier.xml on ecrit au contraire un chemin RELATIF : lu par
# from_xml_path, il se resout depuis le dossier du fichier, et le XML versionne
# ne porte pas le chemin d une machine.
MESHDIR_XML = '../../hardware/cao/stl_assemblage'
AXE = (0.0741, 0.03968, 1.02100)          # m, dans le repere de l assemblage

# Rotation Rz(-90) : (x, y, z) -> (y, -x, z). Elle envoie l entraxe de
# l assemblage (x) sur l axe des roues du modele (y).
EULER = '0 0 -90'
TRANS = '%.5f %.5f %.5f' % (-AXE[1], AXE[0], -AXE[2])

PIECES = [
    # nom       fichier                              parametre de masse
    ('moteur_d', 'Assemblage1 - nema17HS3401S-1.STL', 'masse_moteur'),
    ('moteur_g', 'Assemblage1 - nema17HS3401S-2.STL', 'masse_moteur'),
    ('piece1',   'Assemblage1 - piece1-1.STL',        'masse_piece1'),
    ('piece2',   'Assemblage1 - piece2-1.STL',        'masse_piece2'),
    ('piece3',   'Assemblage1 - piece3-1.STL',        'masse_piece3'),
    ('piece4',   'Assemblage1 - piece4-1.STL',        'masse_piece4'),
]
TEINTES = {'moteur_d': '0.28 0.30 0.33 1', 'moteur_g': '0.28 0.30 0.33 1'}
TEINTE_IMPRIME = '0.60 0.63 0.66 1'


def bloc_assemblage(v):
    ass = '\n'.join(
        '    <mesh name="%s" file="%s" scale="0.001 0.001 0.001"/>' % (n, f)
        for n, f, _ in PIECES)
    geo = '\n'.join(
        '      <geom name="%s" type="mesh" mesh="%s" pos="%s" euler="%s"\n'
        '            mass="%.4f" rgba="%s"/>'
        % (n, n, TRANS, EULER, v[mk], TEINTES.get(n, TEINTE_IMPRIME))
        for n, _, mk in PIECES)
    return ass, geo


GABARIT = """<mujoco model="balancier">
  <!-- GENERE PAR modele.py -- ne pas editer a la main.
       Modifier les valeurs dans modele.py puis relancer : python modele.py

       Reperes :  +x = avant   |   tangage > 0 = penche en avant
                  vitesse de roue > 0 = le robot avance
       Origine du chassis : l axe des roues.
  -->
  <compiler angle="degree" autolimits="true" meshdir="{meshdir}"/>
  <option timestep="{timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>
    <global azimuth="130" elevation="-16" offwidth="1920" offheight="1440"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
{assets_mesh}
    <texture type="skybox" builtin="gradient" rgb1="0.26 0.31 0.36" rgb2="0.06 0.08 0.11"
             width="512" height="512"/>
    <texture name="damier" type="2d" builtin="checker" rgb1="0.19 0.21 0.23"
             rgb2="0.25 0.27 0.29" width="300" height="300"/>
    <material name="sol" texture="damier" texrepeat="14 14" reflectance="0.1"/>
  </asset>

  <default>
    <geom friction="{frottement} 0.005 0.0001"/>
    <default class="roue">
      <geom type="cylinder" size="{rayon_roue} {largeur_roue}" zaxis="0 1 0"
            mass="{masse_roue}" rgba="0.11 0.11 0.13 1"/>
      <!-- armature = inertie du rotor pas-a-pas ramenee a l axe (34 g.cm2),
           du meme ordre que celle de la roue. L oublier fausse la dynamique. -->
      <joint type="hinge" axis="0 1 0" armature="1.2e-5" damping="2e-5"/>
    </default>
  </default>

  <worldbody>
    <light pos="0.6 -0.6 1.8" dir="-0.3 0.3 -1" directional="true" diffuse="0.55 0.55 0.55"/>
    <geom name="sol" type="plane" size="5 5 0.1" material="sol"/>

    <!-- Chassis : corps racine. Modele 3D par defaut : le robot reel tourne
         depuis le 10 septembre 2026 (un accumulateur de pas par roue). Le
         modele plan reste disponible, plus rapide, quand la direction ne
         compte pas. -->
    <body name="chassis" pos="0 0 {rayon_roue}">
{articulations}

      <!-- L ASSEMBLAGE. Ces maillages portent la masse ET la collision.
           MuJoCo deduit l inertie de la vraie forme, mais convexifie pour le
           contact : le robot tombe sur son enveloppe, pas sur son cadre. -->
{geoms_mesh}

      <!-- ABSENTS DE LA CAO. Positions et masses a etablir. La batterie est
           le terme dominant de l inertie : son bras de levier intervient au
           carre. Se tromper de 3 cm dessus compte plus que tout le reste. -->
      <geom name="bat_haut" type="box" size="0.033 0.030 0.009" pos="0 0 {h_bat_haut}"
            mass="{masse_bat_haut}" contype="0" conaffinity="0" rgba="0.85 0.45 0.15 1"/>
      <geom name="bat_bas" type="cylinder" size="0.009 0.033" zaxis="0 1 0"
            pos="0 0 {h_bat_bas}"
            mass="{masse_bat_bas}" contype="0" conaffinity="0" rgba="0.85 0.45 0.15 1"/>
      <geom name="elec" type="box" size="0.028 0.030 0.009" pos="0 0 {h_elec}"
            mass="{masse_elec}" contype="0" conaffinity="0" rgba="0.20 0.55 0.35 1"/>

      <!-- Le MPU-6050 : x devant, z en haut. Sur le vrai robot il est couche,
           d ou angleOffset = -91,4 deg ; ici la verticale vaut 0. -->
      <site name="mpu" pos="0 0 {h_mpu}" size="0.005" rgba="0 0.85 0.85 1"/>

      <body name="corps_roue_d" pos="0  {voie} 0">
        <joint name="roue_d" class="roue"/>
        <geom  name="roue_d" class="roue"/>
        <geom name="rayon_d" type="box" size="{rayon_visuel} 0.0135 0.004" pos="0 0 0"
              mass="0" rgba="0.72 0.74 0.77 1" contype="0" conaffinity="0"/>
      </body>
      <body name="corps_roue_g" pos="0 -{voie} 0">
        <joint name="roue_g" class="roue"/>
        <geom  name="roue_g" class="roue"/>
        <geom name="rayon_g" type="box" size="{rayon_visuel} 0.0135 0.004" pos="0 0 0"
              mass="0" rgba="0.72 0.74 0.77 1" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <!-- Freins desactives. Les activer a chaud (data.eq_active) bloque les roues :
       distingue "pendule sur son essieu" de "robot qui bascule en bloc". -->
  <equality>
    <joint name="frein_d" joint1="roue_d" active="false"/>
    <joint name="frein_g" joint1="roue_g" active="false"/>
  </equality>

  <!-- Un pas-a-pas suit sa consigne de vitesse tant qu il ne decroche pas :
       actionneur en VITESSE plafonne en couple. Au-dela, il perd des pas.
       ctrlrange en rad/s : 3000 pas/s = 3000*2pi/{pas_par_tour} = {ctrl_max} rad/s -->
{bloc_actionneur}

  <!-- Les MEMES grandeurs que le vrai robot : le filtre complementaire du
       firmware se porte tel quel. Le bruit est ajoute cote Python
       (bruit_gyro, bruit_accel) : l attribut noise a disparu de MuJoCo 3.x. -->
  <sensor>
    <gyro          name="gyro"  site="mpu"/>
    <accelerometer name="accel" site="mpu"/>
    <jointpos      name="pos_roue_d" joint="roue_d"/>
    <jointpos      name="pos_roue_g" joint="roue_g"/>
    <!-- Verite terrain, pour la MESURE seulement -- jamais pour la commande
         ni pour l observation d un agent : le vrai robot ne les a pas. Passer
         par des capteurs plutot que par qpos rend tous les scripts
         independants du choix plan / 3D. -->
    <framequat name="orientation" objtype="body" objname="chassis"/>
    <framepos  name="position"    objtype="body" objname="chassis"/>
  </sensor>
</mujoco>
"""


ACTIONNEUR_VITESSE = '''  <!-- MODELE SIMPLE : le moteur suit sa consigne de vitesse tant qu il ne
       decroche pas. Rapide, suffisant pour degrossir, mais il laisse une
       erreur de vitesse permanente sous charge -- ce qu un vrai pas-a-pas
       ne fait pas. Voir moteur.py.
       ctrlrange en rad/s : 3000 pas/s = 3000*2pi/{pas_par_tour} = {ctrl_max} rad/s -->
  <actuator>
    <velocity name="roue_d" joint="roue_d" kv="0.5"
              ctrlrange="-{ctrl_max} {ctrl_max}" forcerange="-{couple_max} {couple_max}"/>
    <velocity name="roue_g" joint="roue_g" kv="0.5"
              ctrlrange="-{ctrl_max} {ctrl_max}" forcerange="-{couple_max} {couple_max}"/>
  </actuator>'''

ACTIONNEUR_COUPLE = '''  <!-- MODELE FIDELE : MuJoCo ne recoit qu un COUPLE. C est moteur.py qui le
       calcule a chaque pas de physique, selon la loi du ressort magnetique
       C = C_maintien * sin(50 * (theta_commande - theta_rotor)). Le decrochage,
       l angle de charge, le crantage et la chute de couple avec la vitesse en
       decoulent naturellement -- ils ne sont pas imposes. -->
  <actuator>
    <motor name="roue_d" joint="roue_d" gear="1" ctrlrange="-0.5 0.5"/>
    <motor name="roue_g" joint="roue_g" gear="1" ctrlrange="-0.5 0.5"/>
  </actuator>'''


PLAN = '''      <!-- Modele PLAN : 3 libertes, pas de lacet. Plus rapide, et suffisant
           tant qu on ne commande pas de direction. Il correspondait au
           firmware d avant le 10 septembre 2026, ou les deux STEP partageaient
           un timer. -->
      <joint name="glissiere_x" type="slide" axis="1 0 0"/>
      <joint name="glissiere_z" type="slide" axis="0 0 1"/>
      <joint name="tangage"     type="hinge" axis="0 1 0"/>'''

TROIS_D = '''      <!-- Modele 3D : articulation libre, 6 libertes. Necessaire des qu on
           veut TOURNER -- le lacet n existe tout simplement pas dans le modele
           plan. Les deux roues deviennent commandables separement, comme sur
           le vrai robot depuis le 10 septembre 2026. -->
      <freejoint name="libre"/>'''


def xml(actionneur='vitesse', dimension='3d', **surcharges):
    v = dict(V, **surcharges)
    v['articulations'] = TROIS_D if dimension == '3d' else PLAN
    if actionneur == 'couple':
        # La raideur magnetique resonne a 71 Hz -- MESURE sur le robot le
        # 9 septembre 2026, contre 113 supposes auparavant. Le pas de 0,0001 s
        # avait ete choisi pour les 113 Hz, et par prudence. Verifie depuis :
        # a 0,0005 s la resonance simulee sort a 71,7 Hz contre 74,3 a 0,0001,
        # et l integration reste stable meme sous decrochage force a 6000 pas/s
        # avec les deux extremes de la plage de roues (30 et 110 g).
        # Le mode couple coute donc desormais le meme pas de physique que le
        # mode vitesse, et l entrainement passe de 34 a 9 minutes.
        v.setdefault('timestep', 0.0005)
        v['timestep'] = min(v['timestep'], 0.0005)
        v['bloc_actionneur'] = ACTIONNEUR_COUPLE
    else:
        v['bloc_actionneur'] = ACTIONNEUR_VITESSE
    ass, geo = bloc_assemblage(v)
    v['meshdir'] = MESHDIR
    v['assets_mesh'] = ass
    v['geoms_mesh'] = geo
    v['rayon_visuel'] = round(v['rayon_roue'] * 0.86, 4)
    v['ctrl_max'] = round(3000 * 2 * 3.141592653589793 / v['pas_par_tour'], 2)
    v['bloc_actionneur'] = v['bloc_actionneur'].format(**v)
    return GABARIT.format(**v)


def construire(actionneur='vitesse', dimension='3d', **surcharges):
    import mujoco
    return mujoco.MjModel.from_xml_string(xml(actionneur, dimension, **surcharges))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--etat', action='store_true')
    a = ap.parse_args()

    if a.etat:
        rangs = {'MESURE': 0, 'DEDUIT': 1, 'CHOISI': 2, 'ESTIME': 3,
                 'DOUTEUX': 4, 'SUPPOSE': 5}
        print('%-17s %9s  %-8s  %s' % ('parametre', 'valeur', 'source', 'note'))
        print('-' * 104)
        for k, (val, src, note) in sorted(P.items(), key=lambda x: (rangs[x[1][1]], x[0])):
            print('%-17s %9.4f  %-8s  %s' % (k, val, src, note))
        n = sum(1 for _, s, _ in P.values() if s in ('SUPPOSE', 'DOUTEUX'))
        print('-' * 104)
        print('%d parametres sur %d restent a etablir -- et ce sont TOUS des masses'
              ' ou des positions absentes de la CAO.' % (n, len(P)))
        print('La geometrie, elle, est desormais lue dans l assemblage. Voir MESURES.md.')
    else:
        with open('modeles/balancier.xml', 'w', encoding='utf-8') as f:
            f.write(xml().replace('meshdir="%s"' % MESHDIR, 'meshdir="%s"' % MESHDIR_XML))
        print('modeles/balancier.xml regenere depuis l assemblage.')
