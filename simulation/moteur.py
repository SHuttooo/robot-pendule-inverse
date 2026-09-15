# -*- coding: utf-8 -*-
"""
Le pas-a-pas, tel qu il est vraiment.

UN PAS-A-PAS N EST NI UNE SOURCE DE VITESSE, NI UNE SOURCE DE COUPLE.
C est un RESSORT MAGNETIQUE. Le driver impose l orientation du champ ; le
rotor s aligne dessus, avec un retard -- l angle de charge -- d autant plus
grand que la charge est forte. Le couple developpe vaut

    C = C_maintien * sin(p * (theta_commande - theta_rotor))

ou p est le nombre de paires de poles : 50 pour un moteur a 200 pas entiers.
Le maximum est atteint pour p*ecart = pi/2, soit un ecart d exactement
UN PAS ENTIER (1,8 deg). Au-dela, le sinus REDESCEND : le couple faiblit
alors que l ecart grandit, le rotor decroche et perd des poles. C est le
saut de pas -- une perte de position definitive, pas une saturation douce.

Consequences, toutes visibles en simulation :
  * a l arret, l erreur de position est nulle a un angle de charge pres ;
    il n y a pas d erreur de vitesse permanente ;
  * la raideur vaut p * C_maintien = 14 N.m/rad -- avec l inertie du rotor
    et de la roue, ca donne une resonance vers 110 Hz, la fameuse resonance
    mediane des pas-a-pas ;
  * le couple decroit avec la vitesse (force contre-electromotrice) ;
  * un couple de detente subsiste hors alimentation, a 4x la frequence
    electrique -- le crantage qu on sent en tournant l arbre a la main.

Ce qui n est PAS modelise ici : le hachage de courant du A4988, la
saturation magnetique, le chauffage, et l inegalite des micropas.
"""
import math


class PasAPas:
    """17HS3401S pilote par un A4988, en courant constant."""

    def __init__(self,
                 couple_maintien=0.28,     # N.m, fiche technique
                 paires_poles=50,          # 200 pas entiers / tour
                 pas_par_tour=1600.0,      # micropas effectifs (cmd 'M' du firmware)
                 omega_coupure=40.0,       # rad/s, chute de couple par la fcem
                 couple_detente=0.012,     # N.m, crantage a vide (~4 % du maintien)
                 amortissement=0.004):     # N.m/(rad/s), pertes fer + visqueux
        self.C0 = couple_maintien
        self.p = paires_poles
        self.rad_par_pas = 2 * math.pi / pas_par_tour
        self.w_c = omega_coupure
        self.C_det = couple_detente
        self.b = amortissement
        self.theta_cmd = 0.0               # position commandee, en radians
        self.reste = 0.0                   # fraction de pas non encore emise
        self.pas_emis = 0                  # l equivalent de stepCount du firmware
        self.decroche = False

    # ------------------------------------------------------------------
    def avancer(self, sps, dt):
        """Le timer emet des pas ENTIERS. On accumule la fraction, exactement
        comme l ISR du firmware : c est ce qui donne stepCount."""
        self.reste += sps * dt
        n = int(self.reste)                # troncature vers zero
        self.reste -= n
        self.pas_emis += n
        self.theta_cmd += n * self.rad_par_pas
        return n

    # ------------------------------------------------------------------
    def couple(self, theta, omega):
        """Couple developpe sur l arbre, en N.m."""
        ecart = self.theta_cmd - theta

        # chute de couple avec la vitesse : force contre-electromotrice
        derate = 1.0 / math.sqrt(1.0 + (omega / self.w_c) ** 2)

        c = self.C0 * derate * math.sin(self.p * ecart)
        c -= self.C_det * math.sin(4 * self.p * theta)     # crantage
        c -= self.b * omega                                # pertes

        # au-dela d un pas entier d ecart, le sinus redescend : on decroche
        self.decroche = abs(ecart) > math.pi / (2 * self.p)
        return c

    # ------------------------------------------------------------------
    def angle_de_charge_deg(self, theta):
        """L ecart commande / rotor, en degres mecaniques. Sous 1,8 deg le
        moteur tient ; au-dela il perd des pas."""
        return math.degrees(self.theta_cmd - theta)

    def resonance_hz(self, inertie):
        """La resonance mediane, celle qui fait chanter les pas-a-pas."""
        return math.sqrt(self.p * self.C0 / inertie) / (2 * math.pi)


if __name__ == '__main__':
    import modele
    mo = PasAPas()
    J_roue = 0.5 * modele.V['masse_roue'] * modele.V['rayon_roue'] ** 2
    J = J_roue + 1.2e-5                    # roue + rotor (armature)

    print('  couple de maintien        %6.3f N.m' % mo.C0)
    print('  raideur magnetique        %6.2f N.m/rad   (= p * C0)' % (mo.p * mo.C0))
    print('  ecart au couple maximum   %6.2f deg mecaniques  (= 1 pas entier)'
          % math.degrees(math.pi / (2 * mo.p)))
    print('  inertie roue + rotor      %.2e kg.m2' % J)
    print('  resonance mediane         %6.0f Hz' % mo.resonance_hz(J))
    print()
    print('  ANGLE DE CHARGE SOUS COUPLE  (ce que le vrai moteur fait)')
    print('   couple demande   angle de charge   etat')
    for c in (0.05, 0.10, 0.15, 0.20, 0.25, 0.28, 0.30):
        if c <= mo.C0:
            a = math.degrees(math.asin(c / mo.C0) / mo.p)
            print('     %.2f N.m          %5.2f deg       tient' % (c, a))
        else:
            print('     %.2f N.m              --          DECROCHE' % c)
    print()
    print('  CHUTE DE COUPLE AVEC LA VITESSE')
    print('    pas/s      rad/s     couple disponible')
    for sps in (400, 800, 1600, 2400, 2667, 4000):
        w = sps * mo.rad_par_pas
        print('    %5d    %6.2f      %5.3f N.m  (%3.0f %%)'
              % (sps, w, mo.C0 / math.sqrt(1 + (w / mo.w_c) ** 2),
                 100 / math.sqrt(1 + (w / mo.w_c) ** 2)))
