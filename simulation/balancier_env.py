# -*- coding: utf-8 -*-
"""
06 - L environnement d apprentissage.

Les quatre objets du contrat, et rien d autre. Chaque choix est commente,
parce que ce sont eux qui decident du resultat -- pas l algorithme.

    python 06_env.py            verifie l environnement et mesure ses debits
    python 06_env.py --cascade  fait jouer TA cascade dedans, comme temoin

------------------------------------------------------------------ CADENCE
La politique decide a 100 Hz ; l integrateur de vitesse et la physique
tournent a 200 Hz et 10 kHz. Sur l ESP32 la boucle reste a 200 Hz, la
politique se contente d un tick sur deux et garde sa sortie entre-temps.

Pourquoi pas 200 Hz : a 200 Hz un episode de 10 s fait 2000 decisions et
l effet d une action isolee devient minuscule -- l attribution du merite
devient tres bruitee. 100 Hz est le compromis habituel.

--------------------------------------------------------------- OBSERVATION
Uniquement ce que le VRAI robot mesure. Aucune verite terrain : ni qpos, ni
masse, ni frottement. Un agent nourri d information privilegiee apprend un
robot qui n existe pas.

------------------------------------------------------------------- ACTION
Deux accelerations de roue, en pas/s2, integrees en vitesse -- exactement la
sortie de ta boucle interne. L agent se substitue a elle au meme point
d insertion. Deux valeurs et non une : le modele 3D permet de tourner.

--------------------------------------------------------------- RECOMPENSE
Prime de survie + suivi de consigne + regularisations. Les regularisations
ne servent pas a la performance, elles servent au TRANSFERT.
"""
import math
from collections import deque

import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces

import modele
import moteur
import etat

RAD_PAR_PAS = 2 * math.pi / 1600.0
RAYON = modele.V['rayon_roue']
VOIE = modele.V['voie']

HZ_POLITIQUE = 100.0
HZ_INTERNE = 200.0                  # l integrateur, comme le firmware
MAX_ACCEL = 25000.0                 # pas/s2, la valeur du carnet
DUREE_EPISODE = 10.0                # s

V_MAX_CONSIGNE = 0.15               # m/s
W_MAX_CONSIGNE = math.radians(60)   # rad/s de lacet


def _borner_pi(x):
    """Ramene un ecart d angle dans [-pi, pi]. Sans ca, un robot qui a fait un
    tour complet croit avoir une erreur de 360 deg."""
    return (x + math.pi) % (2 * math.pi) - math.pi


class Balancier(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, difficulte=0.0, actionneur='vitesse', graine=None,
                 avec_consignes=True):
        super().__init__()
        self.m = modele.construire(actionneur=actionneur, dimension='3d')
        self.d = mujoco.MjData(self.m)
        self.avec_consignes = avec_consignes

        # on garde les valeurs nominales pour pouvoir les re-perturber a chaque
        # episode SANS recompiler le modele (le chargement des maillages coute
        # bien plus cher que tout le reste)
        self.b_chassis = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
        self.masse0 = float(self.m.body_mass[self.b_chassis])
        self.inertie0 = self.m.body_inertia[self.b_chassis].copy()
        self.ipos0 = self.m.body_ipos[self.b_chassis].copy()
        self.frot0 = self.m.geom_friction.copy()

        self.n_interne = round(HZ_POLITIQUE and (1 / HZ_POLITIQUE) / (1 / HZ_INTERNE))
        self.n_phys = round((1 / HZ_INTERNE) / self.m.opt.timestep)
        self.dt_pol = 1 / HZ_POLITIQUE
        self.dt_int = 1 / HZ_INTERNE

        self.observation_space = spaces.Box(-np.inf, np.inf, (15,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)

        self.difficulte = float(difficulte)
        self._alea = np.random.default_rng(graine)
        # poussee imposee de l exterieur, pour le pilotage manuel (09_piloter.py).
        # Elle s ajoute a celle du curriculum, elle ne la remplace pas.
        self.force_externe = 0.0
        self.hauteur_externe = 0.21

        # ---------------------------------------------------- le pas-a-pas
        # En mode 'couple', MuJoCo ne recoit plus une consigne de vitesse mais
        # le COUPLE calcule par moteur.py a chaque pas de physique. Le rotor
        # suit alors sa commande comme une masse au bout d un ressort
        # magnetique -- ce que fait le vrai moteur, et ce que l actionneur
        # <velocity> ne fait pas.
        #
        # C est LA difference sim/reel identifiee le 9 septembre : l agent
        # entraine sur l actionneur en vitesse croit la roue instantanee, et
        # commande des inversions a plus de 5 Hz que le vrai moteur ne peut pas
        # suivre. Sur le robot il tient debout mais tremble dix fois plus que
        # la cascade.
        self.actionneur = actionneur
        self.moteurs = None
        if actionneur == 'couple':
            self.adr_q = []
            self.adr_v = []
            for nom in ('roue_d', 'roue_g'):
                j = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, nom)
                self.adr_q.append(int(self.m.jnt_qposadr[j]))
                self.adr_v.append(int(self.m.jnt_dofadr[j]))
            self.moteurs = [moteur.PasAPas(pas_par_tour=modele.V['pas_par_tour']),
                            moteur.PasAPas(pas_par_tour=modele.V['pas_par_tour'])]
            self.b_roues = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n)
                            for n in ('corps_roue_d', 'corps_roue_g')]
            # L inertie du ROTOR manquait au modele : le MJCF ne porte que la
            # roue. Elle vaut 1,2e-5 kg.m2, du meme ordre que la roue elle-meme
            # (1,58e-5 a 30 g) -- l oublier decale la resonance de 30 %.
            self.J_ROTOR = 1.2e-5

    # ================================================================ curriculum
    def regler_difficulte(self, x):
        """0 = facile (pas de poussee, pas de randomisation), 1 = tout."""
        self.difficulte = float(np.clip(x, 0.0, 1.0))

    # ================================================================ episode
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self._alea = np.random.default_rng(seed)
        r = self._alea
        # difficulte TIREE entre 0 et le maximum courant, pas egale au maximum :
        # une rampe pure fait oublier le cas facile.
        k = r.uniform(0.0, self.difficulte)
        self.k = k

        # ---- randomisation de domaine. On perturbe m, d et I directement :
        # incertitude.py a montre que tout se joue dans ces trois nombres.
        km = 1.0 + k * r.uniform(-0.30, 0.30)
        kh = 1.0 + k * r.uniform(-0.15, 0.15)
        ki = 1.0 + k * r.uniform(-0.25, 0.25)
        self.m.body_mass[self.b_chassis] = self.masse0 * km
        self.m.body_ipos[self.b_chassis] = self.ipos0 * kh
        self.m.body_inertia[self.b_chassis] = self.inertie0 * ki
        self.m.geom_friction[:] = self.frot0 * (1.0 + k * r.uniform(-0.5, 0.5))

        self.max_sps = r.uniform(1600 - k * 400, 1600 + k * 1200)
        self.bruit_gyro = modele.V['bruit_gyro'] * (1.0 + k * r.uniform(-0.5, 1.0))
        self.bruit_accel = modele.V['bruit_accel'] * (1.0 + k * r.uniform(-0.5, 1.0))
        self.biais_angle = k * r.uniform(-1.0, 1.0)          # deg
        self.retard = int(r.integers(0, 1 + int(k * 3)))     # pas de politique

        # ---- le pas-a-pas, recale sur la resonance MESUREE
        # La mesure du 9 septembre donne 71 Hz, contre 113 dans le modele. Elle
        # contraint le RAPPORT p*C0/J, pas ses deux facteurs : un couple plus
        # faible et des roues plus lourdes donnent la meme resonance. On tire
        # donc la masse de roue dans tout l intervalle compatible, et on en
        # deduit le couple qui redonne la resonance mesuree. C est exactement a
        # quoi sert la randomisation : couvrir ce qu on ne sait pas separer.
        if self.moteurs is not None:
            R = modele.V['rayon_roue']
            J_rotor = 1.2e-5
            f = modele.V['resonance_moteur'] * (1.0 + k * r.uniform(-0.15, 0.15))
            m_roue = r.uniform(0.030, 0.110)
            J = 0.5 * m_roue * R * R + J_rotor
            # La formule p*C0/J suppose la roue SEULE. Dans le robot le chassis
            # reagit au couple, ce qui alourdit l oscillation : mesure dans le
            # simulateur, la resonance sort 13 % sous la formule, de facon
            # stable de 30 a 110 g. On corrige donc le couple de 1/0,878^2 pour
            # que la resonance SIMULEE tombe sur les 71 Hz mesures -- c est la
            # simulation qu on recale sur le robot, pas une formule idealisee.
            CORR = 1.0 / 0.878 ** 2
            C0 = CORR * (2 * math.pi * f) ** 2 * J / 50.0   # p = 50 paires de poles
            # La masse tiree doit aussi aller dans MuJoCo, sinon le couple est
            # recale sur une inertie que la physique n a pas -- c est l erreur
            # qui donnait 100 Hz au lieu de 71.
            for b in self.b_roues:
                self.m.body_mass[b] = m_roue
                self.m.body_inertia[b] = [0.25 * m_roue * R * R,
                                          0.25 * m_roue * R * R,
                                          J]          # axe de rotation : roue + rotor
            for mo in self.moteurs:
                mo.C0 = C0
                mo.theta_cmd = 0.0
                mo.reste = 0.0
                mo.pas_emis = 0
                mo.decroche = False
            self.couple_maintien = C0
            self.masse_roue_tiree = m_roue

        # ---- pose de depart
        etat.poser(self.m, self.d,
                   tangage_deg=r.uniform(-1 - 14 * k, 1 + 14 * k),
                   lacet_deg=r.uniform(-180, 180))

        # ---- etat interne, l equivalent des variables du firmware
        self.sps = np.zeros(2)          # vitesse commandee de chaque roue
        self.pitch = etat.tangage(self.m, self.d)
        self.gyro_f = 0.0
        self.action_prec = np.zeros(2)
        self.action_prec2 = np.zeros(2)
        self.x_cible = 0.0              # integrale de la consigne = target_pos
        self.lacet_cible = 0.0          # integrale de la consigne de lacet
        self.lacet0 = math.radians(etat.lacet(self.m, self.d))   # cap de depart
        self.tampon = deque(maxlen=8)
        self.t_poussee = 1e9
        self.duree_poussee = 0.0
        self.force_poussee = 0.0
        self.hauteur_poussee = 0.21
        if k > 0.05:
            self._programmer_poussee(premiere=True)
        self.consigne = np.zeros(2)
        self._tirer_consigne()
        self.n_pas = 0
        o = np.clip(self._observer(), -6.0, 6.0)
        for _ in range(8):
            self.tampon.append(o)
        return o, {}

    def _programmer_poussee(self, premiere=False):
        """Ce qui bascule un pendule, ce n est ni la force ni la hauteur prises
        separement : c est l IMPULSION ANGULAIRE, couple x duree. On la tire
        elle, puis on repartit au hasard entre hauteur, duree et force.

        Ma premiere version tirait une force fixe a 210 mm pour tout l episode :
        l agent ne rencontrait jamais de bourrade breve et forte, ni de poussee
        basse. Or la hauteur vaut un facteur 7,6 sur la limite (mesure par
        10_limites.py) -- c etait le parametre le plus important, et il etait
        constant.

        Repere : la limite mesuree de la cascade au sommet vaut
        0,66 N x 0,21 m x 0,15 s = 0,021 N.m.s.
        """
        r = self._alea
        self.t_poussee = (2.0 + r.uniform(0, 2) if premiere
                          else self.d.time + r.uniform(2.0, 4.0))
        self.duree_poussee = r.uniform(0.05, 0.20)
        self.hauteur_poussee = r.uniform(0.08, 0.22)
        impulsion = r.uniform(0.0, 0.045) * self.k          # N.m.s
        f = impulsion / (self.hauteur_poussee * self.duree_poussee)
        self.force_poussee = r.choice([-1.0, 1.0]) * min(f, 8.0)

    def _tirer_consigne(self):
        if not self.avec_consignes:
            return
        r = self._alea
        # a faible difficulte on demande surtout de rester sur place
        amp = self.k
        self.consigne = np.array([
            r.uniform(-V_MAX_CONSIGNE, V_MAX_CONSIGNE) * amp,
            r.uniform(-W_MAX_CONSIGNE, W_MAX_CONSIGNE) * amp,
        ])
        self.t_consigne = self.d.time + r.uniform(2.0, 4.0)

    # ================================================================ capteurs
    def _observer(self):
        """Ce que le robot peut reellement lire. Rien d autre."""
        g = etat.gyro(self.m, self.d) + self._alea.normal(0, self.bruit_gyro, 3)
        a = etat.accel(self.m, self.d) + self._alea.normal(0, self.bruit_accel, 3)

        # filtre complementaire, celui du firmware
        gy = math.degrees(g[1])
        gyro_pitch = self.pitch + gy * self.dt_pol
        mag = math.hypot(a[0], a[2]) / 9.81
        if 0.6 < mag < 1.4:
            acc_pitch = math.degrees(math.atan2(-a[0], a[2]))
            alpha = 1.0 / (1.0 + self.dt_pol)
            self.pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch
        else:
            self.pitch = gyro_pitch
        self.gyro_f += (gy - self.gyro_f) * (self.dt_pol / (0.012 + self.dt_pol))

        pd, pg = etat.roues_rad(self.m, self.d)
        odo = (pd + pg) / 2 * RAYON                       # m parcourus, par les roues
        cap = (pd - pg) * RAYON / (2 * VOIE)              # rad de lacet, par odometrie

        return np.array([
            (self.pitch + self.biais_angle) / 10.0,
            self.gyro_f / 100.0,
            math.degrees(g[2]) / 100.0,                   # vitesse de lacet, gyro z
            self.sps[0] / 1600.0,
            self.sps[1] / 1600.0,
            # SATURE. Sans borne, une derive accumulee sort du domaine vu a
            # l entrainement (l ecart n y depasse jamais 1,1) et le reseau
            # extrapole n importe quoi : a 5,0 il se jette par terre en 0,6 s.
            # C est EXACTEMENT le probleme que MAX_LEAD resout dans ton
            # firmware -- la dette de position qui file et se rembourse d un
            # coup. Un regulateur y perd sa linearite, un reseau y perd tout.
            max(-1.75, min(1.75, (odo - self.x_cible) / 0.20)),
            _borner_pi(cap - self.lacet_cible) / 0.5,
            self.consigne[0] / V_MAX_CONSIGNE,
            self.consigne[1] / W_MAX_CONSIGNE,
            self.action_prec[0],
            self.action_prec[1],
            # a(t-2) : indispensable pour que la politique puisse maitriser sa
            # propre derivee seconde. Sans memoire dans le reseau, l information
            # doit venir de l observation.
            self.action_prec2[0],
            self.action_prec2[1],
            math.sin(math.radians(self.pitch)),           # redondance utile
            math.cos(math.radians(self.pitch)),
        ], dtype=np.float32)

    # --------------------------------------------------------------------
    #  Filet de securite general : aucune entree ne sort de +-6. Un reseau ne
    #  sait pas extrapoler, il faut donc lui garantir qu il ne verra jamais
    #  rien d inconnu. Bornes larges pour ne rien deformer dans le domaine
    #  normal, et strictes au-dela.
    # --------------------------------------------------------------------

    # ==================================================================== pas
    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)

        if self.avec_consignes and self.d.time > self.t_consigne:
            self._tirer_consigne()

        # ---- integration de l accélération en vitesse, comme le firmware
        self.charge_max = 0.0
        self.n_decroche = 0
        # La loi du moteur est recopiee a plat ici, hors de moteur.py. Elle y
        # tourne 40 fois par pas de politique (2 moteurs x 10 pas de physique
        # x 2 pas internes) : les appels de methode et les acces d attributs
        # y coutaient 6 fois le temps de la physique elle-meme. moteur.py
        # reste la reference lisible et testable ; ceci en est la transcription
        # chaude, et 15_moteur_identique.py verifie qu elles coincident.
        mos = self.moteurs
        if mos is not None:
            _sin = math.sin; _sqrt = math.sqrt
            C0 = mos[0].C0; pp = mos[0].p; rpp = mos[0].rad_par_pas
            wc = mos[0].w_c; cdet = mos[0].C_det; bb = mos[0].b
            seuil = math.pi / (2 * pp)
            th_cmd = [mos[0].theta_cmd, mos[1].theta_cmd]
            reste = [mos[0].reste, mos[1].reste]
            dtp = self.m.opt.timestep
            qpos = self.d.qpos; qvel = self.d.qvel; ctrl = self.d.ctrl
            aq = self.adr_q; av = self.adr_v
            chmax = 0.0; ndec = 0
        for _ in range(self.n_interne):
            self.sps += a * MAX_ACCEL * self.dt_int
            self.sps = np.clip(self.sps, -self.max_sps, self.max_sps)
            sps_l = (float(self.sps[0]), float(self.sps[1]))
            if self.moteurs is None:
                self.d.ctrl[:] = np.round(self.sps) * RAD_PAR_PAS

            self.d.qfrc_applied[:] = 0
            # Direction AVANT DU ROBOT, pas +x du monde. Le robot a un cap, et
            # il change quand il tourne : une force le long de x pousserait de
            # cote des qu il a pivote. On ne garde que le lacet (pas le
            # tangage) pour que la poussee reste horizontale.
            lac = math.radians(etat.lacet(self.m, self.d))
            avant = np.array([math.cos(lac), math.sin(lac), 0.0])
            if self.force_externe != 0.0:
                pt = self.d.xpos[self.b_chassis] + np.array(
                    [0.0, 0.0, self.hauteur_externe])
                mujoco.mj_applyFT(
                    self.m, self.d, self.force_externe * avant,
                    np.zeros(3), pt, self.b_chassis, self.d.qfrc_applied)
            if self.t_poussee <= self.d.time < self.t_poussee + self.duree_poussee:
                pt = self.d.xpos[self.b_chassis] + np.array(
                    [0.0, 0.0, self.hauteur_poussee])
                mujoco.mj_applyFT(self.m, self.d, self.force_poussee * avant,
                                  np.zeros(3), pt, self.b_chassis,
                                  self.d.qfrc_applied)
            for _ in range(self.n_phys):
                if mos is not None:
                    # Le couple se recalcule a CHAQUE pas de physique, pas a
                    # chaque pas de commande : la resonance mesuree est a 71 Hz,
                    # il faut la RESOUDRE, pas l echantillonner a 200 Hz.
                    # A 2 kHz de physique on a 28 points par periode.
                    for i in (0, 1):
                        reste[i] += sps_l[i] * dtp
                        n = int(reste[i])
                        if n:
                            reste[i] -= n
                            th_cmd[i] += n * rpp
                        th = qpos[aq[i]]
                        om = qvel[av[i]]
                        ecart = th_cmd[i] - th
                        ctrl[i] = (C0 * _sin(pp * ecart) / _sqrt(1.0 + (om / wc) ** 2)
                                   - cdet * _sin(4.0 * pp * th) - bb * om)
                        # L angle de charge est l ecart entre ou le champ dit au
                        # rotor d etre et ou il est. A 1,8 deg le couple est
                        # maximal ; au-dela le sinus redescend et le moteur
                        # decroche. C est la mesure physique de "j en demande
                        # trop a mon moteur", et elle n existe pas du tout avec
                        # l actionneur en vitesse.
                        e2 = ecart if ecart >= 0.0 else -ecart
                        if e2 > chmax: chmax = e2
                        if e2 > seuil: ndec += 1
                mujoco.mj_step(self.m, self.d)

        if mos is not None:
            mos[0].theta_cmd, mos[1].theta_cmd = th_cmd
            mos[0].reste, mos[1].reste = reste
            self.charge_max = math.degrees(chmax)
            self.n_decroche = ndec

        if self.k > 0.05 and self.d.time > self.t_poussee + self.duree_poussee:
            self._programmer_poussee()

        # ---- la cible de position suit l integrale de la consigne
        self.x_cible += self.consigne[0] * self.dt_pol
        self.lacet_cible += self.consigne[1] * self.dt_pol

        o = np.clip(self._observer(), -6.0, 6.0)
        self.tampon.append(o)
        obs = self.tampon[max(0, len(self.tampon) - 1 - self.retard)]

        # ================================================== la recompense
        tang = etat.tangage(self.m, self.d)
        roul = etat.roulis(self.m, self.d)
        v = float(np.mean(self.sps)) * RAD_PAR_PAS * RAYON        # m/s commandes
        w = float(self.sps[0] - self.sps[1]) * RAD_PAR_PAS * RAYON / (2 * VOIE)
        pd, pg = etat.roues_rad(self.m, self.d)
        err_x = (pd + pg) / 2 * RAYON - self.x_cible

        # --------------------------------------------------------------
        #  REGLE : la recompense d un pas doit rester POSITIVE tant que le
        #  robot est debout. Sinon terminer l episode rapporte plus que le
        #  poursuivre, et l agent apprend a se laisser tomber.
        #
        #  Ma premiere version penalisait la vitesse de roue en quadratique :
        #  a 1200 pas/s la penalite valait 2,08 contre 1,0 de prime de survie.
        #  Or un balancier DOIT lancer ses roues pour se rattraper -- je
        #  penalisais exactement ce qui le sauve. Resultat : il tombait au
        #  bout de 0,9 s et n en bougeait plus, apres 600 000 pas.
        #
        #  D ou des termes en exp(-x^2) : bornes dans [0, 1], ils ne peuvent
        #  jamais ecraser la prime de survie. C est la forme standard des
        #  recompenses de locomotion.
        # --------------------------------------------------------------
        def cloche(erreur, echelle):
            return math.exp(-(erreur / echelle) ** 2)

        r = 1.0                                                   # survie
        r += 0.60 * cloche(tang, 4.0)                             # rester droit
        r += 1.20 * cloche(v - self.consigne[0], 0.06)            # suivre la vitesse
        # ---- le cap. Deux termes, et il faut les deux.
        #  Suivre la VITESSE de lacet ne suffit pas : une petite erreur
        #  permanente s integre en une derive de cap sans limite. Le robot
        #  tournerait lentement sur lui-meme sans jamais rien payer.
        #  Le second terme regarde le CAP lui-meme, donc l integrale.
        w_reel = float(etat.gyro(self.m, self.d)[2])              # rad/s, gyro z
        err_cap = _borner_pi(math.radians(etat.lacet(self.m, self.d))
                             - self.lacet0 - self.lacet_cible)
        r += 0.40 * cloche(w_reel - self.consigne[1], 0.35)       # vitesse de lacet
        r += 0.30 * cloche(err_cap, 0.20)                         # cap tenu (11 deg)
        r += 0.40 * cloche(err_x, 0.15)                           # rester a sa place
        r -= 0.02 * float(np.mean((self.sps / 1600.0) ** 2))      # effort, petit

        # ---------------------------------------------------------------
        #  LISSAGE. Deux termes, et c est la difference entre les deux qui
        #  permet de tuer l oscillation SANS tuer la correction.
        #
        #    premiere difference   a(t) - a(t-1)
        #        penalise TOUT changement, y compris une rampe franche et
        #        legitime. A doser leger, sinon l agent devient mou.
        #
        #    seconde difference    a(t) - 2a(t-1) + a(t-2)
        #        vaut ZERO pour une rampe a pente constante -- une grosse
        #        correction soutenue ne coute donc rien -- et devient enorme
        #        des que la commande s INVERSE. C est exactement l oscillation.
        #
        #  Vu en frequence : la premiere difference est un filtre de gain
        #  proportionnel a w, la seconde a w^2. A 50 Hz de tremblement contre
        #  5 Hz de correction utile, la premiere penalise 10x plus, la seconde
        #  100x plus. C est cette selectivite qu on cherche.
        # ---------------------------------------------------------------
        d1 = a - self.action_prec
        d2 = a - 2.0 * self.action_prec + self.action_prec2
        r -= 0.02 * float(np.sum(d1 ** 2))                        # rampes : leger
        r -= 0.35 * float(np.sum(d2 ** 2))                        # inversions : TRES cher

        # ---------------------------------------------------------------
        #  RESPECT DU MOTEUR. Nouveau le 9 septembre 2026, et c est le terme
        #  qui manquait vraiment.
        #
        #  Les deux termes ci-dessus penalisent la FORME de la commande : ils
        #  devinent ce qui fatiguera le moteur. Celui-ci mesure ce que le
        #  moteur SUBIT, l angle de charge, qui n existe que depuis que
        #  moteur.py est branche.
        #
        #  Sur le robot du 9 septembre, l agent tenait debout mais tremblait
        #  dix fois plus que la cascade (commande d ecart-type 893 pas/s contre
        #  158) parce qu il avait appris sur une roue qui obeit instantanement.
        #  Un ecart de charge proche de 1,8 deg veut dire qu il demande le
        #  couple maximum ; au-dela le moteur perd des pas et la commande ne
        #  veut plus rien dire.
        # ---------------------------------------------------------------
        if self.moteurs is not None:
            r -= 0.30 * (self.charge_max / 1.8) ** 2
            r -= 0.50 * float(self.n_decroche > 0)     # decrochage : franchement puni

        self.action_prec2 = self.action_prec
        self.action_prec = a
        self.n_pas += 1

        tombe = abs(tang) > 30.0 or abs(roul) > 30.0
        fini = self.d.time >= DUREE_EPISODE
        return obs, float(r), bool(tombe), bool(fini), {
            'tangage': tang, 'v': v, 'action': a.copy(),
            'd2': float(np.sum(d2 ** 2)),
            'cap': math.degrees(err_cap), 'w': w_reel}


