# -*- coding: utf-8 -*-
"""
11 - Exporter la politique en C, pour l ESP32.

    python 11_export_c.py                       -> ../firmware/robot_balancier/politique.h
    python 11_export_c.py --agent agents/xxx.zip
    python 11_export_c.py --tanh-rapide         approximation de Pade au lieu de tanhf

Produit un en-tete C autonome : les 1 634 poids, l inference, et la
construction de l observation. Aucune dependance hors math.h.

------------------------------------------------------------- CE QUI CHANGE
L agent remplace la boucle interne de ton firmware au MEME point d insertion.

    ancien   accel = Kp_a*err + Ki_a*integ + Kd_a*gyroFilt;
    nouveau  accel = politique_accel(...);          <- une ligne
             wheel_sps += accel * dt;              <- inchange

Tout le reste du firmware ne bouge pas : le timer, l ISR, l homme mort, le
filtre complementaire, la telemetrie, les securites.

--------------------------------------------------- CE QUE LE ROBOT N A PAS
Ton firmware lit 8 octets a partir de 0x3D : AY, AZ, TEMP, GX. Il n a donc
ni gyro Z, ni odometrie par roue -- donc ni vitesse de lacet ni cap. Mesure
en simulation : mettre ces trois entrees a zero ne coute RIEN (survie 100 %,
erreur d angle 0,085 contre 0,086 deg). Elles sont donc cablees a zero.

Meme constat pour les deux roues : l agent a ete entraine avec ses deux sorties
liees, et les lier ne coute rien. La direction du firmware (10 sept 2026)
s ajoute apres la politique, qui ne la voit pas.
"""
import argparse
import glob
import os
import re

import numpy as np

# Directement dans le croquis : une copie a la main entre l export et le
# firmware est exactement l endroit ou un en-tete non valide finit televerse.
SORTIE = os.path.join('..', 'firmware', 'robot_balancier')


def charger(chemin):
    from stable_baselines3 import PPO
    mdl = PPO.load(chemin, device='cpu')
    net = mdl.policy.mlp_extractor.policy_net
    W = []
    for couche in (net[0], net[2], mdl.policy.action_net):
        W.append((couche.weight.detach().numpy().astype(np.float64),
                  couche.bias.detach().numpy().astype(np.float64)))
    return W


def tableau_c(nom, a, par_ligne=6):
    """Un tableau C lisible, indente, avec la forme en commentaire."""
    plat = a.reshape(-1)
    lignes = []
    for i in range(0, len(plat), par_ligne):
        lignes.append('  ' + ' '.join('%14.8ef,' % v for v in plat[i:i + par_ligne]))
    forme = ('[%d][%d]' % a.shape) if a.ndim == 2 else ('[%d]' % a.shape[0])
    return ('static const float %s%s = {\n%s\n};\n' % (nom, forme, '\n'.join(lignes)))


ENTETE = r'''/* =====================================================================
   POLITIQUE APPRISE PAR RENFORCEMENT -- GENERE PAR 11_export_c.py
   NE PAS EDITER A LA MAIN.

   agent    : %(agent)s
   reseau   : 15 -> 32 -> 32 -> 2,  activation tanh,  %(nposids)d poids
   flash    : %(octets)d octets de constantes
   cadence  : la politique decide a 100 Hz, l integrateur reste a 200 Hz

   ---------------------------------------------------------------------
   USAGE, dans robot_balancier.ino

     #include "politique.h"

     // dans resetLoops() :
         politique_reset();

     // dans innerLoop(), a la place du calcul de accel :
         static bool tickPol = false;
         static float accelPol = 0.0f;
         tickPol = !tickPol;
         if (tickPol) {                       // 100 Hz : un tick sur deux
           accelPol = politique_accel(pitch - angleOffset, gyroFilt,
                                      wheel_sps, stepCount, target_speed_sps,
                                      millis());
           if (politique_defaut != POL_OK) { ... voir INTEGRATION.md }
         }
         float accel = accelPol;

   Le reste de innerLoop() ne change pas : la limite d acceleration deratee,
   l integration, l anti-emballement et applyMotorSpeed() restent en place.

   ---------------------------------------------------------------------
   CONVENTIONS -- a verifier sur la carte AVANT de le laisser debout

     angle   pitch - angleOffset, en DEGRES, positif = penche vers l AVANT
     gyro    gyroFilt, en deg/s, meme signe que la derivee de l angle
     sps     wheel_sps, la vitesse commandee en pas/s
     pas     stepCount, l odometrie en pas
     cible   target_speed_sps, la consigne de vitesse en pas/s

   Le signe de l angle est le point critique du transfert : s il est
   inverse, le robot part par terre en une demi-seconde. Le protocole de
   verification est dans firmware/INTEGRATION.md.
   ===================================================================== */

#ifndef POLITIQUE_H
#define POLITIQUE_H

#include <math.h>

#define POL_N_OBS   15
#define POL_N_CACHE 32
#define POL_N_ACT    2

/* --- grandeurs physiques, doivent correspondre a modele.py ------------- */
#define POL_RAD_PAR_PAS  (2.0f * 3.14159265358979f / 1600.0f)
#define POL_RAYON_ROUE   %(rayon).6ff        /* m */
#define POL_MAX_ACCEL    %(maxaccel).1ff     /* pas/s2, pleine echelle de l action */
#define POL_V_MAX        %(vmax).4ff         /* m/s, pleine echelle de la consigne */
#define POL_DT_POL       0.01f               /* s, periode de la politique */

/* --- garde-fous, tous CHIFFRES en simulation -------------------------------
   POL_ANGLE_MAX  au-dela de 11 deg l agent ne se rattrape plus JAMAIS
                  (mesure : 6/6 rattrapes a 10 deg, 0/6 a 12 deg, laches
                  immobiles). Ton FALL_LIMIT de 30 deg le laisse donc
                  s acharner 19 deg apres que c est perdu. Ici on coupe tot.
   POL_SAT_MS     en marche normale l action ne sature JAMAIS : 0,0 %% du temps
                  a |a| > 0,98, meme tres perturbe. Une saturation qui dure
                  est donc une anomalie franche, sans faux positif.
   Le test NaN    n est pas une precaution de principe : aucune comparaison
                  avec NaN n etant vraie, un NaN traverse EN SILENCE le
                  "if (a < MIN_SPS)" et le "if (a > lim)" de applyMotorSpeed,
                  et (uint32_t)(500000.0f/NaN) finit borne a 20 us de
                  demi-periode -- les moteurs partent a 25 000 pas/s.       */
#define POL_ANGLE_MAX    14.0f   /* deg, marge sur les 11 mesures */
#define POL_SAT_MS         120   /* ms de saturation continue tolerees */
#define POL_SAT_SEUIL    0.98f

enum {
  POL_OK = 0,
  POL_DEFAUT_ANGLE,      /* hors du domaine ou il sait se rattraper */
  POL_DEFAUT_SATURE,     /* commande collee a la butee trop longtemps */
  POL_DEFAUT_NAN         /* sortie non numerique */
};

'''

CORPS = r'''
/* ===================================================== etat de la politique */
static float pol_x_cible   = 0.0f;   /* integrale de la consigne, en metres  */
static float pol_a_prec[2]  = {0.0f, 0.0f};
static float pol_a_prec2[2] = {0.0f, 0.0f};
static unsigned long pol_sat_depuis = 0;   /* date du debut de saturation */

int politique_defaut = POL_OK;       /* a tester APRES chaque appel */

/* A APPELER A CHAQUE ARMEMENT, depuis resetLoops().
   Sans cela l agent repart avec l action qui precedait la chute et une
   cible de position periemee : il corrige une erreur qui n existe plus.
   C est le meme piege que les echeances en date absolue apres un reset. */
void politique_reset(void) {
  pol_x_cible = 0.0f;
  pol_a_prec[0]  = pol_a_prec[1]  = 0.0f;
  pol_a_prec2[0] = pol_a_prec2[1] = 0.0f;
  pol_sat_depuis = 0;
  politique_defaut = POL_OK;
}

/* Un flottant utilisable ? Ni NaN, ni infini. Ecrit sans isnan() pour ne
   dependre d aucune option de compilation : x != x n est vrai que pour NaN. */
static inline int pol_fini(float x) {
  return (x == x) && (x < 1e30f) && (x > -1e30f);
}

%(tanh)s

static inline float pol_borne(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* ============================================================== inference
   Deux couches cachees de 32, activation tanh, puis la couche de sortie.
   1 568 multiplications-accumulations et 64 tanh par appel.               */
static void pol_avant(const float obs[POL_N_OBS], float act[POL_N_ACT]) {
  float h0[POL_N_CACHE], h1[POL_N_CACHE];
  int i, j;

  for (i = 0; i < POL_N_CACHE; i++) {
    float s = pol_b0[i];
    for (j = 0; j < POL_N_OBS; j++) s += pol_w0[i][j] * obs[j];
    h0[i] = pol_tanh(s);
  }
  for (i = 0; i < POL_N_CACHE; i++) {
    float s = pol_b1[i];
    for (j = 0; j < POL_N_CACHE; j++) s += pol_w1[i][j] * h0[j];
    h1[i] = pol_tanh(s);
  }
  for (i = 0; i < POL_N_ACT; i++) {
    float s = pol_b2[i];
    for (j = 0; j < POL_N_CACHE; j++) s += pol_w2[i][j] * h1[j];
    act[i] = pol_borne(s, -1.0f, 1.0f);
  }
}

/* ==================================================== construire l observation
   Uniquement ce que le robot mesure. Les trois entrees indisponibles --
   gyro de lacet, ecart de cap, consigne de lacet -- sont a zero : mesure en
   simulation, ca ne change rien (survie 100 %%, erreur 0,085 contre 0,086 deg).

   L entree 5, l ecart de position, est SATUREE a +-1,75. Sans cette borne une
   derive accumulee sort du domaine vu a l entrainement et le reseau extrapole
   n importe quoi -- il se jette par terre. C est le meme probleme que MAX_LEAD
   resout dans la cascade.                                                    */
void politique_observation(float angle_deg, float gyro_deg_s, float sps,
                           long pas, float cible_sps, float obs[POL_N_OBS]) {
  float odo = (float)pas * POL_RAD_PAR_PAS * POL_RAYON_ROUE;   /* metres */
  float cons_v = cible_sps * POL_RAD_PAR_PAS * POL_RAYON_ROUE; /* m/s    */
  int k;

  pol_x_cible += cons_v * POL_DT_POL;

  obs[0]  = angle_deg / 10.0f;
  obs[1]  = gyro_deg_s / 100.0f;
  obs[2]  = 0.0f;                       /* gyro de lacet : pas lu par le firmware */
  obs[3]  = sps / 1600.0f;
  obs[4]  = sps / 1600.0f;              /* roues liees, comme a l entrainement    */
  obs[5]  = pol_borne((odo - pol_x_cible) / 0.20f, -1.75f, 1.75f);
  obs[6]  = 0.0f;                       /* ecart de cap : pas d odometrie par roue */
  obs[7]  = cons_v / POL_V_MAX;
  obs[8]  = 0.0f;                       /* consigne de lacet */
  obs[9]  = pol_a_prec[0];
  obs[10] = pol_a_prec[1];
  obs[11] = pol_a_prec2[0];
  obs[12] = pol_a_prec2[1];
  obs[13] = sinf(angle_deg * 0.01745329252f);
  obs[14] = cosf(angle_deg * 0.01745329252f);

  /* filet general : le reseau n extrapole pas, on lui garantit qu il ne
     verra jamais rien d inconnu. */
  for (k = 0; k < POL_N_OBS; k++) obs[k] = pol_borne(obs[k], -6.0f, 6.0f);
}

/* ====================================================== l appel de haut niveau
   Retourne une acceleration de roue en pas/s2, a integrer exactement comme
   la sortie de la cascade.                                                   */
float politique_accel(float angle_deg, float gyro_deg_s, float sps,
                      long pas, float cible_sps, unsigned long ms) {
  float obs[POL_N_OBS], act[POL_N_ACT], moy;

  /* --- garde-fou amont : une entree non numerique ne doit jamais atteindre
     le reseau, elle en ressortirait en NaN et traverserait tout le firmware */
  if (!pol_fini(angle_deg) || !pol_fini(gyro_deg_s) || !pol_fini(sps)) {
    politique_defaut = POL_DEFAUT_NAN;
    return 0.0f;
  }

  /* --- garde-fou d angle : au-dela il ne se rattrape plus, inutile d essayer */
  if (angle_deg > POL_ANGLE_MAX || angle_deg < -POL_ANGLE_MAX) {
    politique_defaut = POL_DEFAUT_ANGLE;
    return 0.0f;
  }

  politique_observation(angle_deg, gyro_deg_s, sps, pas, cible_sps, obs);
  pol_avant(obs, act);

  /* --- garde-fou aval */
  if (!pol_fini(act[0]) || !pol_fini(act[1])) {
    politique_defaut = POL_DEFAUT_NAN;
    return 0.0f;
  }

  pol_a_prec2[0] = pol_a_prec[0];
  pol_a_prec2[1] = pol_a_prec[1];
  pol_a_prec[0]  = act[0];
  pol_a_prec[1]  = act[1];

  /* roues liees a l entrainement : on moyenne les deux sorties */
  moy = 0.5f * (act[0] + act[1]);

  if (moy > POL_SAT_SEUIL || moy < -POL_SAT_SEUIL) {
    if (pol_sat_depuis == 0) pol_sat_depuis = ms;
    else if (ms - pol_sat_depuis > POL_SAT_MS) politique_defaut = POL_DEFAUT_SATURE;
  } else {
    pol_sat_depuis = 0;
  }

  return moy * POL_MAX_ACCEL;
}

#endif /* POLITIQUE_H */
'''

TANH_EXACT = '''/* tanh de la bibliotheque. Exact, mais ~1 a 2 us par appel sur ESP32
   (newlib, pas d instruction materielle) : 64 appels par pas de politique.
   Si le chronometrage sur la carte le demande, regenerer avec --tanh-rapide. */
static inline float pol_tanh(float x) { return tanhf(x); }'''

TANH_RAPIDE = '''/* Approximation de Pade de tanh, sans appel a la bibliotheque.
   Erreur maximale mesuree sur les entrees reelles du reseau : voir le
   rapport de 11_export_c.py. Environ 10 fois plus rapide que tanhf. */
static inline float pol_tanh(float x) {
  float x2;
  if (x >  4.0f) return  1.0f;
  if (x < -4.0f) return -1.0f;
  x2 = x * x;
  return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}'''


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent', type=str, default=None)
    ap.add_argument('--tanh-rapide', action='store_true')
    a = ap.parse_args()

    chemin = a.agent or sorted(
        glob.glob(os.path.join('agents', 'balancier_*_pas.zip')),
        key=lambda x: int(re.search(r'_(\d+)_pas', x).group(1)))[-1]

    import modele
    W = charger(chemin)
    n = sum(w.size + b.size for w, b in W)

    os.makedirs(SORTIE, exist_ok=True)
    with open(os.path.join(SORTIE, 'politique.h'), 'w', encoding='ascii') as f:
        f.write(ENTETE % dict(agent=chemin.replace('\\', '/'), nposids=n,
                              octets=n * 4, rayon=modele.V['rayon_roue'],
                              maxaccel=25000.0, vmax=0.15))
        for i, (w, b) in enumerate(W):
            f.write(tableau_c('pol_w%d' % i, w))
            f.write(tableau_c('pol_b%d' % i, b))
            f.write('\n')
        f.write(CORPS % dict(tanh=(TANH_RAPIDE if a.tanh_rapide else TANH_EXACT)))

    ko = os.path.getsize(os.path.join(SORTIE, 'politique.h')) / 1024
    # Trace de l agent exporte. Sans elle, 12_valider_c.py devinait l agent
    # de reference en prenant le dernier jalon _pas.zip -- et comparait donc
    # le C d un agent au Python d un autre. Ecart de 7,8 % annonce le
    # 9 sept 2026, alors que les deux codes etaient justes.
    with open(os.path.join(SORTIE, 'agent_source.txt'), 'w', encoding='ascii') as f:
        f.write(chemin)
    print('  agent    %s' % chemin)
    print('  poids    %d  (%d octets en flash)' % (n, n * 4))
    print('  ecrit    %s/politique.h   (%.0f ko de source)' % (SORTIE, ko))
    print('  tanh     %s' % ('approximation de Pade' if a.tanh_rapide else 'tanhf() exact'))
