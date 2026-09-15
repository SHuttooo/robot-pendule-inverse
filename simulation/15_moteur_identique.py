"""Verifie que la transcription a plat de la loi du moteur, dans la boucle
chaude de balancier_env.py, donne EXACTEMENT le meme couple que moteur.py.

Pourquoi ce fichier existe : moteur.py est la reference lisible, mais il
tournait 40 fois par pas de politique et coutait 6 fois le temps de la
physique. La boucle de l environnement en porte donc une copie a plat. Deux
copies d une meme loi divergent toujours un jour ; celle-ci est verifiee.

    python 15_moteur_identique.py
"""
import math
import numpy as np
import moteur

N = 200000
alea = np.random.default_rng(0)

mo = moteur.PasAPas()
_sin, _sqrt = math.sin, math.sqrt
C0, pp, rpp = mo.C0, mo.p, mo.rad_par_pas
wc, cdet, bb = mo.w_c, mo.C_det, mo.b
seuil = math.pi / (2 * pp)

reste_plat, th_cmd_plat = 0.0, 0.0
ecart_max = 0.0
desaccord_decroche = 0

for k in range(N):
    sps = float(alea.uniform(-4000, 4000))
    dt = 0.0005
    th = float(alea.uniform(-20, 20))
    om = float(alea.uniform(-200, 200))

    # --- reference
    mo.avancer(sps, dt)
    c_ref = mo.couple(th, om)
    dec_ref = mo.decroche

    # --- transcription
    reste_plat += sps * dt
    n = int(reste_plat)
    if n:
        reste_plat -= n
        th_cmd_plat += n * rpp
    ecart = th_cmd_plat - th
    c_plat = (C0 * _sin(pp * ecart) / _sqrt(1.0 + (om / wc) ** 2)
              - cdet * _sin(4.0 * pp * th) - bb * om)
    e2 = ecart if ecart >= 0.0 else -ecart
    dec_plat = e2 > seuil

    ecart_max = max(ecart_max, abs(c_ref - c_plat))
    if dec_ref != dec_plat:
        desaccord_decroche += 1

print('  %d tirages, sps dans +/-4000, angle +/-20 rad, vitesse +/-200 rad/s' % N)
print('  ecart maximum sur le couple : %.3e N.m' % ecart_max)
print('  desaccords sur le decrochage : %d' % desaccord_decroche)
ok = ecart_max < 1e-12 and desaccord_decroche == 0
print()
print('  ' + ('IDENTIQUES.' if ok else 'ELLES DIVERGENT -- a corriger avant tout entrainement.'))
raise SystemExit(0 if ok else 1)
