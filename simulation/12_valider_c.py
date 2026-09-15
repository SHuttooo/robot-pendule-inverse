# -*- coding: utf-8 -*-
"""
12 - Verifier que le C donne exactement les memes actions que Python.

    python 12_valider_c.py

Le C genere n est utile que s il reproduit la politique au bit pres. On
compile firmware/robot_balancier/politique.h avec gcc, on lui donne des observations issues
d un vrai episode, et on compare a stable-baselines3.

Sans ce test, exporter des poids revient a esperer -- une transposition de
matrice ou un ordre d activation inverse ne se voit pas autrement qu en
regardant le robot tomber.
"""
import os
import subprocess
import sys

import numpy as np

SORTIE = os.path.join('..', 'firmware', 'robot_balancier')   # celui qu on televerse
SCRATCH = os.environ.get('TEMP', '.')

PROG = r'''
#include <stdio.h>
#include "politique.h"

int main(void) {
  float obs[POL_N_OBS], act[POL_N_ACT];
  int i;
  /* on lit des observations DEJA construites, pour tester le reseau seul */
  while (1) {
    for (i = 0; i < POL_N_OBS; i++)
      if (scanf("%f", &obs[i]) != 1) return 0;
    pol_avant(obs, act);
    printf("%.9f %.9f\n", act[0], act[1]);
  }
  return 0;
}
'''


def main():
    if not os.path.exists(os.path.join(SORTIE, 'politique.h')):
        raise SystemExit('  lance d abord : python 11_export_c.py')

    # --- 1. des observations REELLES, issues d un episode simule
    from balancier_env import Balancier
    from stable_baselines3 import PPO
    import glob, re
    # L agent de reference doit etre CELUI QU ON A EXPORTE, pas le dernier
    # jalon. Le 9 septembre 2026 cette ligne devinait, et le validateur a
    # annonce 7,8 % d ecart alors que les deux codes etaient justes : il
    # comparait le C de balancier_final au Python de balancier_8600000_pas.
    # L en-tete genere porte la reponse, on la lit.
    chemin = None
    trace = os.path.join(SORTIE, 'agent_source.txt')
    if os.path.exists(trace):
        chemin = open(trace, encoding='ascii').read().strip()
    if not chemin:
        for l in open(os.path.join(SORTIE, 'politique.h'), encoding='ascii',
                      errors='replace').readlines()[:30]:
            m = re.search(r'agent\s*:\s*(\S+)', l)
            if m:
                chemin = m.group(1)
                break
    if not chemin or not os.path.exists(chemin):
        raise SystemExit('  impossible de retrouver l agent exporte ; relance 11_export_c.py')
    print('  agent de reference : %s  (celui inscrit dans l en-tete)' % chemin)
    mdl = PPO.load(chemin, device='cpu')

    env = Balancier(difficulte=0.6, avec_consignes=True, graine=1234)
    O = []
    for ep in range(6):
        o, _ = env.reset(seed=1234 + ep)
        for _ in range(400):
            O.append(o.copy())
            a, _ = mdl.predict(o, deterministic=True)
            o, r, t, f, i = env.step(a)
            if t or f:
                break
    O = np.array(O, dtype=np.float32)
    print('  %d observations issues de %d episodes reels' % (len(O), 6))

    # --- 2. les actions de reference, cote Python
    A_py, _ = mdl.predict(O, deterministic=True)
    A_py = np.clip(np.asarray(A_py, dtype=np.float64), -1.0, 1.0)

    # --- 3. compilation et execution du C
    src = os.path.join(SCRATCH, 'test_politique.c')
    exe = os.path.join(SCRATCH, 'test_politique.exe')
    with open(src, 'w', encoding='ascii') as f:
        f.write(PROG)
    r = subprocess.run(['gcc', '-O2', '-I', os.path.abspath(SORTIE),
                        src, '-o', exe, '-lm'],
                       capture_output=True, text=True)
    if r.returncode:
        print(r.stderr[:2000])
        raise SystemExit('  la compilation a echoue')
    print('  compile avec gcc -O2')

    entree = '\n'.join(' '.join('%.9g' % v for v in ligne) for ligne in O)
    r = subprocess.run([exe], input=entree, capture_output=True, text=True)
    A_c = np.array([[float(x) for x in l.split()]
                    for l in r.stdout.strip().splitlines()])

    # --- 4. le verdict
    if A_c.shape != A_py.shape:
        raise SystemExit('  formes differentes : %s vs %s' % (A_c.shape, A_py.shape))
    d = np.abs(A_c - A_py)
    print()
    print('=' * 62)
    print('  C  CONTRE  PYTHON')
    print('=' * 62)
    print('  ecart maximum      %.3e' % d.max())
    print('  ecart moyen        %.3e' % d.mean())
    print('  amplitude action   %.4f  (ecart-type %.4f)' % (np.abs(A_py).max(), A_py.std()))
    print('  ecart relatif      %.2e' % (d.max() / max(1e-9, np.abs(A_py).std())))
    print()
    seuil = 2e-5
    if d.max() < seuil:
        print('  >>> IDENTIQUE a %.0e pres. L export est fidele.' % seuil)
        print('      (l ecart residuel est la difference float32 / float64)')
    else:
        print('  >>> ECART TROP GRAND. Ne pas televerser.')
        k = int(np.argmax(d.max(axis=1)))
        print('      pire observation, indice %d :' % k)
        print('        obs    %s' % np.round(O[k], 4))
        print('        python %s' % np.round(A_py[k], 6))
        print('        C      %s' % np.round(A_c[k], 6))
        sys.exit(1)


if __name__ == '__main__':
    main()
