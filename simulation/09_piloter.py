# -*- coding: utf-8 -*-
"""
09 - Piloter le robot, avec une console.

Deux fenetres : celle de MuJoCo pour regarder, et un petit pupitre pour
commander. Curseurs pour les consignes, boutons pour pousser, une bascule
AGENT / CASCADE a chaud, et deux traces qui defilent.

    python 09_piloter.py
    python 09_piloter.py --cascade                     demarrer sur la cascade
    python 09_piloter.py --agent agents/balancier_2000000_pas.zip
    python 09_piloter.py --difficulte 0.6              robot randomise

Pourquoi pas le clavier dans la fenetre MuJoCo : le viewer s approprie presque
toutes les lettres pour ses bascules d affichage (Z lumiere, Q cameras, S
echelle d inertie, D point de selection, H enveloppe convexe, P decoupe des
contacts...). Le callback utilisateur recoit la touche EN PLUS, sans pouvoir
l intercepter : un appui declenchait deux actions.

Tkinter tient la boucle principale et cadence la simulation par root.after ;
le viewer passif se contente d un sync(). La souris marche toujours dans la
fenetre MuJoCo : double-clic pour selectionner, Ctrl + clic DROIT + glisser.
"""
import argparse
import collections
import csv
import datetime
import glob
import math
import os
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import mujoco
import mujoco.viewer

from balancier_env import (Balancier, MAX_ACCEL, HZ_POLITIQUE, RAD_PAR_PAS,
                           RAYON, V_MAX_CONSIGNE, W_MAX_CONSIGNE)
from firmware import Firmware

DUREE_POUSSEE = 0.15
FOND, CADRE, ENCRE, ENCRE2 = '#161f27', '#1d2830', '#e3e9ee', '#99a6b1'
TRAIT, ACCENT, ALERTE, VERT = '#37434d', '#4eb8d8', '#e0a055', '#5fc4a2'
N_TRACE = 300                                   # 3 s a 100 Hz

ap = argparse.ArgumentParser()
ap.add_argument('--agent', type=str, default=None)
ap.add_argument('--cascade', action='store_true')
ap.add_argument('--difficulte', type=float, default=0.0)
a = ap.parse_args()

chemin = a.agent or sorted(glob.glob(os.path.join('agents', '*.zip')),
                           key=os.path.getmtime)[-1]
from stable_baselines3 import PPO
mdl = PPO.load(chemin, device='cpu')
fw = Firmware(dual=True)

env = Balancier(difficulte=a.difficulte, avec_consignes=False)
obs, _ = env.reset()
env.t_poussee = 1e9                 # aucune poussee automatique : c est toi qui pousses
BRUITS = (env.bruit_gyro, env.bruit_accel)

E = {'fin_poussee': -1.0, 'sens': 1.0, 'agent': not a.cascade}
# Memoire tampon des 1,5 s precedentes. Une chute ne se diagnostique pas sur
# l instant ou elle arrive mais sur ce qui l a precedee : c est la seconde
# d avant qui contient la cause.
BOITE_NOIRE = collections.deque(maxlen=150)
N_CHUTE = [0]

T_ANGLE = collections.deque([0.0] * N_TRACE, maxlen=N_TRACE)
T_VMES = collections.deque([0.0] * N_TRACE, maxlen=N_TRACE)
T_VCON = collections.deque([0.0] * N_TRACE, maxlen=N_TRACE)


def pilote_cascade(o):
    fw.pitch = float(o[0]) * 10.0
    fw.gyroFilt = float(o[1]) * 100.0
    fw.wheel_sps = float(np.mean(env.sps))
    corr = fw.Kp_spd * (fw.wheel_sps - env.consigne[0] / (RAD_PAR_PAS * RAYON))
    corr += fw.Ki_spd * (float(o[5]) * 0.20 / (RAD_PAR_PAS * RAYON))
    cible = -max(-6.0, min(6.0, corr))
    accel = fw.Kp_a * (fw.pitch - cible) + fw.Kd_a * fw.gyroFilt
    return np.clip(np.array([accel, accel]) / MAX_ACCEL, -1.0, 1.0)


# ============================================================== le pupitre
root = tk.Tk()
root.title('Pupitre du balancier')
root.configure(bg=FOND)
root.geometry('420x660')

st = ttk.Style()
st.theme_use('clam')
st.configure('.', background=FOND, foreground=ENCRE, fieldbackground=CADRE,
             bordercolor=TRAIT, lightcolor=CADRE, darkcolor=CADRE)
st.configure('TLabel', background=FOND, foreground=ENCRE2, font=('Segoe UI', 9))
st.configure('Titre.TLabel', foreground=ACCENT, font=('Segoe UI', 8, 'bold'))
st.configure('Val.TLabel', foreground=ENCRE, font=('Consolas', 11))
st.configure('TButton', background=CADRE, foreground=ENCRE, borderwidth=1,
             font=('Segoe UI', 9))
st.map('TButton', background=[('active', TRAIT)])
st.configure('TScale', background=FOND, troughcolor=CADRE)
st.configure('TCheckbutton', background=FOND, foreground=ENCRE2)


def titre(parent, texte):
    ttk.Label(parent, text=texte.upper(), style='Titre.TLabel').pack(
        anchor='w', padx=14, pady=(14, 2))


def separateur(parent):
    tk.Frame(parent, bg=TRAIT, height=1).pack(fill='x', padx=14, pady=(8, 0))


# ---------------------------------------------------------------- pilote
titre(root, 'pilote')
cadre_pil = tk.Frame(root, bg=FOND)
cadre_pil.pack(fill='x', padx=14)
lbl_pilote = tk.Label(cadre_pil, text='', bg=CADRE, fg=ACCENT,
                      font=('Segoe UI', 12, 'bold'), width=12, pady=6)
lbl_pilote.pack(side='left')


def basculer():
    E['agent'] = not E['agent']
    fw.__init__(dual=True)
    maj_pilote()


def maj_pilote():
    lbl_pilote.config(text='AGENT' if E['agent'] else 'CASCADE',
                      fg=ACCENT if E['agent'] else ALERTE)


ttk.Button(cadre_pil, text='basculer', command=basculer).pack(side='left', padx=8)
ttk.Button(cadre_pil, text='redresser', command=lambda: redresser()).pack(side='left')

# ------------------------------------------------------------- consignes
separateur(root)
titre(root, 'consignes')
cadre_c = tk.Frame(root, bg=FOND)
cadre_c.pack(fill='x', padx=14)

v_var = tk.DoubleVar(value=0.0)
w_var = tk.DoubleVar(value=0.0)
lbl_v = ttk.Label(cadre_c, text='', style='Val.TLabel', width=14)
lbl_w = ttk.Label(cadre_c, text='', style='Val.TLabel', width=14)


def maj_consignes(*_):
    env.consigne[0] = v_var.get()
    env.consigne[1] = math.radians(w_var.get())
    lbl_v.config(text='%+.3f m/s' % v_var.get())
    lbl_w.config(text='%+.0f deg/s' % w_var.get())


ttk.Label(cadre_c, text='avancer').grid(row=0, column=0, sticky='w')
lbl_v.grid(row=0, column=1, sticky='e')
ttk.Scale(cadre_c, from_=-V_MAX_CONSIGNE, to=V_MAX_CONSIGNE, variable=v_var,
          command=maj_consignes, length=380).grid(row=1, column=0, columnspan=2)
ttk.Label(cadre_c, text='tourner').grid(row=2, column=0, sticky='w', pady=(8, 0))
lbl_w.grid(row=2, column=1, sticky='e', pady=(8, 0))
ttk.Scale(cadre_c, from_=-math.degrees(W_MAX_CONSIGNE),
          to=math.degrees(W_MAX_CONSIGNE), variable=w_var,
          command=maj_consignes, length=380).grid(row=3, column=0, columnspan=2)


def stop():
    v_var.set(0.0)
    w_var.set(0.0)
    maj_consignes()


ttk.Button(cadre_c, text='tout arreter', command=stop).grid(
    row=4, column=0, columnspan=2, pady=8, sticky='ew')

# --------------------------------------------------------------- poussee
separateur(root)
titre(root, 'poussee')
cadre_p = tk.Frame(root, bg=FOND)
cadre_p.pack(fill='x', padx=14)

f_var = tk.DoubleVar(value=1.0)
h_var = tk.DoubleVar(value=210.0)
lbl_f = ttk.Label(cadre_p, text='', style='Val.TLabel', width=14)
lbl_h = ttk.Label(cadre_p, text='', style='Val.TLabel', width=14)


def maj_poussee(*_):
    lbl_f.config(text='%.1f N' % f_var.get())
    lbl_h.config(text='%.0f mm' % h_var.get())
    env.hauteur_externe = h_var.get() / 1000.0


ttk.Label(cadre_p, text='force').grid(row=0, column=0, sticky='w')
lbl_f.grid(row=0, column=1, sticky='e')
ttk.Scale(cadre_p, from_=0.1, to=5.0, variable=f_var, command=maj_poussee,
          length=380).grid(row=1, column=0, columnspan=2)
ttk.Label(cadre_p, text='hauteur / essieu').grid(row=2, column=0, sticky='w', pady=(8, 0))
lbl_h.grid(row=2, column=1, sticky='e', pady=(8, 0))
ttk.Scale(cadre_p, from_=0.0, to=250.0, variable=h_var, command=maj_poussee,
          length=380).grid(row=3, column=0, columnspan=2)


def pousser(sens):
    E['sens'] = sens
    E['fin_poussee'] = env.d.time + DUREE_POUSSEE


b = tk.Frame(cadre_p, bg=FOND)
b.grid(row=4, column=0, columnspan=2, pady=8, sticky='ew')
ttk.Button(b, text='<<  pousser arriere', command=lambda: pousser(-1.0)).pack(
    side='left', expand=True, fill='x', padx=(0, 4))
ttk.Button(b, text='pousser avant  >>', command=lambda: pousser(1.0)).pack(
    side='left', expand=True, fill='x', padx=(4, 0))

bruit_var = tk.BooleanVar(value=True)


def maj_bruit():
    env.bruit_gyro, env.bruit_accel = (BRUITS if bruit_var.get() else (0.0, 0.0))


ttk.Checkbutton(cadre_p, text='bruit des capteurs', variable=bruit_var,
                command=maj_bruit).grid(row=5, column=0, columnspan=2, sticky='w')

# --------------------------------------------------------------- mesures
separateur(root)
titre(root, 'mesures')
lbl_mes = tk.Label(root, text='', bg=FOND, fg=ENCRE, justify='left',
                   font=('Consolas', 10))
lbl_mes.pack(anchor='w', padx=14)

cnv = tk.Canvas(root, width=392, height=150, bg=CADRE, highlightthickness=1,
                highlightbackground=TRAIT)
cnv.pack(padx=14, pady=(8, 14))


def enregistrer_chute(info):
    """Ecrit la boite noire sur disque et resume en une ligne."""
    os.makedirs('chutes', exist_ok=True)
    N_CHUTE[0] += 1
    horo = datetime.datetime.now().strftime('%H%M%S')
    nom = os.path.join('chutes', 'chute_%s_%02d.csv' % (horo, N_CHUTE[0]))
    with open(nom, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t', 'angle_deg', 'gyro_deg_s', 'sps_moyen', 'ecart_pos_mm',
                    'v_mesure', 'v_consigne', 'lacet_deg_s', 'force_N',
                    'hauteur_mm', 'pilote'])
        w.writerows(BOITE_NOIRE)
    deb = BOITE_NOIRE[0] if BOITE_NOIRE else [0] * 11
    fin = BOITE_NOIRE[-1] if BOITE_NOIRE else [0] * 11
    forces = [l[8] for l in BOITE_NOIRE if l[8]]
    print()
    print('  ===== CHUTE %d  ->  %s' % (N_CHUTE[0], nom))
    print('    pilote            %s' % ('agent' if E['agent'] else 'cascade'))
    print('    angle final       %+.1f deg' % fin[1])
    print('    poussee           %s'
          % ('%.2f N a %.0f mm, %d pas' % (forces[0], fin[9], len(forces))
             if forces else 'aucune dans la derniere seconde'))
    print('    consigne          %+.3f m/s   %+.0f deg/s'
          % (fin[6], math.degrees(env.consigne[1])))
    print('    ecart de position %+.0f mm  ->  %+.0f mm' % (deb[4], fin[4]))
    print('    vitesse roue      %+.0f  ->  %+.0f pas/s   (max %.0f)'
          % (deb[3], fin[3], env.max_sps))
    sat = sum(abs(l[3]) >= 0.98 * env.max_sps for l in BOITE_NOIRE)
    print('    saturation        %.0f %% de la derniere seconde'
          % (100.0 * sat / max(1, len(BOITE_NOIRE))))
    print()


def redresser():
    global obs
    cons = env.consigne.copy()
    obs, _ = env.reset()
    env.t_poussee = 1e9
    env.consigne[:] = cons
    maj_bruit()
    E['fin_poussee'] = -1.0
    E['chute_signalee'] = False
    for d in (T_ANGLE, T_VMES, T_VCON):
        d.clear()
        d.extend([0.0] * N_TRACE)


def tracer(info):
    cnv.delete('all')
    L, H = 392, 150
    # --- trace du haut : angle, +-3 deg
    cnv.create_text(6, 8, text='angle  +-3 deg', anchor='w', fill=ENCRE2,
                    font=('Consolas', 8))
    cnv.create_line(0, 38, L, 38, fill=TRAIT)
    pts = []
    for i, v in enumerate(T_ANGLE):
        pts += [i * L / N_TRACE, 38 - max(-28, min(28, v / 3.0 * 28))]
    cnv.create_line(pts, fill=ACCENT, width=1)
    # --- trace du bas : vitesse, consigne et mesure
    cnv.create_text(6, 82, text='vitesse   consigne / mesure   +-0,15 m/s',
                    anchor='w', fill=ENCRE2, font=('Consolas', 8))
    cnv.create_line(0, 116, L, 116, fill=TRAIT)
    for serie, coul, larg in ((T_VCON, ALERTE, 1), (T_VMES, VERT, 1)):
        pts = []
        for i, v in enumerate(serie):
            pts += [i * L / N_TRACE, 116 - max(-30, min(30, v / 0.15 * 30))]
        cnv.create_line(pts, fill=coul, width=larg)
    if env.force_externe:
        cnv.create_rectangle(L - 60, 4, L - 4, 20, fill=ALERTE, outline='')
        cnv.create_text(L - 32, 12, text='POUSSEE', fill=FOND,
                        font=('Segoe UI', 7, 'bold'))


# ============================================================== la boucle
vue = mujoco.viewer.launch_passive(env.m, env.d, show_left_ui=False,
                                   show_right_ui=False)
DT = 1.0 / HZ_POLITIQUE
PAS_PAR_RAFRAICHISSEMENT = 2                    # 20 ms d horloge, 50 Hz d affichage


def boucle():
    global obs
    if not vue.is_running():
        root.destroy()
        return
    t0 = time.time()
    for _ in range(PAS_PAR_RAFRAICHISSEMENT):
        action = (mdl.predict(obs, deterministic=True)[0] if E['agent']
                  else pilote_cascade(obs))
        env.force_externe = (E['sens'] * f_var.get()
                             if env.d.time < E['fin_poussee'] else 0.0)
        obs, r, tombe, fini, info = env.step(action)
        if vue.perturb.active and vue.perturb.select > 0:
            mujoco.mjv_applyPerturbForce(env.m, env.d, vue.perturb)
        T_ANGLE.append(info['tangage'])
        T_VMES.append(info['v'])
        T_VCON.append(env.consigne[0])
        BOITE_NOIRE.append([
            round(env.d.time, 3), round(info['tangage'], 3),
            round(math.degrees(env.gyro_f * 0 + info['w']), 2)
            if False else round(float(obs[1]) * 100.0, 2),
            round(float(np.mean(env.sps)), 1),
            round(float(obs[5]) * 200.0, 1),
            round(info['v'], 4), round(env.consigne[0], 4),
            round(math.degrees(info['w']), 2),
            round(env.force_externe, 3), round(env.hauteur_externe * 1000, 0),
            'agent' if E['agent'] else 'cascade'])
        # 'fini' signale les 10 s de l episode d entrainement. Ici on pilote
        # sans limite de duree, donc on l ignore -- surtout, on NE REMET PAS
        # d.time a zero : la fin de poussee est une date absolue, et la
        # remettre a zero relancait la force pour dix secondes. C est ce qui
        # faisait tomber le robot a 1 N, une poussee qu il encaisse largement.
        if tombe:
            if not E.get('chute_signalee'):
                E['chute_signalee'] = True
                enregistrer_chute(info)
            lbl_mes.config(fg=ALERTE)
            break
    else:
        lbl_mes.config(fg=ENCRE)

    lbl_mes.config(text=(
        'angle   %+7.2f deg%s\n'
        'vitesse %+7.3f m/s   consigne %+6.3f\n'
        'lacet   %+7.1f d/s   consigne %+6.0f\n'
        'cap     %+7.1f deg'
        % (info['tangage'], '   -- CHUTE, redresser' if tombe else '',
           info['v'], env.consigne[0],
           math.degrees(info['w']), math.degrees(env.consigne[1]), info['cap'])))
    tracer(info)
    vue.sync()
    reste = int(1000 * (PAS_PAR_RAFRAICHISSEMENT * DT - (time.time() - t0)))
    root.after(max(1, reste), boucle)


maj_pilote()
maj_consignes()
maj_poussee()
print('  agent  %s' % chemin)
print('  Le pupitre est dans la seconde fenetre.')
root.after(50, boucle)
root.protocol('WM_DELETE_WINDOW', lambda: (vue.close(), root.destroy()))
root.mainloop()
try:
    vue.close()
except Exception:
    pass
print('  fin.')
