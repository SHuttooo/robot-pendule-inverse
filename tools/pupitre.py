"""Pupitre de commande du robot balancier.

    python tools\\pupitre.py

Ne parle PAS au port serie : il est deja tenu par serial_monitor.ps1, et deux
processus sur le meme port ne cohabitent pas. Le pupitre passe donc par les
memes deux fichiers que tout le reste du projet --  il lit logs\\robot.log et
il depose ses commandes dans tools\\cmd.txt. Le moniteur reste seul maitre du
port, ce qui evite de reintroduire la classe de panne qu on a passe la soiree
du 9 septembre a eliminer.

Conséquence utile : on peut lancer et fermer le pupitre autant de fois qu on
veut sans jamais perturber le robot.
"""
import os
import re
import time
import tkinter as tk
from tkinter import font as tkfont

RACINE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG = os.path.join(RACINE, 'logs', 'robot.log')
CMD = os.path.join(RACINE, 'tools', 'cmd.txt')

RAYON = 0.0325
PAS_PAR_TOUR = 1600.0
MS_PAR_PAS = 2 * 3.141592653589793 * RAYON / PAS_PAR_TOUR   # m/s par pas/s
SPS_MAX = 800                      # 0,102 m/s a l echelle 1
LACET_MAX = 600                    # pas/s de difference, a l echelle 1
ECHELLES = (0.5, 1.0, 2.0, 3.0)    # le firmware plafonne a 3000 et 2000

FOND = '#14161a'
CARTE = '#1c1f26'
TEXTE = '#e6e8ec'
GRIS = '#8a90a0'
ACCENT = '#5aa9e6'
OCRE = '#e0a458'
ROUGE = '#e05a5a'
VERT = '#5ae08a'

LIGNE = re.compile(
    r'^(\d\d):(\d\d):([\d.]+) P:(-?[\d.]+) tgt:(-?[\d.]+) e:(-?[\d.]+) '
    r'g:(-?[\d.]+) (PID|AGENT) (DUAL|ANG|OFF) sps:(-?\d+) c:(-?[\d.]+) '
    r'off:(-?[\d.]+) pos:(-?\d+) hz:(\d+) f:(\d+) Lus:(\d+) Ius:(\d+)'
    r'(?: Pus:(\d+))?')


class Pupitre:
    def __init__(self, racine):
        self.r = racine
        self.file = []            # commandes en attente
        self.hist = []            # erreur d angle, pour la bande
        self.derniere = None
        self.taille_log = 0
        self._construire()
        self._boucle()
        self._pousser()

    # ------------------------------------------------------------- commandes
    def envoyer(self, txt):
        """On EMPILE au lieu d ecrire tout de suite. Le moniteur lit cmd.txt
        puis le supprime ; ecrire pendant qu il lit perdrait la commande. On
        n ecrit donc que quand le fichier a disparu, c est-a-dire quand la
        precedente a bien ete consommee."""
        # Une consigne ECRASE la precedente du meme type au lieu de s ajouter :
        # le robot n a que faire des positions intermediaires du joystick, seule
        # la derniere compte. Sans ca la file gonfle et les BOUTONS DEVIENNENT
        # SOURDS -- Matthieu l a constate le 10 septembre, STOP attendait
        # derriere des dizaines de commandes de joystick.
        tete = txt[0]
        if tete in 'N#':
            self.file = [c for c in self.file if c[0] != tete]
        self.file.append(txt)
        del self.file[:-4]

    def urgent(self, txt):
        """Pour STOP : on jette la file et on ecrit tout de suite, quitte a
        ecraser une commande en cours de lecture. Un arret qui attend son tour
        n est pas un arret."""
        self.file = []
        for _ in range(3):
            try:
                with open(CMD, 'w', encoding='ascii') as f:
                    f.write(txt + chr(10))
                return
            except OSError:
                time.sleep(0.01)

    def _vider_file(self):
        if self.file and not os.path.exists(CMD):
            try:
                with open(CMD, 'w', encoding='ascii') as f:
                    f.write(self.file[0] + '\n')
                self.file.pop(0)
            except OSError:
                pass

    # --------------------------------------------------------- joystick
    def _pad_bouge(self, e):
        R = 100.0
        self.jx = max(-1.0, min(1.0, (e.x - 110) / R))
        self.jy = max(-1.0, min(1.0, (110 - e.y) / R))
        self._dessiner_pad()
        self.consigne()

    def _pad_lache(self, _e):
        # Deux comportements, au choix, parce qu il n y a pas de bon defaut.
        #
        #   RESSORT  : tout retombe a zero, comme une manette. Naturel pour
        #              piloter, mais il faut garder le doigt appuye.
        #   MAINTIEN : tout reste ou on l a laisse. Pratique pour tenir une
        #              vitesse, mais un virage oublie fait tourner en rond.
        #
        # Par defaut RESSORT : c est le moins surprenant, et le plus sur.
        if self.ressort:
            self.jx = self.jy = 0.0
        else:
            self.jx = 0.0        # le lacet retombe toujours
        self._dessiner_pad()
        self.consigne()

    def _touche(self, e):
        pas = 0.15
        if e.keysym == 'Up':      self.jy = min(1.0, self.jy + pas)
        elif e.keysym == 'Down':  self.jy = max(-1.0, self.jy - pas)
        elif e.keysym == 'Left':  self.jx = max(-1.0, self.jx - pas)
        elif e.keysym == 'Right': self.jx = min(1.0, self.jx + pas)
        elif e.keysym == 'space': self.jx = self.jy = 0.0
        self._dessiner_pad()
        self.consigne()

    def _mode_relache(self, ressort):
        self.ressort = ressort
        for nom, btn in self.btn_mode.items():
            actif = (nom == ('ressort' if ressort else 'maintien'))
            btn.config(bg=OCRE if actif else '#2a2f3a',
                       fg='#14161a' if actif else TEXTE)

    def _echelle(self, e):
        self.echelle = e
        for k, b in self.btn_ech.items():
            actif = (k == e)
            b.config(bg=ACCENT if actif else '#2a2f3a',
                     fg='#14161a' if actif else TEXTE)
        self.consigne()

    def _dessiner_pad(self):
        p = self.pad
        p.delete('all')
        p.create_oval(10, 10, 210, 210, outline='#2a2f3a')
        p.create_line(110, 10, 110, 210, fill='#1e222a')
        p.create_line(10, 110, 210, 110, fill='#1e222a')
        p.create_text(110, 22, text='avant', fill=GRIS, font=('Consolas', 8))
        p.create_text(30, 110, text='gauche', fill=GRIS, font=('Consolas', 8))
        x = 110 + self.jx * 100
        y = 110 - self.jy * 100
        p.create_line(110, 110, x, y, fill='#2a2f3a', width=2)
        p.create_oval(x - 11, y - 11, x + 11, y + 11, fill=ACCENT, outline='')

    def consigne(self, _=None):
        # On arrondit : sans ca le moindre pixel de deplacement emet une
        # commande, et la liaison serie ne suit pas.
        av = int(round(self.jy * SPS_MAX * self.echelle / 25.0)) * 25
        la = int(round(self.jx * LACET_MAX * self.echelle / 25.0)) * 25
        if av != getattr(self, '_av', None):
            self.envoyer('N%d' % av)
            self._av = av
        if la != getattr(self, '_la', None):
            self.envoyer('#%d' % la)
            self._la = la
        self.lbl_consigne.config(
            text='avance %+5d pas/s (%+.3f m/s)   lacet %+5d   x%g'
                 % (av, av * MS_PAR_PAS, la, self.echelle))

    def recentrer(self):
        self.jx = self.jy = 0.0
        self._dessiner_pad()
        self.consigne()

    def stop(self):
        self.jx = self.jy = 0.0
        self._dessiner_pad()
        self.urgent('S')
        self._av = self._la = 0
        self.lbl_consigne.config(text='avance 0 pas/s   lacet 0')

    # ------------------------------------------------------------- interface
    def _construire(self):
        r = self.r
        r.title('Pupitre — robot balancier')
        r.configure(bg=FOND)
        r.geometry('600x850')
        gros = tkfont.Font(family='Consolas', size=26, weight='bold')
        moyen = tkfont.Font(family='Consolas', size=13)
        petit = tkfont.Font(family='Consolas', size=10)

        def carte(parent, titre):
            c = tk.Frame(parent, bg=CARTE, padx=14, pady=10)
            c.pack(fill='x', padx=14, pady=6)
            tk.Label(c, text=titre, bg=CARTE, fg=GRIS,
                     font=petit, anchor='w').pack(fill='x')
            return c

        # ---- etat
        c = carte(r, 'ETAT')
        self.lbl_mode = tk.Label(c, text='—', bg=CARTE, fg=TEXTE, font=gros)
        self.lbl_mode.pack(anchor='w')
        self.lbl_detail = tk.Label(c, text='', bg=CARTE, fg=GRIS,
                                   font=moyen, justify='left', anchor='w')
        self.lbl_detail.pack(fill='x')

        # ---- bande de l erreur d angle
        c = carte(r, "ERREUR D ANGLE  (±3°, 15 dernieres secondes)")
        self.bande = tk.Canvas(c, height=90, bg='#0e1013',
                               highlightthickness=0)
        self.bande.pack(fill='x', pady=(4, 0))

        # ---- joystick
        c = carte(r, 'JOYSTICK   (glisser, ou les fleches du clavier)')
        self.lbl_consigne = tk.Label(c, text='avance 0 pas/s   lacet 0',
                                     bg=CARTE, fg=ACCENT, font=moyen)
        self.lbl_consigne.pack(anchor='w')
        self.pad = tk.Canvas(c, width=220, height=220, bg='#0e1013',
                             highlightthickness=0)
        self.pad.pack(pady=6)
        self.pad.bind('<B1-Motion>', self._pad_bouge)
        self.pad.bind('<Button-1>', self._pad_bouge)
        self.pad.bind('<ButtonRelease-1>', self._pad_lache)
        self.jx = 0.0        # -1..1, lacet
        self.jy = 0.0        # -1..1, avance
        self.echelle = 1.0
        self.ressort = True
        self._dessiner_pad()
        b = tk.Frame(c, bg=CARTE)
        b.pack(fill='x')
        tk.Button(b, text='recentrer', command=self.recentrer,
                  bg='#2a2f3a', fg=TEXTE, font=petit, relief='flat',
                  padx=12, pady=4).pack(side='left')
        tk.Label(b, text='   echelle ', bg=CARTE, fg=GRIS,
                 font=petit).pack(side='left')
        self.btn_ech = {}
        for e in ECHELLES:
            btn = tk.Button(b, text='x%g' % e, font=petit, relief='flat',
                            padx=10, pady=4,
                            command=lambda x=e: self._echelle(x))
            btn.pack(side='left', padx=2)
            self.btn_ech[e] = btn
        self._echelle(1.0)
        b2 = tk.Frame(c, bg=CARTE)
        b2.pack(fill='x', pady=(4, 0))
        tk.Label(b2, text='au relachement ', bg=CARTE, fg=GRIS,
                 font=petit).pack(side='left')
        self.btn_mode = {}
        for nom, val in (('ressort', True), ('maintien', False)):
            btn = tk.Button(b2, text=nom, font=petit, relief='flat',
                            padx=12, pady=4,
                            command=lambda v=val: self._mode_relache(v))
            btn.pack(side='left', padx=2)
            self.btn_mode[nom] = btn
        self._mode_relache(True)
        for k in ('<Up>', '<Down>', '<Left>', '<Right>', '<space>'):
            r.bind(k, self._touche)
        r.bind('<Escape>', lambda _e: self.stop())
        r.focus_set()

        # ---- pilote
        c = carte(r, 'PILOTE')
        b = tk.Frame(c, bg=CARTE)
        b.pack(fill='x', pady=4)
        # Les deux boutons de pilote s allument d apres la TELEMETRIE, pas
        # d apres le dernier clic : ce qui compte est ce que le robot fait,
        # pas ce qu on lui a demande. Un ordre perdu se verrait immediatement.
        self.btn_pilote = {}
        for txt, cmd, coul in (('cascade', '1', OCRE), ('agent', '2', ACCENT)):
            btn = tk.Button(b, text=txt,
                            command=lambda x=cmd: self.envoyer(x),
                            bg='#2a2f3a', fg=coul, font=moyen, relief='flat',
                            padx=22, pady=8)
            btn.pack(side='left', padx=(0, 8))
            self.btn_pilote[txt] = (btn, coul)
        self.btn_armer = tk.Button(b, text='ARMER',
                                   command=lambda: self.envoyer('K'),
                                   bg='#2a2f3a', fg=VERT, font=moyen,
                                   relief='flat', padx=22, pady=8)
        self.btn_armer.pack(side='left')
        # Calage de la verticale. Indispensable AVANT de passer a l agent :
        # l auto-trim vit dans la boucle externe, et la boucle externe ne
        # tourne pas en mode agent -- volontairement, pour que la reference
        # ne bouge pas sous ses pieds. Personne ne corrige donc l offset la.
        tk.Button(b, text='calage', command=lambda: self.envoyer('Z'),
                  bg='#2a2f3a', fg=GRIS, font=moyen, relief='flat',
                  padx=16, pady=8).pack(side='left', padx=(8, 0))
        tk.Button(r, text='STOP', command=self.stop, bg=ROUGE, fg='#14161a',
                  font=gros, relief='flat', pady=10).pack(fill='x', padx=14,
                                                          pady=(8, 14))

    # ------------------------------------------------------------- lecture
    def _derniere_ligne(self):
        """On relit uniquement la fin du fichier. Il fait plusieurs mega-octets
        et grossit en continu : le relire en entier dix fois par seconde
        mettrait la machine a genoux."""
        try:
            taille = os.path.getsize(LOG)
            with open(LOG, 'rb') as f:
                f.seek(max(0, taille - 4000))
                bloc = f.read().replace(b'\x00', b'')
        except OSError:
            return None
        for ligne in reversed(bloc.decode('utf-8', 'replace').splitlines()):
            m = LIGNE.match(ligne)
            if m:
                return m
        return None

    def _boucle(self):
        self._vider_file()
        m = self._derniere_ligne()
        if m:
            mode = m.group(8)
            arme = m.group(9)
            err = float(m.group(6))
            sps = int(m.group(10))
            pos = int(m.group(13))
            f = int(m.group(15))
            lus = int(m.group(16))
            pus = m.group(18)

            coul = VERT if arme != 'OFF' else GRIS

            # ---- qui pilote vraiment, et est-il arme
            actif = 'agent' if mode == 'AGENT' else 'cascade'
            for nom, (btn, c0) in self.btn_pilote.items():
                if nom == actif:
                    btn.config(bg=c0, fg='#14161a',
                               text=nom.upper() + '  <')
                else:
                    btn.config(bg='#2a2f3a', fg=c0, text=nom)
            if arme != 'OFF':
                self.btn_armer.config(text='ARME', bg=VERT, fg='#14161a')
            else:
                self.btn_armer.config(text='ARMER', bg='#2a2f3a', fg=VERT)

            self.lbl_mode.config(
                text='%s  %s' % ('AGENT' if mode == 'AGENT' else 'CASCADE',
                                 'ARME' if arme != 'OFF' else 'coupe'),
                fg=ACCENT if mode == 'AGENT' else OCRE)
            # Le % se lie plus fort que le + : concatener les morceaux
            # PUIS formater, sinon seul le dernier recoit les arguments.
            fmt = chr(10).join((
                'erreur %+6.2f deg      vitesse %+5d pas/s',
                'position %+6.0f mm     boucle %4d us',
                'verticale %+7.2f deg   correction %+5.2f',
                'echecs I2C %-6d     politique %s us'))
            self.lbl_detail.config(text=fmt % (
                err, sps, pos * MS_PAR_PAS * 1000, lus,
                float(m.group(12)), float(m.group(11)), f,
                pus if pus else '—'))
            self.hist.append(err)
            if len(self.hist) > 150:      # 15 s a 10 Hz
                self.hist.pop(0)
            self._tracer(coul)
        self.r.after(100, self._boucle)

    def _pousser(self):
        # La file est poussee 5 fois plus vite que l affichage : une commande ne
        # doit pas attendre le prochain rafraichissement pour partir.
        self._vider_file()
        self.r.after(20, self._pousser)

    def _tracer(self, coul):
        c = self.bande
        c.delete('all')
        L = c.winfo_width() or 480
        H = 90
        c.create_line(0, H / 2, L, H / 2, fill='#2a2f3a')
        for d in (-2, -1, 1, 2):
            y = H / 2 - d * (H / 2) / 3.0
            c.create_line(0, y, L, y, fill='#1e222a')
        if len(self.hist) < 2:
            return
        pts = []
        for i, v in enumerate(self.hist):
            x = L * i / max(1, len(self.hist) - 1)
            y = H / 2 - max(-3.0, min(3.0, v)) * (H / 2) / 3.0
            pts += [x, y]
        c.create_line(*pts, fill=coul, width=2, smooth=True)


if __name__ == '__main__':
    racine = tk.Tk()
    Pupitre(racine)
    racine.mainloop()
