# -*- coding: utf-8 -*-
"""
07 - L entrainement.

    python 07_ppo.py                      1,5 M de pas, ~4 min
    python 07_ppo.py --pas 4000000        plus long
    python 07_ppo.py --env 8              moins de coeurs
    python 07_ppo.py --reprendre agents/balancier_800000_pas.zip

Tout est aveugle : aucune fenetre ne s ouvre. Le rendu coute 100 a 1000 fois
plus cher que la physique. On garde une trace autrement --

    agents/balancier_<n>_pas.zip     un point de sauvegarde tous les 200 k pas
    rendus/agent_<n>.gif             un episode filme hors-ecran, meme cadence
    agents/journal.csv               difficulte, survie, recompense

et 08_agent.py ouvre n importe lequel de ces points en direct.

--------------------------------------------------------------- LE CURRICULUM
La difficulte monte AU MERITE, pas au chronometre : on ne la releve que
lorsque le taux de survie depasse 80 % sur les 50 derniers episodes. Un
curriculum indexe sur le nombre de pas avance quand l agent n est pas pret et
detruit ce qu il avait acquis.

------------------------------------------------------- PAS DE NORMALISATION
On n utilise PAS VecNormalize, volontairement. L observation est deja mise a
l echelle a la main dans l environnement, par des grandeurs physiques
connues (10 deg, 100 deg/s, 1600 pas/s...). Consequence : la politique est
autonome. Pour la porter sur l ESP32 il n y a que des poids a copier, aucune
statistique de normalisation a transporter ni a tenir a jour.
"""
import argparse
import csv
import os
import time
import numpy as np
import torch

# Le reseau fait 1 500 poids : la synchronisation entre threads coute plus cher
# que le calcul. Et pendant l entrainement, 16 workers MuJoCo se disputent deja
# les 16 threads. Mesure sur cette machine : 2 threads = -18 % de temps par
# rapport aux 8 par defaut.
torch.set_num_threads(2)

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor

from balancier_env import Balancier, HZ_POLITIQUE, DUREE_EPISODE

DOSSIER = 'agents'
PAS_MAX_EPISODE = int(DUREE_EPISODE * HZ_POLITIQUE)


# Le mode d actionneur est global au module : sous Windows SubprocVecEnv
# relance l interpreteur, donc fabriquer() doit pouvoir le retrouver sans qu on
# le lui passe -- une fermeture ne se serialise pas. D ou la variable
# d environnement, lue a l import dans chaque processus fils.
ACTIONNEUR = os.environ.get('BALANCIER_ACTIONNEUR', 'vitesse')


def fabriquer():
    return Monitor(Balancier(actionneur=ACTIONNEUR))


class Curriculum(BaseCallback):
    """Monte la difficulte quand l agent la merite, et journalise."""

    def __init__(self, seuil=0.80, fenetre=50, marche=0.05, verbose=0):
        super().__init__(verbose)
        # Le tampon est VIDE apres chaque montee. Sans ca, la fenetre glissante
        # reste au-dessus du seuil et la difficulte monte a chaque pas : elle
        # passait de 0 a 1,0 en 300 ms, et l agent ne voyait jamais les niveaux
        # intermediaires. Il faut re-meriter chaque palier sur des episodes NEUFS.
        self.seuil, self.fenetre, self.marche = seuil, fenetre, marche
        self.difficulte = 0.0
        self.recents = []
        self.journal = []
        self.t0 = time.time()

    def _on_step(self):
        for info, fini in zip(self.locals['infos'], self.locals['dones']):
            if not fini:
                continue
            # SB3 marque ainsi un episode arrete par le temps, donc SURVECU.
            survecu = bool(info.get('TimeLimit.truncated', False))
            self.recents.append(1.0 if survecu else 0.0)
        if len(self.recents) >= self.fenetre:
            taux = float(np.mean(self.recents[-self.fenetre:]))
            self.journal.append((self.num_timesteps, self.difficulte, taux))
            if taux > self.seuil and self.difficulte < 1.0:
                self.difficulte = min(1.0, self.difficulte + self.marche)
                self.training_env.env_method('regler_difficulte', self.difficulte)
                print('  [%7d pas] survie %3.0f %%  ->  difficulte %.2f   (%.0f s)'
                      % (self.num_timesteps, 100 * taux, self.difficulte,
                         time.time() - self.t0))
                self.recents = []                    # <- on repart a zero
            else:
                self.recents = self.recents[-self.fenetre:]
        return True


class Trace(BaseCallback):
    """Sauvegarde et filme, a intervalle regulier."""

    # Jalons en progression GEOMETRIQUE, pas uniforme. Entre 0 et 200 k le
    # comportement change du tout au tout ; entre 5 et 6 M il ne bouge plus.
    # Un pas fixe de 200 k rate donc tout l interessant et filme vingt fois
    # la meme chose.
    # Places d apres la courbe du run precedent : le decollage se produit
    # vers 1,5-1,8 M et tout se joue entre la et 6 M. Un espacement
    # geometrique uniforme gaspillait huit jalons sur douze avant le
    # decollage -- ils tombaient tous a la premiere seconde et racontaient
    # tous la meme chose. On garde trois temoins tot, et on resserre la ou
    # le comportement change.
    JALONS = [25_000, 300_000, 900_000,
              1_300_000, 1_500_000, 1_700_000, 1_900_000, 2_100_000,
              2_400_000, 2_800_000, 3_300_000, 3_900_000,
              4_600_000, 5_500_000, 6_800_000, 9_000_000]

    def __init__(self, tous_les=None, curriculum=None, verbose=0):
        super().__init__(verbose)
        self.restants = list(self.JALONS)
        self.curriculum = curriculum

    def _on_step(self):
        if not self.restants or self.num_timesteps < self.restants[0]:
            return True
        self.restants.pop(0)
        chemin = os.path.join(DOSSIER, 'balancier_%d_pas' % self.num_timesteps)
        self.model.save(chemin)
        print('  [%7d pas] sauvegarde  %s.zip' % (self.num_timesteps, chemin))
        try:
            filmer(self.model, self.num_timesteps,
                   self.curriculum.difficulte if self.curriculum else 0.0)
        except Exception as e:
            print('       (video impossible : %s)' % e)
        return True


def filmer(modele_ppo, n_pas, difficulte, duree=8.0, fps=25):
    """Un episode rendu hors-ecran. Cout : quelques secondes, tous les 200 k pas."""
    import mujoco
    from PIL import Image
    import rendu as R

    env = Balancier(difficulte=difficulte, actionneur=ACTIONNEUR)
    o, _ = env.reset()
    r = mujoco.Renderer(env.m, height=560, width=760)
    b = mujoco.mj_name2id(env.m, mujoco.mjtObj.mjOBJ_BODY, 'chassis')
    images, prochaine = [], 0.0
    while env.d.time < duree:
        a, _ = modele_ppo.predict(o, deterministic=True)
        o, _, tombe, fini, info = env.step(a)
        if env.d.time >= prochaine:
            img = R.rendre(r, env.d, 'profil', suivre=env.d.xpos[b])
            images.append(R.bandeau(
                img, 't %5.2f s   tangage %+6.2f deg   v %+5.3f / %+5.3f m/s'
                '   cap %+5.1f deg   difficulte %.2f'
                % (env.d.time, info['tangage'], info['v'], env.consigne[0],
                   info['cap'], difficulte)))
            prochaine += 1.0 / fps
        if tombe or fini:
            break
    os.makedirs('rendus', exist_ok=True)
    chemin = os.path.join('rendus', 'agent_%d.gif' % n_pas)
    Image.fromarray(images[0]).save(
        chemin, save_all=True,
        append_images=[Image.fromarray(i) for i in images[1:]],
        duration=int(1000 / fps), loop=0, optimize=True)
    print('       video       %s   (%s a t=%.1f s)'
          % (chemin, 'CHUTE' if tombe else 'tenu', env.d.time))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--pas', type=int, default=1_500_000)
    ap.add_argument('--env', type=int, default=16)
    ap.add_argument('--reprendre', type=str, default=None)
    ap.add_argument('--graine', type=int, default=0)
    ap.add_argument('--actionneur', choices=('vitesse', 'couple'), default='vitesse',
                    help="'couple' branche moteur.py : ressort magnetique, "
                         "decrochage, resonance recalee sur les 71 Hz mesures. "
                         "4,5 fois plus lent, mais c est le seul qui reproduise "
                         "le vrai moteur.")
    a = ap.parse_args()
    os.environ['BALANCIER_ACTIONNEUR'] = a.actionneur
    ACTIONNEUR = a.actionneur

    os.makedirs(DOSSIER, exist_ok=True)
    cls = DummyVecEnv if a.env == 1 else SubprocVecEnv
    venv = cls([fabriquer for _ in range(a.env)])

    if a.reprendre:
        mdl = PPO.load(a.reprendre, env=venv, device='cpu')
        print('  reprise depuis %s' % a.reprendre)
    else:
        mdl = PPO(
            'MlpPolicy', venv,
            # --- 2 couches de 32 : ~1 500 poids, quelques microsecondes sur
            #     un ESP32 a 240 MHz. La contrainte est posee des le depart,
            #     sinon on obtient un agent indeployable.
            policy_kwargs=dict(
                net_arch=dict(pi=[32, 32], vf=[64, 64]),
                # log_std_init = 0 par defaut, soit un ecart-type de 1,0 sur une
                # action qui vaut +-25 000 pas/s2 : l exploration etait un bruit
                # blanc pleine echelle, impossible d equilibrer par hasard. D ou
                # 1,8 M de pas de plateau avant le decollage. exp(-1,5) = 0,22.
                log_std_init=-1.5),
            # --- a 100 Hz, gamma = 0,99 regarde 100 pas = 1 s devant. C est
            #     l ordre de grandeur du retour a l equilibre du balancier.
            gamma=0.99,
            gae_lambda=0.95,
            n_steps=512,               # 512 x 16 = 8192 transitions par mise a jour
            batch_size=2048,
            n_epochs=10,
            learning_rate=3e-4,
            clip_range=0.2,            # le bridage : jamais plus de 20 % d un coup
            ent_coef=0.001,            # un peu d exploration, sinon il se fige tot
            vf_coef=0.5,
            max_grad_norm=0.5,
            device='cpu',              # un MLP de 1500 poids est plus lent sur GPU
            seed=a.graine,
            verbose=1,
        )

    cur = Curriculum()
    print('  %d environnements, %s pas, reseau 2x32' % (a.env, f'{a.pas:,}'))
    print('  la difficulte monte quand la survie depasse 80 %')
    print()
    t0 = time.time()
    mdl.learn(total_timesteps=a.pas, callback=[cur, Trace(curriculum=cur)],
              progress_bar=False)
    duree = time.time() - t0

    mdl.save(os.path.join(DOSSIER, 'balancier_final'))
    with open(os.path.join(DOSSIER, 'journal.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['pas', 'difficulte', 'taux_survie'])
        w.writerows(cur.journal)

    print()
    print('  termine en %.1f min   -   %.0f pas/s' % (duree / 60, a.pas / duree))
    print('  difficulte atteinte : %.2f' % cur.difficulte)
    print('  agent final : %s' % os.path.join(DOSSIER, 'balancier_final.zip'))
    print()
    print('  pour le regarder :  python 08_agent.py')
    venv.close()
