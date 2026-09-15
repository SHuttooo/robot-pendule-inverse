# Robot balancier — simulation MuJoCo et apprentissage

Projet d'apprentissage : MuJoCo puis RL, appliqués au **vrai robot balancier**
(ESP32, MPU-6050, 2× NEMA17 17HS3401, châssis imprimé 3D). Objectif à terme :
robot humanoïde.

Depuis le 15 septembre 2026, un seul dépôt : `hardware/` (CAO, câblage),
`firmware/` (le `.ino` et `politique.h`), `tools/` (moniteur série, pupitre),
`simulation/` (MuJoCo, PPO, export C), `docs/` (carnet, journal),
`donnees/` (journaux série archivés). Les scripts de `simulation/` se lancent
**depuis `simulation/`**.

Tout est en français, code compris. Les commentaires portent le **pourquoi**,
pas le quoi.

---

## Commencer ici

**`docs/journal/ETAT.md`** — où on en est, et la prochaine étape. À lire en
premier à chaque reprise, à mettre à jour en dernier avant de couper.

| | |
|---|---|
| `docs/journal/ETAT.md` | l'état courant, ce qui reste ouvert, les commandes utiles |
| `docs/journal/PIEGES.md` | les bugs rencontrés : symptôme, cause, coût, règle. **La partie la plus réutilisable.** |
| `docs/journal/DECISIONS.md` | chaque choix de conception, sa raison, ce qui le remettrait en cause |
| `docs/journal/RESULTATS.md` | tous les chiffres mesurés, avec la commande qui les produit |
| `docs/journal/SESSIONS/` | ce qui a été fait, par date |

---

## Règle de travail

**Mesurer, pas supposer.** Chaque valeur du modèle porte sa provenance
(`MESURE` / `DEDUIT` / `ESTIME` / `SUPPOSE`) — voir `python modele.py --etat`.
Un résultat annoncé sans chiffre à l'appui ne vaut rien ici ; un résultat
négatif clairement établi vaut beaucoup.

Corollaire : **ne jamais annoncer une correction sans l'avoir vérifiée.**
C'est arrivé deux fois (collision du châssis, activation d'un patch) — le
motif de remplacement n'avait pas matché et j'ai annoncé un succès inexistant.

---

## Le robot réel — le lien série

Un moniteur tourne en tâche de fond et écrit
**`logs/robot.log`** (à la racine du dépôt, ignoré par git), horodaté à la
milliseconde. **Le lire là, jamais le faire couler dans la conversation.**

Pour envoyer une commande à la carte : déposer la lettre dans
`tools/cmd.txt`. Le moniteur l'envoie puis vide le fichier.

```
powershell -ExecutionPolicy Bypass -File "tools\serial_monitor.ps1"   relancer
tail -40 logs/robot.log                                               relire
printf '3\n' > tools/cmd.txt                                          commander
```

Le moniteur lâche le port tout seul dès qu'`esptool` tourne : pas besoin de
l'arrêter pour téléverser. Mais il **ne survit pas toujours au téléversement** —
vérifier qu'il est vivant avant de conclure quoi que ce soit d'un log muet.

**Vider `cmd.txt` avant de flasher** : une commande oubliée s'applique au
redémarrage.

---

## Les fichiers

| | |
|---|---|
| `modele.py` | génère le MJCF depuis l'assemblage. **Provenance de chaque chiffre.** |
| `etat.py` | lit l'état **par les capteurs**, jamais par `qpos` |
| `firmware.py` | transcription du `.ino` : cascade 200 Hz + boucle externe 40 Hz |
| `moteur.py` | modèle fidèle du pas-à-pas : ressort magnétique, décrochage |
| `balancier_env.py` | l'environnement Gymnasium (15 obs, 2 actions, curriculum) |
| `01_regarder` `02_identifier` `03_pid` `04_pousser` `05_bac_a_sable` | les étapes MuJoCo |
| `06_env` `07_ppo` `08_agent` | les étapes RL |
| `11_export_c.py` `12_valider_c.py` | l'export vers l'ESP32 (écrit directement `firmware/robot_balancier/politique.h`) et sa validation |
| `rendu.py` `incertitude.py` `moteur_duel.py` `bench_rl.py` | outils |
| `docs/MESURES.md` | ce qu'il reste à mesurer sur le vrai robot |

Après toute modification de `modele.py` : **`python modele.py`** pour
régénérer `modeles/balancier.xml`. Les maillages sont lus dans
`hardware/cao/stl_assemblage/`.

---

## Conventions

- **Repère du modèle** : `+x` = avant, `+y` = axe des roues, `+z` = haut,
  origine à **l'axe des roues**. Tangage > 0 = penché en avant.
- **Repère de l'assemblage SolidWorks** (mm) : `x` = entraxe, `y` = avant-arrière,
  `z` = vertical. L'axe des roues y est à **(74,10 ; 39,68 ; 1021,00)**, localisé
  dans le maillage moteur (l'arbre de Ø5,00 mm). Rotation `Rz(−90°)` pour passer
  de l'un à l'autre.
- **La verticale vaut 0°** ici, contre −91,4° sur le robot (MPU monté couché).
- **Lire l'état par `etat.py`**, jamais par `qpos` : les scripts restent
  indépendants du modèle plan / 3D, et surtout on se rend incapable de tricher
  par information privilégiée.

---

## Faits établis, à ne pas re-découvrir

**Les roues font 65 mm**, pas 90 comme le suppose le carnet. Toutes les
conversions pas↔mm du carnet sont surévaluées d'un facteur 1,38.
1 600 pas/s ne font que **0,204 m/s** (le plafond firmware est désormais
`MAX_SPS = 3000`).

**`L = 13,6 cm` du carnet n'est pas la hauteur du centre de masse** : la formule
`g/(2πf)²` donne la longueur du pendule *ponctuel* équivalent, toujours plus
grande. Le modèle place le CdM à ~74 mm de l'essieu.

**Le 1,35 Hz du carnet ne mesure rien de mécanique** — oscillation en boucle
fermée, donc fonction des gains. La bonne mesure est en `MESURES.md` § 1 :
robot suspendu par son essieu, 20 oscillations, **prédiction 16,6 s**.
Le taux de divergence du pendule inversé vaut exactement `2π/T`.

**Poussée encaissée, sur le moteur fidèle** (150 ms à 21 cm,
`17_pousser_agents.py`) : double PID 0,47 N, nouvel agent 0,53 N, ancien agent
0,38 N. L'avantage du nouvel agent ne survit pas à un enchaînement de poussées.
Les 0,80 N du 5 septembre venaient de l'actionneur en vitesse, qui suppose la
roue instantanée : trop optimistes.

**Un pas-à-pas est un ressort magnétique**, pas une source de vitesse :
`C = C_maintien · sin(50·(θ_cmd − θ_rotor))`. Couple maximal à 1,80° d'écart
(un pas entier). Résonance **mesurée à 71 Hz** sur le robot le 9 septembre, et
non les 113 Hz supposés au départ (d'où la raideur de 14 N·m/rad, caduque). Le modèle fidèle
(`moteur.py`) génère 5,45 °/s de vibration mécanique — du bon ordre que les
1,08–3,48 °/s du carnet — là où l'actionneur `<velocity>` n'en produit aucune.
Il divise aussi par deux la poussée encaissée.

**Le bruit capteur du carnet suffit** à retrouver l'erreur réelle :
0,003° sans bruit, **0,043° avec**, contre 0,064° mesurés sur le robot.

**La politique exportée en C porte un état** : `pol_a_prec`, `pol_a_prec2` et
`pol_x_cible`. Tout appel hors boucle fermée doit être précédé de
`politique_reset()`, sinon la sortie dérive à entrée constante.

---

## Pièges rencontrés, tous coûteux

**Récompense négative = suicide.** Ma première version pénalisait l'écart de
vitesse en quadratique non borné : à 1200 pas/s la pénalité valait 2,08 contre
1,0 de prime de survie. Or un balancier *doit* lancer ses roues pour tenir.
L'agent a appris à tomber en 0,9 s. → **primes en cloche `exp(−x²)`, bornées**,
récompense toujours positive tant qu'il est debout.

**`log_std_init = 0` par défaut** = σ de 1,0 sur une action de ±25 000 pas/s²,
soit un bruit blanc pleine échelle. 1,8 M de pas de plateau avant décollage.
→ **`log_std_init = -1.5`**.

**Le curriculum montait à chaque pas** une fois le seuil franchi : la fenêtre
glissante restait au-dessus. 0 → 1,0 en 300 ms. → **vider le tampon après
chaque montée**, chaque palier se re-mérite sur des épisodes neufs.

**`'' in 'GK'` vaut `True`** en Python. Toute touche non imprimable (Ctrl, Maj,
flèche) basculait le bac à sable en mode G et coupait la boucle externe
silencieusement. → tuple, jamais chaîne.

**`mj_resetData` remet `d.time` à zéro**, donc une poussée programmée en date
absolue se ré-applique après un reset.

**`mjv_applyPerturbForce` n'est pas appelé par le viewer passif** : sans lui,
Ctrl+glisser ne produit aucune force. Et c'est un *ressort* vers une référence,
pas une impulsion.

**Sous Windows, `SubprocVecEnv` relance l'interpréteur** : la classe
d'environnement doit vivre dans un module importable (d'où `balancier_env.py`
et non `06_env.py` — un nom de module ne peut pas commencer par un chiffre).

**MuJoCo 3.x n'a plus le drapeau `sensornoise`** : le bruit s'ajoute côté Python.

**Un commentaire de bloc ne s'imbrique pas en C** : un `/* ... */` glissé dans
l'en-tête de `politique.h` le refermait trop tôt, et gcc partait en cascade
d'erreurs sans rapport.

---

## Machine

Ryzen 7 7800X3D (8C/16T, 96 Mo L3), 32 Go, **RX 7900 XTX**. Windows.

Le GPU est **inutilisable** ici : pas de CUDA sur AMD, PyTorch ROCm est Linux
uniquement. Et sans importance — un MLP de 1 500 poids est plus rapide sur CPU.
`torch.set_num_threads(2)` : −18 % par rapport aux 8 par défaut.

Débits mesurés : 131 k pas de physique/s sur un cœur (3D, actionneur vitesse) ;
**12 000 pas de politique/s** en PPO avec 16 environnements, soit 7,2 M de pas
en 10 minutes.

Le GPU deviendrait utile vers 10⁸–10⁹ pas, c'est-à-dire un humanoïde — et il
faudrait alors Ubuntu (ROCm) ou du NVIDIA.
