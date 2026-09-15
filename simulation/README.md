# Simulation et apprentissage

Le robot reconstruit dans MuJoCo depuis son assemblage SolidWorks, la cascade
du firmware portée telle quelle comme témoin, et un agent PPO entraîné pour la
battre puis exporté en C vers l'ESP32.

**Tous les scripts se lancent depuis ce dossier.**

```
pip install -r requirements.txt
pip install torch --index-url https://download.pytorch.org/whl/cpu
```

---

## Le principe

Un agent entraîné sur un modèle faux apprend à équilibrer un robot qui
n'existe pas, et on ne s'en aperçoit qu'au transfert. Le modèle est donc validé
**avant** d'entraîner, et chaque valeur porte sa provenance :

```
python modele.py --etat
```

`MESURE` (sur le robot, une fiche technique ou l'assemblage), `DEDUIT`
(calculé depuis une mesure), `ESTIME`, `SUPPOSE`. Il reste 2 paramètres non
établis sur 24.

La géométrie n'est pas saisie : les 6 STL de `hardware/cao/stl_assemblage/`
portent la masse, et MuJoCo calcule les inerties sur la vraie forme.

## Le moteur, la pièce qui a tout changé

Un moteur pas-à-pas n'est pas une source de vitesse. C'est un **ressort
magnétique** entre la position commandée et la position du rotor :

```
C = C_maintien · sin(50 · (θ_commande − θ_rotor))
```

Ce ressort résonne. La résonance a été **mesurée à 71 Hz** sur le robot, pas les
113 Hz supposés au départ, par des balayages fréquentiels injectés en boucle
fermée et des interspectres, sans démonter le robot. Avec l'actionneur
`<velocity>` de MuJoCo, l'ancien agent paraissait parfait. Sur le robot, il
tremblait dix fois plus. Avec `moteur.py`, la simulation reproduit ce
tremblement à 1 % près sur la commande, sans avoir été recalée dessus.

---

## Les scripts, dans l'ordre du parcours

| | |
|---|---|
| `modele.py` | génère le MJCF, tient la provenance de chaque chiffre |
| `moteur.py` | le pas-à-pas fidèle : ressort magnétique, décrochage |
| `etat.py` | lit l'état **par les capteurs simulés**, jamais par `qpos` |
| `firmware.py` | la cascade du `.ino` transcrite en Python |
| `balancier_env.py` | l'environnement Gymnasium : 15 observations, 2 actions, curriculum |
| `01_regarder.py` | le viewer (`--pousser`, `--mesh`) |
| `02_identifier.py` | le mode propre du modèle, confronté au carnet |
| `03_pid.py` | la cascade du firmware en simulation (`--vue`, `--silence`) |
| `04_pousser.py` | la poussée maximale encaissée (`--balayage`) |
| `05_bac_a_sable.py` | pousser le robot à la souris |
| `06_env.py` | vérification de l'environnement (`--cascade`) |
| `07_ppo.py` | l'entraînement |
| `08_agent.py` | regarder un agent (`--duel` contre la cascade, `--film`) |
| `09_piloter.py` | piloter en simulation avec un pupitre |
| `10_limites.py` | jusqu'où chacun tient |
| `11_export_c.py` | exporte la politique en C, **directement dans** `firmware/robot_balancier/politique.h` |
| `12_valider_c.py` | compile l'en-tête avec gcc et le compare à PyTorch |
| `15_moteur_identique.py` | vérifie que la copie rapide du moteur dans `balancier_env.py` donne exactement le couple de `moteur.py` |
| `17_pousser_agents.py` | la poussée maximale de chaque pilote, sur le moteur fidèle |
| `incertitude.py` `moteur_duel.py` `bench_rl.py` | propagation d'incertitude, vitesse contre couple, débit PPO |
| `film.py` `rendu.py` | les vidéos de simulation |
| `montage_reel.py` `montage_duo.py` `19_mosaique_verticale.py` `montage_post.py` | le montage de la vidéo publiée |

Après toute modification de `modele.py` : `python modele.py` pour régénérer
`modeles/balancier.xml`.

---

## Entraîner, exporter, téléverser

```
python 07_ppo.py --actionneur couple --pas 9000000
python 08_agent.py --duel
python 11_export_c.py --agent agents/balancier_final.zip
python 12_valider_c.py
```

L'agent final a été entraîné en 15,7 min pour 9 M de pas (Ryzen 7 7800X3D,
16 environnements, CPU seul). `12_valider_c.py` doit annoncer un écart de
l'ordre de 10⁻⁷ : c'est la différence float32 / float64, rien d'autre. **Ne
jamais téléverser un en-tête qui n'est pas passé par lui** : une matrice
transposée ne se voit pas autrement qu'en regardant le robot tomber.

### Les agents versionnés

| | |
|---|---|
| `agents/balancier_final.zip` | l'agent actuel, entraîné sur le moteur fidèle. C'est lui qui est dans `politique.h`. |
| `agents_v1_vitesse/balancier_9000000_pas.zip` | l'ancien, entraîné sur l'actionneur en vitesse. Gardé pour la comparaison (`film.py --comparer`). |

Les jalons intermédiaires, les journaux d'entraînement et les rendus ne sont pas
versionnés : ils se régénèrent.

---

## Résultats

Tous jugés sur le moteur fidèle, robot calme :

| | ancien agent | **nouvel agent** | cascade |
|---|---|---|---|
| erreur d'angle, écart-type | 1,743° | **0,106°** | 0,185° |
| commande, écart-type | 873 pas/s | **47 pas/s** | 86 pas/s |
| dérive de position | 55 mm | 12 mm | 13 mm |
| survie à difficulté 1,0 | 25 % | **42 %** | 33 % |

Poussée isolée de 150 ms à 21 cm de l'axe (`17_pousser_agents.py`) : cascade
0,47 N, ancien agent 0,38 N, nouvel agent 0,53 N. **Mais cet avantage ne
survit pas à un enchaînement de poussées** : à 0,55 N les deux tombent, à 0,50 N
les deux tiennent. La vidéo ne montre donc pas de différence de poussée, parce
qu'il n'y en a pas de robuste. Ce qui s'y voit est la vraie différence : le
tremblement.

Le détail de chaque chiffre est dans [`docs/journal/RESULTATS.md`](../docs/journal/RESULTATS.md),
les erreurs commises en route dans [`docs/journal/PIEGES.md`](../docs/journal/PIEGES.md).
