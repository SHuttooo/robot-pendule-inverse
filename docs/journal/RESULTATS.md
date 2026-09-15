# Tous les chiffres

Chaque nombre porte la commande qui le produit. Aucun n'est estimé de tête.
Établis le **5 septembre 2026**, sur l'agent `balancier_9000000_pas.zip`.

---

## Le modèle

`python 02_identifier.py`

| | |
|---|---|
| masse chassis | 0,928 kg |
| masse totale | 0,988 kg |
| hauteur du CdM / essieu | 74,1 mm |
| inertie au CdM (axe y) | 0,00716 kg·m² |
| inertie à l'essieu | 0,01225 kg·m² |
| **prédiction du pendule suspendu** | **T = 0,83 s, soit 16,6 s pour 20 oscillations** |
| divergence, roues libres | ≈ 10 rad/s |

`python incertitude.py` — fourchette à 90 % compte tenu de toutes les
estimations : **16,2 à 17,1 s**. Les trois paramètres qui dominent :
`masse_bat_haut` (4,9 %), `h_bat_haut` (4,0 %), `masse_elec` (2,4 %).

## La géométrie, lue dans l'assemblage

| | |
|---|---|
| axe des roues, repère assemblage | (74,10 ; 39,68 ; 1021,00) mm |
| entraxe des arbres moteur | 84,5 mm |
| demi-voie retenue | 69,1 mm — **déduite, pas mesurée** |
| emprise | 143,7 × 75,7 × 220 mm |
| hauteur hors-tout depuis le sol | 231,5 mm |
| étagères (surfaces horizontales) | +66, +141, +199 mm de l'essieu |
| volumes imprimés | p1 92,8 · p2 65,2 · p3 118,1 · p4 103,5 cm³ |
| PLA total estimé | 201 g |

## La cascade dans la simulation

`python 03_pid.py --silence`

| | simulation | robot réel (carnet) |
|---|---|---|
| erreur d'angle, écart-type | 0,043° | 0,064° |
| commande, écart-type | 39 pas/s | 46 pas/s |
| bruit gyro mécanique | 0,11 °/s (modèle vitesse) | 1,08 à 3,48 °/s |

Avec le modèle **fidèle** du pas-à-pas (`moteur_duel.py`) : 5,45 °/s de vibration
mécanique, du bon ordre que le réel — le modèle en vitesse n'en produit aucune.

## Le duel agent contre cascade

`python 08_agent.py --duel`

| | AGENT | CASCADE |
|---|---|---|
| **difficulté 0,0** | | |
| erreur d'angle | 0,082° | 0,105° |
| commande, écart-type | 32 | 46 pas/s |
| rugosité (RMS d²a) | 0,0650 | 0,0357 |
| **difficulté 0,6** | | |
| épisodes survécus | **92 %** | 58 % |
| erreur d'angle | 1,086° | 1,959° |
| commande, écart-type | 348 | 466 pas/s |
| rugosité | **0,0623** | 0,1448 |
| **difficulté 1,0** | | |
| épisodes survécus | **42 %** | 25 % |
| erreur d'angle | 2,621° | 3,662° |

**Au repos la cascade est plus douce** (0,036 contre 0,065). L'agent gagne quand
ça se corse : sa rugosité reste plate là où celle de la cascade double.

## Les limites de poussée

`python 10_limites.py` — poussée de 150 ms, dichotomie à 0,05 N, une force n'est
retenue que si elle tient sur 4 à 5 tirages de bruit.

| point d'application | AGENT | CASCADE | écart |
|---|---|---|---|
| sommet 210 mm | 0,70 N | 0,56 N | +25 % |
| mi-hauteur 120 mm | 1,12 N | 0,94 N | +20 % |
| essieu 0 mm | 5,30 N | 4,45 N | +19 % |

Asymétrie avant/arrière : **−6 % en arrière**, pour les deux pilotes. C'est le
modèle, pas l'agent — le CdM est décalé de 1,1 mm vers l'arrière.

**Le bras de levier domine tout** : ×7,6 entre le sommet et l'essieu.

Et surtout — mesuré sur la cascade dans `04_pousser.py` :

```
MAX_SPS 1600  →  0,204 m/s  →  0,80 N
MAX_SPS 2400  →  0,306 m/s  →  1,19 N     +49 %
MAX_SPS 2667  →  0,340 m/s  →  1,33 N     +66 %
```

**Réparer la carte vaut le double de ce que gagne le meilleur contrôleur.**

## L'entraînement

`python 07_ppo.py --pas 9000000 --env 16`

| | |
|---|---|
| durée | 16,8 min · 8 903 pas/s |
| difficulté atteinte | 1,00, à 5,1 M de pas |
| décollage | vers 1,5–1,7 M de pas |
| `ep_len_mean` final | 780 à 900 sur 1 000 |
| `clip_fraction` moyen | 3,8 % |
| `explained_variance` final | 0,950 |
| σ d'exploration | 0,223 → **0,040**, divisé par 5,5 tout seul |

9 M pas de politique = **25 h de robot simulé**, 180 M pas de physique,
accélération ×71.

## La politique

| | |
|---|---|
| acteur | 15 → 32 → 32 → 2, activation `tanh` |
| poids | **1 634** (6 885 avec le critique, jeté) |
| taille en float32 | 6,5 ko |
| coût sur ESP32 240 MHz | 1 568 MAC ≈ **13 µs**, soit 0,3 % du budget à 200 Hz |
| part linéaire | R² = 0,91 hors échantillon |

Test de capacité par régression (découpage mélangé) : `1 × 16` atteint déjà
R² = 0,997, `4 × 16` fait aussi bien que `2 × 32` pour 34 % de calcul en moins.
**Le réseau choisi est probablement surdimensionné.**

## La machine

| | |
|---|---|
| physique, un cœur, 3D | 131 000 pas/s |
| PPO, 16 environnements | 11 990 pas de politique/s → 7,2 M en 10 min |
| rendement 16 env / 1 env | ×5,6 seulement |
| `torch.set_num_threads(2)` | −18 % de temps contre les 8 par défaut |
| CPU réellement consommé | 36 % du total, soit 5,8 cœurs sur 16 |
| mémoire | 270 Mo par worker, 4,6 Go au total |

---

# Identification sur le robot réel — 9 septembre 2026, soirée

Tout ce qui suit vient de l'enregistreur embarqué à 200 Hz, sans jamais
manipuler le robot ni le faire tomber. Déplacement maximal d'un essai : 48 mm.

## Le bruit des capteurs, enfin séparé de la vibration

| grandeur | condition | mesure |
|---|---|---|
| gyro brut | robot **immobile**, moteurs muets, 6 s à 200 Hz | **0,062 °/s** |
| gyro brut | robot debout, moteurs actifs | 1,66 à 1,95 °/s |
| gyro filtré | robot immobile, 1 008 s de télémétrie | 0,145 °/s |
| accéléromètre | robot debout, moteurs actifs | **0,205 m/s²** |
| module accéléromètre | robot debout | 1,0086 g, soit 0,9 % d'écart |

**`bruit_gyro` du modèle vaut 1,088 °/s et porte l'étiquette MESURE. C'est faux
d'un facteur 17.** Les 0,062 °/s mesurés correspondent exactement à la fiche
technique du MPU-6050 sur 100 Hz de bande. Ce que mesurait le carnet n'était pas
le capteur mais la **vibration mécanique des pas-à-pas**.

La distinction n'est pas cosmétique : le bruit capteur est indépendant de tout,
la vibration est corrélée à l'activité moteur. L'agent a été entraîné avec du
bruit blanc là où la réalité lui donne une vibration structurée. Et sa
sensibilité au gyro vaut 700 pas/s² par °/s près de la verticale.

Le bruit n'est pas blanc non plus : autocorrélation 0,672 au premier retard,
énergie répartie sur toute la bande — c'est le filtre interne du MPU.

`bruit_accel` valait 0,150 m/s² en SUPPOSE ; mesuré à 0,205, soit 1,37 fois
plus. Le chiffre inclut la vibration, donc il majore le bruit propre.

## La dynamique du corps

Par balayage en fréquence, interspectres avec le signal injecté (seul signal
exogène en boucle fermée), chaque point pondéré par sa cohérence.

| | modèle (hors roues) | mesuré |
|---|---|---|
| `M·h/I` | 5,140 | **4,628 ± 0,146** |
| constante de temps | 0,1408 s | **0,1484 s** |

Vérification de structure **sans aucune masse** : le rapport entre le terme de
gravité et le gain d'actionnement doit valoir exactement g.

| hypothèse | rapport mesuré | écart à 9,81 |
|---|---|---|
| **1600 pas/tour** | 8,86 | **10 %** |
| 3200 pas/tour | 4,43 | 55 % |

**`pas_par_tour` vaut 1600.** Le paramètre DOUTEUX est résolu.

Une seule mesure ne sépare pas l'inertie de la hauteur du centre de masse. Deux
jeux collent également : inertie propre +39 %, ou centre de masse à 171 mm du
sol au lieu de 141. Il faut le pendule suspendu pour trancher.

## Le transitoire d'avance, à rejouer en simulation

Échelon de vitesse commandé, 400 pas/s pendant 2 s, enregistré à 200 Hz.

Le robot part **à reculons** — `sps` descend à −147 — avant de repartir en
avant. C'est le comportement à déphasage non minimal du balancier, et c'est
exactement le régime que la simulation doit reproduire.

Déplacement net : **37 mm** seulement. La boucle externe n'atteint que 30 % de
la vitesse commandée, à cause de `MAX_LEAD` et de la rampe de 0,4 s.

Angle atteint : +1,86° au maximum, contre +0,8° au repos.

## Le moteur, mesuré pour la première fois

Rampe de vitesse roues en l'air, 0 à 4 800 pas/s en 6 s, enregistrée à 200 Hz.

**Pas de décrochage à vide jusqu'à 4 800 pas/s**, soit 3 tours/s. La vibration
*diminue* même en montant : 0,092 °/s à 805 pas/s, 0,049 °/s à 4 009. Le plafond
`MAX_SPS = 3000` de la carte n'est donc pas fixé par le décrochage à vide. La
limite qui compte est celle **en charge**, non mesurée.

**La résonance du moteur est à 71 Hz, pas 113.**

| | moteurs en rotation | moteurs à l'arrêt, bobines alimentées |
|---|---|---|
| puissance du gyro, 60–80 Hz | **60,0 %** | 6,3 % |
| pic | 71 Hz | aucun |

Le pic n'existe que quand les moteurs tournent, et il reste **fixe** pendant que
la vitesse varie d'un facteur 9 (405 à 3 608 pas/s). Ce n'est donc pas la
fréquence des pas mais une résonance excitée par eux — celle du rotor et de la
roue contre le ressort magnétique, exactement ce que `moteur.py` modélise.

`f = (1/2π)√(k/J)` : mesurer 71 au lieu de 113 impose `k/J = 0,395` fois la
valeur du modèle. Deux lectures, non exclusives :

| lecture | conséquence |
|---|---|
| le couple de maintien est plus faible | 0,111 N·m au lieu de 0,280 |
| les roues sont plus lourdes | `masse_roue` = 86 g au lieu de 30 |

`masse_roue` porte l'étiquette ESTIME. Une roue plastique de 65 × 26 mm avec
moyeu à 86 g est parfaitement plausible — l'estimation à 30 g était basse.
La vérité est probablement entre les deux, et une balance de cuisine
trancherait en dix secondes.

**C'est le premier chiffre mesuré sur l'actionneur**, celui dont dépend tout le
comportement que `moteur.py` doit reproduire, et qui n'a jamais servi à
entraîner.

Autre résonance, distincte : ~21 Hz quand le robot est debout au sol, ~70 Hz
quand il est tenu en main. Modes du châssis imprimé, sans lien avec le moteur.
