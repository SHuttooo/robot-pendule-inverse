# Ce qu'il reste à mesurer

> **Mise à jour.** Les masses ne sont plus pesées mais **estimées**, et
> `incertitude.py` montre que ça suffit : la fourchette à 90 % sur les
> 20 oscillations est de **16,1 à 17,2 s**, soit −4 / +3 %. Peser n'apporterait
> presque rien. La seule mesure qui reste indispensable est la **n° 1**.

**L'export d'assemblage a réglé toute la géométrie.** Les 6 STL partagent le
repère de l'assemblage, l'arbre du moteur 1 a été localisé dans le maillage
(cylindre de 5,00 mm), et les maillages portent maintenant la masse — MuJoCo
calcule leur tenseur d'inertie sur la vraie forme.

Il ne reste donc **que des masses**, plus aucune forme — et elles sont estimées
à partir du volume des STL et de l'épaisseur de paroi, pas devinées.

`python modele.py --etat` liste les sources. `python incertitude.py` dit ce que
chaque incertitude coûte réellement.

---

## 1. La période du pendule suspendu — 10 minutes

C'est **la** mesure, et elle vaut contre-épreuve de tout le reste. Elle donne
directement le chiffre que la commande doit combattre, sans balance ni calcul.

**Protocole.** Serrer l'essieu horizontalement dans un étau ou entre deux livres,
roues démontées ou bloquées, le robot **pendant vers le bas**. L'écarter de 5 à
10° et lâcher. Chronométrer **20 oscillations complètes**.

```
T = t(20 oscillations) / 20
ω = 2π / T
```

**Pourquoi ça marche.** Suspendu, le robot est un pendule composé de période
`T = 2π·√(I/(m·g·d))`. Retourné, il diverge à `ω = √(m·g·d/I)` — **le même
groupement**, à l'inverse près. Donc :

> Le taux de divergence du pendule inversé est exactement `2π/T`,
> où `T` est la période du même robot suspendu.

Pas d'algèbre, pas de masse à peser, pas de centre de gravité à trouver. On
compte des allers-retours.

Amplitude sous 10° (au-delà la période s'allonge), et 20 périodes pour que
l'erreur de chronométrage humaine (~0,2 s) tombe sous 1 %.

**Ce que le modèle prédit :** `T = 0,833 s`, soit **20 oscillations en 16,7 s**,
et `ω = 7,54 rad/s`. Fourchette à 90 % compte tenu de toutes les estimations :
**16,1 à 17,2 s**.

- **entre 16 et 17,5 s** → le modèle est validé, on passe au RL sans rien peser ;
- **en dehors** → quelque chose de structurel cloche, et là on cherche.

`python 02_identifier.py` réaffiche la prédiction, `python incertitude.py`
réaffiche la fourchette, à chaque modification du modèle.

## 2. Contre-épreuve : la chute libre — 2 minutes, robot complet

Celle du dessus se fait sur l'établi, roues démontées. Celle-ci se fait sur le
robot entier, posé au sol.

```
S                      désarmer le PID
                       caler le robot à ~1° de l'équilibre, lâcher
                       relever dans logs/robot.log l'instant où P passe 10°
ω = arccosh(10/1) / t  = 2,993 / t
```

Ce n'est **pas** la même grandeur que la mesure 1 : ici le robot peut translater,
le pivot se déplace. Le modèle prédit `ω = 10,0 rad/s`, soit **299 ms** pour aller
de 1° à 10°. Trois lâchers, on garde la médiane.

## 3. Le micropas : 1600 ou 3200 ? — 3 minutes

Le carnet laisse la question ouverte, et elle vaut un **facteur deux sur toutes
les distances**.

Multimètre sur MS1/MS2/MS3 : les trois au 3,3 V = 1/16 = 3200 pas/tour. Le carnet
dit que c'est le câblage constaté — donc 3200 est le plus probable, et le firmware
ment.

Contre-épreuve : repère au feutre sur une roue, robot roues en l'air, `V 60`.
Chronométrer 30 tours. 30 s → 1600 pas/tour. 60 s → 3200.

---

## 4. Les masses — devenu optionnel

`incertitude.py` classe les paramètres par leur effet réel sur `T` :

| Paramètre | Effet sur T | |
|---|---|---|
| `masse_bat_haut` | **4,9 %** | 3 cellules : 14500 (20 g) ou 18650 (45 g) ? |
| `h_bat_haut` | **4,0 %** | hauteur du pack, ±12 mm |
| `masse_elec` | 2,2 % | Labdec + ESP32 + drivers |
| tout le reste réuni | < 1,3 % chacun | |

**Les trois premiers pèsent 66 % de l'incertitude.** Le type de cellule et la
hauteur du pack, c'est-à-dire deux choses qui se règlent à l'œil et au mètre
ruban, pas à la balance :

- **quelle cellule ?** Ø18 × 65 mm = 18650, 45 g. Ø14 × 50 mm = 14500, 20 g.
- **à quelle hauteur** est le centre du paquet de 3, depuis l'axe des roues ?

Peser les pièces imprimées ne changerait `T` que de 1 % au total. Ça ne vaut pas
le démontage.

<details>
<summary>Si tu veux quand même peser — le tableau</summary>

Balance de cuisine à 1 g près. Les volumes viennent des STL, donc une seule
pièce pesée suffit : on en déduit la densité effective et on répartit au prorata.

Les masses actuelles ne sont pas devinées : le volume vient du STL, et la densité
effective de l'**épaisseur caractéristique** `2V/A` de chaque pièce. Une pièce
mince est presque toute en périmètres, donc dense ; une pièce épaisse est surtout
du remplissage, donc légère.

| | volume | 2V/A | densité | masse |
|---|---|---|---|---|
| `piece1` | 92,8 cm³ | 8,01 mm | 0,45 | 41 g |
| `piece2` | 65,2 cm³ | 4,15 mm | 0,63 | 41 g |
| `piece3` | 118,1 cm³ | 5,04 mm | 0,56 | 67 g |
| `piece4` | 103,5 cm³ | 6,17 mm | 0,51 | 52 g |

Total PLA **201 g**. Robot complet estimé à **986 g**.

</details>

## 5. Où sont les piles et la Labdec — 5 minutes, mètre ruban

Les seules géométries absentes de la CAO, et les plus sensibles de la liste : le
bras de levier intervient **au carré** dans l'inertie.

Placement actuel, déduit de ce que tu m'as décrit (`piece4` va de +136 à
+199 mm de l'axe) :

| | Paramètre | Actuel | D'où |
|---|---|---|---|
| 3 cellules **posées sur** la pièce du haut | `h_bat_haut` | 208 mm | +199 + 9 de rayon |
| 1 cellule **collée sous** la pièce du haut | `h_bat_bas` | 127 mm | +136 − 9 |
| la Labdec | `h_elec` | 110 mm | **inventé — la seule position encore devinée** |

`h_elec` est le dernier chiffre sans aucune source. Il pèse 1,3 % sur `T`, donc
ce n'est pas urgent, mais c'est 30 secondes de mètre ruban.

## 6. L'adhérence — 3 minutes, optionnel

Robot désarmé sur une planche qu'on incline jusqu'au glissement :
`µ = tan(angle)`. Provisoire : `frottement = 1.0`.

Optionnel parce que la randomisation de domaine balaiera ce paramètre de toute
façon. Utile seulement si tu soupçonnes du patinage — ton carnet n'en mentionne
aucun.

## 7. Le bruit des capteurs — déjà dans tes logs

`bruit_gyro = 0,019 rad/s` vient du carnet (1,08 °/s d'écart-type). Pour
`bruit_accel`, il faudrait logger `ay`/`az` bruts pendant que les moteurs tournent
à vitesse constante, roues en l'air.

C'est ce bruit-là qui fixe l'écart entre les **0,003°** d'erreur de la simulation
et tes **0,064°** réels. Il n'entrera pas dans le modèle nominal, mais dans la
randomisation.

---

## Le test de recette

Une fois les valeurs saisies dans `modele.py` :

```
python modele.py                 régénère le XML depuis l'assemblage
python 02_identifier.py          ω doit valoir 2π/T mesuré
python 03_pid.py --silence       la cascade doit tenir avec TES gains
python rendu.py --pid 8          et ça doit ressembler à ton robot
```

Si `03_pid.py` tient debout avec `Kp=4500, Ki=40, Kd=600` **sans rien retoucher**,
le modèle est validé pour la commande. C'est le feu vert pour le RL — pas avant.
