# Firmware ESP32

[English](README.md) · **Français**

Un seul croquis Arduino, `robot_balancier/robot_balancier.ino`, et l'en-tête
généré `robot_balancier/politique.h`.

| | |
|---|---|
| carte | ESP32 Dev Module (`esp32:esp32:esp32`) |
| core | esp32 d'Espressif 3.3 (compatible 2.x par les macros `TIMER_*`) |
| taille | 364 ko de flash (27 %), 71 ko de RAM (21 %) |
| bibliothèques | aucune hors du core : `Wire`, `Preferences` |

```
arduino-cli compile --fqbn esp32:esp32:esp32 robot_balancier
```

**Avant de téléverser :** vider `tools/cmd.txt` (une commande oubliée part au
redémarrage), et poser le robot : le téléversement coupe la carte huit
secondes.

---

## Deux pilotes pour le même robot

**Le double PID** (commande `1`), réglé à la main pour mettre en pratique mes
cours d'automatique. Les gains viennent d'essais et de mesures sur le robot,
pas d'une modélisation du pendule avec placement de pôles : j'ai sauté cette
étape pour passer directement à l'apprentissage par renforcement.

```
boucle interne, 200 Hz, cadencée par l'INT du MPU-6050
    accélération roue = Kp·erreur + Ki·∫erreur + Kd·gyro      [pas/s²]
    vitesse roue     += accélération · dt                      l'intégrateur EST la commande
boucle externe, 40 Hz
    angle cible = offset − (Kp_v·erreur de vitesse + Ki_v·erreur de position)
auto-trim, très lent
    recale l'offset pour que la correction externe tende vers zéro
```

Pour un pendule inversé, c'est l'accélération du point d'appui qui redresse le
corps. Une vitesse proportionnelle à l'angle garde toujours un pôle réel
positif, quels que soient les gains : c'était la première version, et elle ne
pouvait pas marcher.

**L'agent** (commande `2`) : `politique.h`, un réseau 15 → 32 → 32 → 2 en tanh,
1 634 poids en flash. Il décide à 100 Hz et remplace **les deux boucles** : il
a déjà l'écart de position et la consigne de vitesse dans ses entrées.
L'intégrateur de vitesse, les limites et les sécurités restent en place.

> **L'auto-trim ne tourne pas en mode agent.** Il vit dans la boucle externe,
> désactivée quand l'agent pilote. Armer le double PID **une minute avant** de
> passer à l'agent, pour que l'offset corresponde à la vraie verticale.

Le mode d'emploi complet de l'intégration, et le protocole de vérification du
signe, sont dans [`INTEGRATION.md`](INTEGRATION.md). Pour régénérer l'en-tête
après un réentraînement : `simulation/11_export_c.py` puis
`simulation/12_valider_c.py`, **jamais l'un sans l'autre**.

---

## Architecture temps réel

**Lecture de l'IMU dans une tâche dédiée**, sur le cœur 0. Réveillée par l'INT
du MPU-6050, elle lit 8 octets à partir du registre 0x3D (AY, AZ, TEMP, GX) et
publie l'échantillon sous verrou. Quand la transaction I2C vivait dans
`loop()`, un bus figé bloquait la boucle de commande et le chien de garde
redémarrait la carte : deux plantages en 106 s de secousse. Après : 252 s sans
un seul, et 0 échec I2C au repos contre 1 par seconde.

**Génération des pas.** Un timer bat à 25 kHz. Chaque roue a son accumulateur
en virgule fixe 16.16 ; quand il déborde, cette roue fait un pas (algorithme de
Bresenham). Deux vitesses indépendantes sur un seul timer, donc la direction.

**Sécurités**, de la plus rapide à la plus lente :

| | |
|---|---|
| homme mort dans l'ISR | 1 250 tops (50 ms) sans nouvelle consigne : les pas s'arrêtent |
| IMU muette 40 ms | la commande est coupée |
| IMU muette 150 ms | désarmement, `>>> SECURITE : bus I2C fige` |
| IMU muette 3 s | redémarrage de la carte |
| inclinaison > 30° | coupure |
| chien de garde | redémarrage si `loop()` se bloque |
| boîte noire | l'étape en cours est écrite en RAM RTC, qui survit au redémarrage : après un plantage, le démarrage dit **où** la boucle s'est arrêtée |

---

## Commandes série

115 200 bauds. Une lettre, éventuellement suivie d'une valeur (`N400`).

**Piloter**

| cmd | effet |
|---|---|
| `1` | pilote = double PID |
| `2` | pilote = agent |
| `K` | arme la double boucle, angle et position |
| `G` | angle seul (le robot dérive, c'est normal) |
| `S` | **arrêt**, annule aussi un armement en attente |
| `Z` | calage : l'angle courant devient la verticale |
| `N`*sps* | consigne de vitesse en pas/s |
| `#`*sps* | différence de vitesse entre les roues, positif = virage à droite, ±2 000 |
| `W`*s* | durée d'un trajet `N` ; non nulle, le robot fait l'aller puis le retour |

**Tester et mesurer**

| cmd | effet |
|---|---|
| `3` | test de signe de la politique, **roues en l'air**, moteurs muets |
| `4` | secousse : inversions pleine amplitude à 5 Hz, sans IMU ni asservissement |
| `5` | rampe de décrochage moteur, **roues en l'air** |
| `6` | enregistrement 200 Hz pendant 6 s, vidé en CSV (`ENR`) sans désarmer |
| `7` | balayage de 0,5 à 20 Hz en 5 s, injecté sur l'accélération. Le robot vibre sur place. Double PID armé d'abord (`1` puis `K`). |
| `9` | balayage de 0,2 à 3 Hz en 5,5 s sur la consigne d'angle, amplitude 1°. Le robot se dandine sur environ 2 cm. |
| `0` | avance d'environ 10 cm après 1 s de repos, enregistrée, puis revient seul. Double PID armé d'abord. |
| `8` | affiche un message d'un ancien protocole de poussée, sans effet |
| `C` | relance la calibration du gyroscope |
| `V`*rpm* | mode manuel avec rampe, roues en l'air |

**Régler** (sauvegardé en flash)

| cmd | paramètre |
|---|---|
| `P` `I` `D` | gains de la boucle interne |
| `T` `Y` | gains vitesse et position de la boucle externe |
| `O` | offset d'angle |
| `L` | constante de temps du lissage de vitesse |
| `U` `E` | coup de pouce au démarrage, avance max de la cible |
| `M` `R` `B` | vitesse max, accélération max, accélération du mode manuel |
| `A` `H` `J` `X` | bascules : auto-trim, réarmement auto, armement au démarrage, signe de la boucle externe |
| `F` `Q` `?` | forcer la sauvegarde, effacer la flash, afficher tous les paramètres |

---

## Télémétrie

Une ligne toutes les 100 ms :

```
P:-90.06 tgt:-90.46 e:0.40 g:6.2 PID DUAL sps:-19 c:-0.62 off:-91.08 pos:-165 hz:199 f:0 Lus:1225 Ius:1170 Tp:0 Tr:0
```

| champ | sens | repère |
|---|---|---|
| `P` | angle mesuré, degrés | verticale vers −91° (IMU couchée) |
| `tgt` | angle cible | |
| `e` | erreur d'angle | |
| `g` | vitesse gyroscopique filtrée, °/s | ≈ 0 au repos |
| `PID` / `AGENT` | pilote actif | |
| `DUAL` / `ANG` / `OFF` | double boucle, angle seul, désarmé | |
| `sps` | commande de vitesse roue, pas/s | saturée = chute imminente |
| `c` | correction de la boucle externe, degrés | > 1° durablement = offset faux |
| `off` | offset d'angle | stable au repos |
| `pos` | odométrie, en pas commandés | |
| `hz` | fréquence de boucle | 200 |
| `f` | échecs I2C cumulés | 0 |
| `Lus` | pire itération de boucle, µs | < 3 000 |
| `Ius` | pire transaction I2C, µs | < 1 500 |
| `Pus` | pire inférence de la politique, µs (mode agent) | |
| `Tp` `Tr` | reprogrammations et rallumages du timer | 0 depuis le timer à cadence fixe |
| `BLOC` | présent seulement si le bus I2C est bloqué | |

---

## Les outils, dans `tools/`

Un seul programme tient le port série : **`serial_monitor.ps1`**. Il détecte la
carte, écrit tout dans `logs/robot.log` horodaté à la milliseconde, envoie ce
qu'on dépose dans `tools/cmd.txt`, et libère le port tout seul pendant un
téléversement. Le fichier `tools/PAUSE` libère le port, `tools/STOP` arrête le
moniteur. Il ne survit pas toujours au téléversement : vérifier qu'il tourne
avant de conclure quoi que ce soit d'un journal muet.

**`pupitre.py`** : joystick 2D pour la vitesse et la direction, échelles de
×0,5 à ×3, calage, choix du pilote, STOP (touche Échap). Il ne touche jamais le
port : il lit le journal et écrit dans `cmd.txt`. Ses boutons s'allument
d'après la télémétrie, pas d'après le dernier clic, donc un ordre perdu se voit.

```
powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
python tools\pupitre.py
```
