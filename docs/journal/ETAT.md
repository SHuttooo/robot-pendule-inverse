# Où on en est

> **À lire en premier à chaque reprise. À mettre à jour en dernier avant de couper.**
> Dernière mise à jour : **9 septembre 2026**.

## En une phrase

Le robot balancier est modélisé dans MuJoCo depuis la CAO, la cascade du
firmware y est portée comme témoin, et une politique PPO de 1 634 poids la bat
sur tous les critères sous contrainte. **La politique est exportée en C et
validée à 2×10⁻⁷ près, mais elle n'a pas encore tourné sur la carte.**

## La prochaine étape

**Le témoin cascade, 45 secondes.** C'est la seule mesure qui manque pour
pouvoir écrire un chiffre au lieu d'une impression. Le nouvel agent a volé
76 s sur le robot avec 536 pas/s d'écart-type de commande, contre 893 pour
l'ancien. Mais la cascade n'a pas été relancée juste après, donc le rapport
agent contre cascade sur matériel reste inconnu.

Puis le Vref de l'A4988, qui tranche entre couple faible et roues lourdes.

## 10 et 11 septembre : la direction, le pupitre, la publication

### La direction fonctionne enfin

C'était la limite ouverte depuis le début du projet : les deux broches STEP
partageaient un timer, donc les deux roues recevaient exactement les mêmes
impulsions et le robot ne pouvait pas tourner.

Le timer bat désormais à **cadence fixe, 25 kHz**, et chaque moteur porte son
propre accumulateur en virgule fixe 16.16. Quand il déborde, cette roue-là émet
un pas. C'est un Bresenham : deux vitesses indépendantes sur un seul timer,
erreur de phase bornée à un pas.

Effet de bord : la demi-période ne se reprogramme plus jamais. On avait mesuré
186 reprogrammations par seconde en cherchant la cause d'un plantage. Il n'y en
a plus aucune.

Nouvelle commande `#<valeur>`, différence de vitesse entre les roues, positif =
virage à droite. Plafond 2000, remise à zéro à chaque désarmement.

### Deux bugs trouvés par Matthieu en pilotant

**La consigne accumulait une dette.** La boucle externe intégrait la vitesse
demandée en une cible de position ; quand le robot ne suivait pas, la cible
filait devant et il restait penché à rembourser. Sa formulation : « s'il n'a
pas fait sa consigne il tente à l'infini ». Corrigé par deux régimes séparés :
consigne non nulle, la cible de position SUIT la position réelle et aucune
dette ne s'accumule ; consigne nulle, elle se fige à l'instant du relâchement.

**Les boutons du pupitre étaient sourds.** STOP attendait derrière des dizaines
de commandes de joystick. Corrigé : STOP jette la file et écrit immédiatement,
une consigne écrase la précédente du même type, et la file est poussée toutes
les 20 ms.

### Le pupitre

`tools/pupitre.py`. Joystick 2D, échelles de ×0,5 à ×3, ressort
ou maintien au relâchement, bouton de calage, et l'état en direct dont la
verticale et la correction. Les boutons de pilote s'allument **d'après la
télémétrie**, pas d'après le dernier clic : un ordre perdu se voit.

Il ne touche pas au port série. Il lit `logs/robot.log` et dépose ses commandes
dans `tools/cmd.txt`, comme tout le reste du projet.

### Un piège de plus, à retenir

**L'auto-trim ne tourne pas en mode agent.** Il vit dans la boucle externe, et
celle-ci est désactivée quand l'agent pilote, volontairement, pour que la
verticale ne bouge pas sous ses pieds. Mesuré : en cascade l'offset dérive de
2,09° en 65 s ; en agent il ne bouge pas d'un centième.

**Conséquence pratique : armer la cascade une minute AVANT de passer à
l'agent**, pour que l'auto-trim trouve le vrai point d'équilibre. Sinon l'agent
part avec l'offset qu'on lui a laissé, se penche, et dérive.

## La publication

Vidéo : `rendus/v3_moteur_fidele/post_sim_vs_real.mp4`, 1080x1920, 45 s, muette.
Une seconde de couverture fixe (c'est elle que LinkedIn prend comme vignette),
robot réel plein cadre, simulation en médaillon étiqueté, mosaïque verticale
centrée, signature. Construite par `montage_post.py`, avec `montage_reel.py` et
`19_mosaique_verticale.py`.

Texte : 2 609 caractères, en anglais (hors dépôt).

**Les chiffres du post, revérifiés sur `logs/robot.log.old`** — le journal a
pivoté le 10 à 19 h 12, donc tout le 9 septembre est dans `.old`, et une
analyse faite sur `robot.log` seul donne des chiffres faux :

| | simulation corrigée | robot réel |
|---|---|---|
| erreur d'angle | 1,743° | 1,572° |
| commande | 873 pas/s | 878 pas/s |

Soit 1 % sur la commande et environ 10 % sur l'inclinaison.

**Une objection de Matthieu, tranchée par la mesure.** Il a demandé si le
tremblement de l'ancien agent ne venait pas d'un mauvais calage plutôt que du
modèle de moteur. Non, et pour deux raisons indépendantes : l'offset venait de
l'auto-trim de la cascade qui avait convergé juste avant, et il n'a pas bougé
d'un centième pendant le vol ; et la simulation corrigée, qui n'a aucune erreur
de calage possible, reproduit le même défaut.

## La boucle est bouclée

C'est le résultat de la soirée du 9 septembre. Le même agent, entraîné sur
l'actionneur en vitesse, jugé dans la nouvelle simulation et sur le vrai robot :

| | simulation fidèle | robot réel |
|---|---|---|
| erreur d'angle, écart-type | 1,743° | 1,621° |
| commande, écart-type | 873 pas/s | 893 pas/s |

**2 % d'écart sur la commande, 8 % sur l'angle.** La simulation reproduit
maintenant le défaut observé sur le matériel, et elle n'a jamais été ajustée sur
cette observation : les seules choses recalées sont la résonance à 71 Hz et le
bruit des capteurs, mesurés séparément.

Avant, la simulation disait que l'agent battait la cascade partout. Le robot
disait qu'il tremblait dix fois plus. Les deux disent maintenant la même chose.

## Le nouvel agent

Réentraîné avec `moteur.py`, 9 M pas en 15,7 min, difficulté 0,85.
Tous les agents jugés sur le moteur fidèle, robot calme :

| | ancien | **nouveau** | cascade |
|---|---|---|---|
| erreur d'angle, écart-type | 1,743° | **0,106°** | 0,185° |
| commande, écart-type | 873 pas/s | **47 pas/s** | 86 pas/s |
| dérive de position | 55 mm | 12 mm | 13 mm |
| récompense | 2 451 | **3 777** | 3 833 |
| survie à difficulté 1,0 | 25 % | **42 %** | 33 % |

Erreur divisée par 16, commande par 19. Il passe devant la cascade.

Réserve : le compteur d'inversions monte à 65/s contre 13. Ce n'est pas une
régression — le nouvel agent fait de petites corrections qui traversent zéro
souvent, l'ancien faisait de grandes embardées lentes. C'est l'amplitude qui
compte.

## Ce qui a été construit ce soir

**Le plantage** : la transaction I2C bloquait la boucle de commande. Déplacée
dans une tâche dédiée sur le cœur 0, non abonnée au chien de garde. 252 s de
secousse à plein régime sans un plantage, contre deux en 106 s. Effet de bord :
les échecs I2C au repos sont passés de 1,01/s à 0,00/s.

**L'identification sans manipulation.** Enregistreur 200 Hz dont le vidage ne
fait plus tomber le robot. Balayage sur l'accélération (48 mm de déplacement)
puis sur la consigne d'angle (45 mm), interspectres avec le signal injecté.

**Les mesures** : `pas_par_tour` = 1600 (était DOUTEUX), `bruit_gyro` divisé
par 17, `bruit_accel` mesuré, résonance moteur à 71 Hz au lieu de 113 supposés.
Il reste **2 paramètres non établis sur 24**, contre 4 sur 22 le matin.

**Le modèle** : `moteur.py` branché pour la première fois, résonance recalée,
masse de roue randomisée entre 30 et 110 g avec le couple qui redonne les
71 Hz. Pas de physique relâché de 10 kHz à 2 kHz après vérification :
l'entraînement passe de 34 à 15,7 min.

**La récompense** : pénalité d'inversion de 0,15 à 0,35, et surtout un terme
neuf sur **l'angle de charge du moteur** — ce que le moteur subit, pas ce que
la commande a l'air de valoir.

## Les rendus

Dans `rendus/v3_moteur_fidele/`, tous sur le moteur fidèle. `film.py` le prend
désormais par défaut : filmer un agent sur l'actionneur en vitesse revient à le
filmer sur un robot qui n'existe pas.

| | |
|---|---|
| `comparaison_large.mp4` | ancien contre nouveau, le tremblement se voit |
| `duel_large.mp4` | nouvel agent contre cascade |
| `demo_large.mp4` | le nouvel agent seul |
| `progression_large.mp4` | les 16 jalons, 152 s |
| `mosaique_large.mp4` | les 16 jalons simultanés |

### Le scénario était devenu faux

Matthieu a repéré dans le journal de rendu que **l'agent de 9 M tombait**. Ce
n'était pas un artefact : le scénario poussait à 0,45 puis 0,64 N, valeurs
calibrées sur l'actionneur en vitesse. Le moteur fidèle divise la poussée
encaissée par près de deux, exactement comme `moteur.py` l'annonçait avant
qu'on s'en serve.

**Limites mesurées** (`17_pousser_agents.py`, poussée isolée de 150 ms à 21 cm,
6 graines sur 8) :

| | actionneur vitesse | **moteur fidèle** |
|---|---|---|
| cascade | 0,56 N | **0,47 N** |
| ancien agent | 0,70 N | **0,38 N** |
| nouvel agent | — | **0,53 N** |

L'ancien agent encaisse donc **moins que la cascade** sur le vrai moteur.

**Et un résultat négatif qu'il ne faut pas maquiller** : l'avantage de 13 % du
nouvel agent en poussée isolée ne survit pas à un enchaînement. Dans le
scénario, à 0,55 N les deux tombent, à 0,50 les deux tiennent — il n'existe pas
de fenêtre qui les sépare. On a donc laissé le scénario **sous la limite des
deux** plutôt que de fabriquer une différence non robuste dans une vidéo
destinée à être publiée. Ce qui s'y voit est la vraie différence : le
tremblement.

Scénario figé : 0,28 N à 1,8 s et 4,0 s, 0,50 N à 16 s. Vérifié : le nouvel
agent tient, la cascade tient, l'ancien agent tombe à 4,8 s.

La progression raconte bien l'apprentissage : chute à 0,8 s jusqu'à 1,3 M de
pas, puis 2,3 s, 4,8 s, 15,6 s, et les deux derniers jalons tiennent le
scénario entier.

## Bloqué sur le matériel — 9 septembre, 23 h 15

Le vol du nouvel agent **n'a pas pu être fait**. Le firmware est téléversé, le
test de signe est passé (portage vérifié à 0,2 % contre gcc), il ne manque que
l'essai.

**Symptômes** : une roue ne tourne plus et les moteurs sifflent à l'arrêt. Un
élément 18650 sur quatre est mort, les trois autres sont mal chargés.

> **Correction du 15 septembre 2026.** Il n'y a pas de problème de batterie :
> elles étaient déchargées. Une fois rechargées, le robot a retrouvé son
> comportement et le nouvel agent a volé. Les points 1 et 5 de la liste
> « À faire » ci-dessous sont donc caducs.

**La cascade a servi de témoin, et c'est ce qui a sauvé la mesure.** Lancée
avant l'agent :

| | référence 21 h 33 | après recharge partielle |
|---|---|---|
| erreur d'angle, écart-type | 0,155° | **1,851°** |
| commande, écart-type | 158 pas/s | **1 035 pas/s** |

Douze fois pire, et tenue seulement 11 s. Sans ce témoin, on aurait mesuré
l'agent, vu un désastre et conclu que le réentraînement avait raté.

**Diagnostic le plus probable** : sous-tension. Les bobines restent alimentées
à l'arrêt ; quand la tension chute, le découpage du driver n'arrive plus à
réguler et siffle. Et comme les deux moteurs reçoivent les mêmes impulsions
d'un seul timer, celui qui a le plus de frottement décroche le premier — ce
n'est pas une roue en panne, c'est une roue en limite.

## À faire au retour du robot, dans l'ordre

1. **Refaire le pack.** Quatre cellules identiques choisies pour le courant
   (Samsung 25R, Sony VTC6, Molicel P26A), pas pour la capacité. Ne pas mélanger
   une neuve avec trois usées.

2. **Mesurer le Vref de l'A4988.** Hypothèse chiffrée : la résonance à 71 Hz
   implique un couple de 0,111 N·m si les roues font 30 g. Or un 17HS3401 réglé
   à 0,5 A donnerait **0,108**, à 3 % près. Matthieu n'a jamais touché le
   potentiomètre — or ces cartes sortent d'usine à un réglage arbitraire, souvent
   bas. Si le Vref lit ~0,4 V avec des shunts `R100`, tout se recoupe, et monter
   à 1 A rendrait **le double de couple**. Si le Vref est correct, alors ce sont
   les roues qui font 110 g. Cette mesure tranche l'ambiguïté mieux qu'une
   balance.

3. **Commande `5`, roues en l'air, 20 s.** La résonance vaut la racine du couple
   disponible. Retour à 71 Hz = moteur et driver sains, c'était l'alimentation.

4. **Puis le vol.** Cascade témoin, puis agent. La prédiction à confirmer :
   **écart-type de commande ~47 pas/s**, contre 873 pour l'ancien agent et 86
   pour la cascade.

5. **Un pont diviseur vers une entrée analogique.** Il n'existe aucune lecture
   de tension sur ce robot, et c'est pour ça qu'un élément est mort sans
   prévenir. La résonance du moteur ne le dit pas : l'A4988 régule le courant,
   donc le couple reste identique jusqu'à ce que la régulation lâche — mesuré
   71,0 Hz à 21 h 56 et 71,5 Hz à 22 h 40 avec un élément déjà mort.

## Pistes matérielles notées

`TMC2209` en remplacement de l'A4988 : courant réglé en logiciel, silencieux,
**détection de décrochage intégrée** — exactement le signal qu'on a cherché à
mesurer indirectement toute la soirée. 6 à 8 € pièce, le meilleur euro à
dépenser sur ce robot.

Encodeurs : `stepCount` compte les pas **commandés**, jamais les pas faits.
L'AS5600 a une adresse I2C fixe donc on ne peut pas en mettre deux ; préférer
AS5047P ou MT6701 en SPI, ou de la quadrature comptée en matériel. Matthieu a
des AS5600 sous la main, réservés à un autre projet.

`RP2350` : ses blocs PIO génèrent les impulsions de pas en matériel, ce qui
lèverait la contrainte du timer unique qui empêche aujourd'hui la direction, et
aurait rendu impossible le plantage corrigé ce soir.
