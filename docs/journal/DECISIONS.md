# Décisions de conception

Chaque choix, sa raison, et ce qu'il faudrait pour le remettre en cause.
Objectif : ne pas re-débattre de la même chose dans trois semaines.

---

## Le modèle

**Des primitives, pas les STL, pour la physique.** Ce qui compte pour la
dynamique, ce sont les masses et les inerties, pas la forme. Les maillages
portent la masse — MuJoCo en déduit l'inertie sur la vraie géométrie — mais le
contact au sol reste un **cylindre**, plus juste qu'un maillage pour un pneu et
100 fois plus rapide.

Les maillages portent aussi la collision, mais MuJoCo les **convexifie** : le
robot tombe sur son enveloppe, pas sur son cadre ajouré. Faux dans le détail,
sans conséquence — ça ne sert qu'après une chute. Coût mesuré : −24 % de vitesse.

**Modèle 3D par défaut, plan disponible.** L'articulation libre coûte 11 % et
apporte le lacet, sans lequel il n'y a pas de direction. Le modèle plan reste
utilisé par `02_identifier.py`, où mesurer un mode propre est un problème à un
seul degré de liberté.

**Lire l'état par les capteurs, jamais par `qpos`.** Deux raisons : les scripts
deviennent indépendants du choix plan/3D, et surtout on se rend **incapable de
tricher**. L'information privilégiée est la première cause de transfert raté.

**Le pas-à-pas modélisé en actionneur de vitesse, par défaut.** Le modèle fidèle
(`moteur.py`, ressort magnétique `C = C₀·sin(50·Δθ)`) est plus juste — il génère
la vibration mécanique du bon ordre, et divise par deux la poussée encaissée —
mais coûte 7× plus cher (pas de temps 5× plus fin, résonance à 113 Hz).
**À remettre en cause :** il devrait servir au moins comme variante de
randomisation avant tout transfert.

---

## L'environnement d'apprentissage

**Politique à 100 Hz, intégrateur à 200 Hz.** À 200 Hz un épisode fait 2 000
décisions et l'attribution du mérite devient trop bruitée. Sur l'ESP32, la boucle
reste à 200 Hz et la politique se contente d'un tick sur deux.

**Action = accélération de roue, en pas/s².** Exactement la sortie de la boucle
interne du firmware. L'agent devient un **remplaçant direct** de la cascade, au
même point d'insertion. Deux valeurs et non une, parce que le modèle 3D permet
de tourner.

**Observation : 15 nombres, tous mesurables sur le robot.** Aucune vérité
terrain. L'angle vient du même filtre complémentaire que le firmware, appliqué à
des capteurs **bruités**, avec un biais tiré à chaque épisode.

**Récompense : primes en cloche, pénalités sur les différences.** Voir
`PIEGES.md` n° 1 — c'est la seule forme qui garantit une récompense positive
tant que le robot est debout.

**Lissage par la dérivée seconde, pas la première.** `a(t) − 2a(t−1) + a(t−2)`
vaut zéro pour une rampe à pente constante — une grosse correction soutenue ne
coûte rien — et explose dès que la commande s'inverse. Mesuré : la première
différence sépare correction et tremblement d'un facteur 192, la seconde d'un
facteur 4 125. Dosage 0,02 / 0,15.

Conséquence obligatoire : **`a(t−2)` doit être dans l'observation**, sinon on
pénalise quelque chose que la politique ne peut pas contrôler.

**Pas de `VecNormalize`.** L'observation est déjà mise à l'échelle à la main par
des grandeurs physiques connues. La politique est donc **autonome** : pour la
porter sur l'ESP32 il n'y a que des poids à copier, aucune statistique à
transporter ni à tenir à jour.

**Curriculum tiré au hasard entre 0 et le maximum courant**, jamais une rampe —
une rampe fait oublier le cas facile. Le maximum monte **au mérite** (80 % de
survie sur 50 épisodes), et le tampon se vide après chaque montée.

**Poussées : on tire l'impulsion angulaire, puis on répartit.** Ce qui bascule un
pendule, ce n'est ni la force ni la hauteur prises séparément, c'est
`couple × durée`. On tire donc `impulsion ∈ [0 ; 0,045·k] N·m·s`, une hauteur
dans [0,08 ; 0,22] m, une durée dans [0,05 ; 0,20] s, et la force en découle.

---

## L'algorithme

**PPO plutôt que SAC.** La simulation produit 9 000 pas/s : les échantillons ne
coûtent rien, et PPO est plus stable et se parallélise mieux. SAC deviendrait le
bon choix pour apprendre **sur le vrai robot**, où chaque chute coûte cher.

**Réseau 2 × 32, `tanh`.** Choisi d'abord par la contrainte de déploiement :
1 634 poids, 13 µs sur ESP32, 0,3 % du budget. L'architecture standard de la
locomotion (512-256-128) demanderait 691 ko de poids — elle ne tient pas dans les
520 ko de RAM.

`tanh` plutôt que ReLU : lisse (pas de coude qui produirait des à-coups) et
**bornée** — une entrée hors domaine sature au lieu de traverser. Le contre-
argument du gradient qui s'évanouit ne mord qu'à partir de dix couches.

**À remettre en cause :** la mesure de capacité montre que `4 × 16` fait aussi
bien pour 34 % de calcul en moins, et que `1 × 16` atteint déjà R² = 0,997.
Le réseau est probablement surdimensionné. Le choix vient de la convention du
domaine, pas d'une mesure.

**`gamma = 0,99` à 100 Hz** → horizon de 100 pas = 1 s, l'ordre de grandeur du
retour à l'équilibre. Le défaut des bibliothèques est calibré pour 30–50 Hz ; à
200 Hz il rendrait l'agent myope.

**`log_std_init = -1.5`.** Voir `PIEGES.md` n° 2.

---

## L'outillage

**Le rendu est séparé de l'entraînement.** L'entraînement produit des poids, le
rendu produit des images. On peut refaire toutes les vidéos — format, habillage,
scénario, caméra — sans jamais réentraîner.

**Jalons placés d'après la courbe, pas en progression géométrique.** Un
espacement uniforme gaspillait huit jalons sur douze avant le décollage. Les 16
retenus sont resserrés entre 1,3 M et 4 M, là où le comportement change.

**Scénario de film écrit et identique pour tous les pilotes et tous les jalons.**
C'est ce qui rend les vidéos comparables entre elles. Les forces sont calibrées
sur les limites mesurées : la dernière poussée vaut 0,64 N, entre les 0,56 N de
la cascade et les 0,70 N de l'agent — la seule fenêtre où le duel montre quelque
chose.

**Pupitre Tkinter séparé** plutôt que le clavier du viewer. Voir `PIEGES.md` n° 9.

**Boîte noire des chutes.** Une chute ne se diagnostique pas à l'instant où elle
arrive : c'est la seconde d'avant qui contient la cause. `chutes/*.csv` garde
1,5 s d'historique à 100 Hz.
