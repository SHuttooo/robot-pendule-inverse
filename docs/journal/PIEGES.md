# Catalogue des pièges

Chaque entrée : le **symptôme** tel qu'il apparaît, la **cause**, le **coût**,
et la **règle** qui en sort. C'est la partie la plus réutilisable du projet —
les bugs se transposent à l'humanoïde, pas le réseau.

---

## 1. Une récompense négative apprend au robot à se suicider

**Symptôme.** Après 600 000 pas, `ep_len_mean` plafonne à 91 sur 1 000. Le robot
tombe en 0,9 s et n'en bouge plus. La courbe est parfaitement plate.

**Cause.** Je pénalisais l'écart de vitesse en quadratique, sans borne :

```
sps      pénalité vitesse   prime survie   TOTAL
   0            −0,00            1,00       1,00
 800            −0,93            1,00       0,07
1200            −2,08            1,00      −1,08   ← négatif
```

Or un pendule inversé **doit** lancer ses roues pour se rattraper. Je pénalisais
exactement ce qui le sauve, plus fort que le fait de rester debout. Dès qu'un
pas rapporte une récompense négative, **terminer l'épisode devient optimal**.

**Coût.** 600 000 pas d'entraînement, et un diagnostic erroné en cours de route
(« il n'apprend pas » alors qu'il apprenait très bien la mauvaise chose).

> **Règle.** Tant que le robot est debout, la récompense d'un pas doit rester
> **positive**. Utiliser des termes en cloche `exp(−(x/σ)²)`, bornés dans [0, 1],
> qui ne peuvent jamais écraser la prime de survie.

---

## 2. `log_std_init = 0` : un bruit d'exploration pleine échelle

**Symptôme.** 1,8 million de pas de plateau à `ep_len_mean ≈ 85` avant que quoi
que ce soit ne décolle.

**Cause.** Le défaut de SB3 donne σ = 1,0 sur une action qui vaut ±25 000 pas/s².
L'exploration était un bruit blanc à pleine échelle : impossible d'équilibrer
par hasard, donc aucune trajectoire de départ à exploiter.

**Correction.** `log_std_init = -1.5`, soit σ = 0,22. Le décollage passe de
1,8 M à 1,5 M de pas et devient franc.

> **Règle.** Le bruit d'exploration se règle **par rapport à l'échelle de
> l'action**, pas par rapport au défaut de la bibliothèque.

---

## 3. Le curriculum qui monte à chaque pas

**Symptôme.** La difficulté passe de 0,15 à 1,00 en 300 millisecondes, à la fin
de l'entraînement. L'agent n'a jamais vu les niveaux intermédiaires.

**Cause.** Une fenêtre glissante de 50 épisodes. Une fois le seuil de survie
franchi, elle **reste** au-dessus, donc la condition se déclenche à chaque pas.

**Correction.** Vider le tampon après chaque montée : chaque palier doit être
re-mérité sur des épisodes neufs.

> **Règle.** Un curriculum monte **au mérite**, et le mérite se reconstate sur
> des données fraîches après chaque changement.

---

## 4. La poussée appliquée dans le repère du monde

**Symptôme.** Même commande « pousser en arrière », le robot part parfois en
avant, parfois en arrière. Repéré par Matthieu en pilotant.

**Cause.** `mj_applyFT` recevait une force le long de **x du monde**. Le robot
démarre avec un cap tiré au hasard sur ±180° — la direction de poussée était donc
arbitraire par rapport à lui.

**Conséquence bien plus grave.** Les poussées **d'entraînement** avaient le même
défaut. La composante utile valait `2·cos(θ)` avec θ uniforme, soit une moyenne
de 1,27 N au lieu de 2,00 — **36 % de moins qu'annoncé**, et parfois presque
rien quand le robot était de profil.

**Correction.** Force le long de l'avant du robot, lacet seulement :

```python
lac = radians(etat.lacet(m, d))
avant = [cos(lac), sin(lac), 0]
```

Après correction, l'avantage de l'agent sur la cascade en rejet de perturbation
passe de **+8 % à +25 %**.

> **Règle.** Toute grandeur directionnelle — force, vitesse, angle — doit être
> exprimée dans un repère **explicitement choisi**. Le même piège a frappé deux
> fois : le tangage était aussi mesuré dans le repère du monde, et se répartissait
> entre tangage et roulis dès que le robot avait tourné.

---

## 5. Une observation sans borne, et le réseau se jette par terre

**Symptôme.** Le robot encaisse une poussée, puis « fait un truc » et tombe
quelques secondes plus tard. Repéré par Matthieu.

**Cause.** L'écart de position entrait dans l'observation sans borne. Il n'avait
**jamais dépassé 1,10** pendant l'entraînement :

```
écart 0,5 m  →  obs = 2,50   il tient
écart 1,0 m  →  obs = 5,00   CHUTE en 0,64 s
écart 2,0 m  →  obs = 10,0   CHUTE en 0,42 s
```

Un réseau n'extrapole pas. Devant une valeur inconnue il produit une accélération
aberrante — ici, pour « rentrer » — et se jette par terre.

**Correction.** Saturer l'écart à ±1,75 dans l'observation, plus un `clip(−6, 6)`
général sur toutes les entrées. **Le même agent, sans réentraînement, tient
ensuite jusqu'à 10 m d'écart.**

**Le point remarquable.** C'est exactement le problème que `MAX_LEAD` résout dans
le firmware du robot réel — la dette de position qui file et se rembourse d'un
coup. Découvert indépendamment des deux côtés.

> **Règle.** **Borner toutes les observations.** C'est gratuit dans le domaine
> normal et ça évite la catastrophe au-delà. Vérifier après entraînement
> l'amplitude réellement vue de chaque entrée.

---

## 6. `'' in 'GK'` vaut `True`

**Symptôme.** Dans le bac à sable, le robot dérive comme s'il n'avait plus de
boucle externe. Repéré par Matthieu : « il est censé avoir 2 boucles ».

**Cause.** Le gestionnaire de touches faisait `c = chr(code) if 32 <= code < 127
else ''`, puis `elif c in 'GK'`. En Python, **la chaîne vide est incluse dans
toute chaîne**. Une touche non imprimable — `Ctrl`, `Maj`, une flèche — basculait
donc silencieusement en mode G et coupait la boucle externe.

Un appui sur `Ctrl` pour tirer à la souris suffisait.

> **Règle.** Tester l'appartenance avec un **tuple**, jamais une chaîne.
> Et rejeter les codes non imprimables en amont.

---

## 7. `mj_resetData` remet `d.time` à zéro

**Symptôme.** Après un « redresser », le robot repart comme si une force
permanente le poussait. Puis, plus tard : chute à 1 N, une force qu'il encaisse
largement.

**Cause.** Les fins de poussée étaient stockées en **date absolue**
(`fin = d.time + 0.15`). Une remise à zéro du temps — par `mj_resetData`, ou par
ma prolongation d'épisode toutes les 10 s — rendait la condition
`d.time < fin_poussee` à nouveau vraie, et **réappliquait la force pendant dix
secondes**.

> **Règle.** Ne jamais mémoriser une échéance en date absolue quand l'horloge
> peut être remise à zéro. Annuler explicitement les événements en cours lors
> d'une réinitialisation.

---

## 8. Le viewer passif n'applique pas la perturbation souris

**Symptôme.** Ctrl + glisser dans la fenêtre MuJoCo ne produit aucune force.

**Cause.** `launch_passive` **expose** `viewer.perturb` mais ne l'applique
jamais — c'est au script d'appeler `mujoco.mjv_applyPerturbForce`.

Et attention : c'est un **ressort** vers une position de référence, pas une
impulsion. Si `perturb.active` reste armé, il tire indéfiniment.

---

## 9. Le viewer s'approprie presque toutes les touches

**Symptôme.** Un appui sur `Z` déclenche l'action voulue **et** une bascule
d'affichage MuJoCo.

**Cause.** Le viewer réserve `Z` lumière, `Q` caméras, `S` échelle d'inertie,
`D` point de sélection, `H` enveloppe convexe, `P` découpe des contacts, `C`
points de contact, `T` transparence, `M` centre de masse… Le callback utilisateur
reçoit la touche **en plus**, sans pouvoir l'intercepter.

**Correction.** Interface séparée — un pupitre Tkinter (`09_piloter.py`).

---

## 10. Pièges d'outillage, sans conséquence mais coûteux en temps

- **MuJoCo 3.x n'a plus le drapeau `sensornoise`.** Le bruit s'ajoute côté Python.
- **Sous Windows, `SubprocVecEnv` relance l'interpréteur** : la classe
  d'environnement doit vivre dans un module importable, et un nom de module ne
  peut pas commencer par un chiffre. D'où `balancier_env.py` distinct de
  `06_env.py`.
- **`offheight` du XML limite le rendu hors-écran.** Rendre en 1080×1350 exige
  de l'augmenter.
- **Annoncer une correction sans la vérifier.** C'est arrivé deux fois : un motif
  de remplacement n'avait pas matché, et j'ai affirmé que la collision du châssis
  était activée alors qu'elle ne l'était pas. **Toujours relire le fichier
  produit, pas le script qui le produit.**

---

## 11. Une politique exportée porte un état : la tester hors boucle la fait dériver

**Symptôme.** Test de signe roues en l'air, robot immobile à 0,20° : la sortie
affichée monte à chaque impression — 17 489, puis 17 645, puis 17 739 pas/s².
La table de référence donne +444 à cet angle.

**Cause.** `politique_accel()` écrit `pol_a_prec` et `pol_a_prec2`, qui sont
quatre des quinze entrées du réseau. Mon bloc de test les laissait s'accumuler
d'un appel au suivant. Pire : le test ment au réseau en lui passant `sps = 0` et
`pas = 0` quoi qu'il commande. Il voit donc une roue qui ne répond jamais et
pousse de plus en plus fort — un enroulement d'intégrateur, mais dans les poids.

**Coût.** Le test aurait été lu comme un signe correct avec une amplitude
aberrante, ou comme une politique instable. Aucune des deux lectures n'aurait
été vraie.

**Règle.** Un appel de politique hors de sa boucle fermée doit être encadré de
`politique_reset()`. Et toute table de référence doit dire depuis quel état elle
a été calculée.

---

## 12. Le moniteur série ne survit pas toujours au téléversement

**Symptôme.** Après un flash réussi, `logs/robot.log` s'arrête douze jours plus
tôt sur `--- aucun port detecte, attente ---`. Le robot semble muet.

**Cause.** Le processus du moniteur avait disparu. Le script lâche bien le port
quand `esptool` tourne, mais il n'est pas immunisé contre sa propre mort.

**Règle.** Avant de conclure quoi que ce soit d'un log muet, vérifier que le
moniteur est vivant, et regarder l'horodatage de la dernière ligne, pas son
contenu.

---

## 13. Six lignes ne font pas une distribution

**Symptôme.** `Lus` lu à 2 060 µs après le patch contre 1 203 µs avant : j'ai
annoncé une régression de temps de boucle de +70 %.

**Cause.** J'ai comparé six lignes de queue à six lignes de queue. Sur les
20 875 lignes de l'ancien log au même état, la distribution est déjà bimodale :
52 % sous 1 500 µs, 48 % au-dessus. Rien n'avait changé.

**Règle.** Une comparaison avant/après se fait sur les deux distributions
complètes, à état égal — même mode, même `hz`. C'est la même règle que pour le
1,35 Hz du carnet : un chiffre isolé ne mesure rien.

---

## 14. Le plantage n'était pas dans le code qu'on venait d'écrire

**Symptôme.** La carte redémarre après quelques dizaines de secondes, toujours
pendant que l'agent pilote. Chien de garde de tâche sur `loopTask`, puis défaut
d'interruption sur le cœur 0.

**Ce que j'ai cru.** Que l'agent, en commandant des inversions à pleine
amplitude, martelait le pilote de timer de pas. L'hypothèse était cohérente : le
firmware documentait déjà une panne de cette famille.

**Ce que la mesure a dit.** Une boîte noire en RAM RTC, qui survit au
redémarrage et enregistre l'étape de boucle en cours. Quatre plantages, quatre
fois `bloque a l etape 3 (transaction I2C)` — dont deux en mode secousse, qui
ne lit pas l'angle et ne fait tourner aucune boucle, et un en cascade seule.
Et les compteurs ont tué l'hypothèse du timer : la cascade le reprogramme déjà
**186 fois par seconde**, le maximum possible à 200 Hz, tandis que la secousse
le reprogramme **0,07 fois par seconde** et plante quand même.

**Coût.** Deux heures à instrumenter la mauvaise piste, et une affirmation
fausse annoncée en cours de route : « la cascade n'a jamais planté ». Elle
n'avait pas planté pendant les 92 premières secondes.

**Règle.** Quand une panne apparaît en même temps qu'une nouveauté, le réflexe
est de l'imputer à la nouveauté. Construire d'abord le mode qui fait tourner le
matériel **sans** la nouveauté : c'est ce qui a tranché en une mesure. Et un
instrument qui survit au plantage vaut dix hypothèses — quand ça plante, plus
rien ne s'imprime, donc la trace série ne peut par construction rien dire.

---

## 15. Une boucle de calibration qui ne nourrit pas le chien de garde

**Symptôme.** Après une chute, la carte redémarre en boucle, toujours à
**5 719 ms d'uptime**, chaque fois précédée de « robot pas assez immobile, on
recommence ».

**Cause.** La calibration du gyro fait 300 lectures espacées de 3 ms, jusqu'à
quatre essais, sans jamais appeler `esp_task_wdt_reset()`. Un robot qui remue
fait échouer l'essai, et la boucle dépasse les 5 s du chien de garde.

**Règle.** Toute boucle d'initialisation qui peut recommencer doit nourrir le
chien de garde. Le symptôme — un uptime identique à la milliseconde près — est
la signature d'un chien de garde, jamais d'un aléa.

---

## 16. Le validateur devinait quel agent il validait

**Symptôme.** `12_valider_c.py` annonce 7,8 % d'écart relatif entre le C et
PyTorch, là où il donnait 2×10⁻⁷ la veille. Verdict : « ne pas téléverser ».

**Cause.** L'exporteur prend `--agent`, mais le validateur choisissait tout seul
le dernier jalon `agents/balancier_*_pas.zip`. On avait exporté
`balancier_final.zip`. Il comparait donc **le C d'un agent au Python d'un
autre**. Les deux codes étaient justes.

**Coût.** Un export déclaré mauvais alors qu'il était bon, et le téléversement
suspendu à tort.

**Règle.** Un outil de vérification ne doit **jamais deviner** ce qu'il
vérifie. `politique.h` portait déjà la réponse en ligne 5 ; il suffisait de la
lire. L'exporteur écrit désormais aussi `sortie_c/agent_source.txt`, et le
validateur lit l'un ou l'autre.
