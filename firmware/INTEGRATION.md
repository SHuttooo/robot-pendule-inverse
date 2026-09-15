# Mettre la politique sur l'ESP32

Tout ce qu'il faut faire, dans l'ordre. Les seuils ne sont pas choisis à
l'estime : chacun est mesuré, et la mesure est indiquée.

---

## 1. Ce qu'est le fichier `politique.h`

Un en-tête C autonome, 39 ko de source, **aucune dépendance hors `math.h`**.

| | |
|---|---|
| `pol_w0/b0` `pol_w1/b1` `pol_w2/b2` | les 1 634 poids, en `static const float` → **flash, pas RAM** |
| `pol_avant()` | l'inférence : 1 568 multiplications-accumulations, 64 `tanh` |
| `politique_observation()` | construit les 15 entrées depuis ce que le robot mesure |
| `politique_accel()` | l'appel de haut niveau → une accélération en pas/s² |
| `politique_reset()` | **à appeler à chaque armement** |
| `politique_defaut` | 0 si tout va bien, sinon le code du garde-fou déclenché |

Régénérer après tout réentraînement :

```
python 11_export_c.py            # écrit sortie_c/politique.h
python 12_valider_c.py           # compile et compare au Python
```

**Ne jamais téléverser sans avoir lancé `12_valider_c.py`.** Il compile
l'en-tête avec gcc, lui donne 2 400 observations issues d'épisodes réels, et
compare aux actions de `stable-baselines3`. Écart mesuré sur l'export actuel :
**2,1 × 10⁻⁷**, c'est-à-dire la différence float32/float64 et rien d'autre.
Une matrice transposée ou une activation inversée ne se voit pas autrement
qu'en regardant le robot tomber.

---

## 2. Ce que l'agent remplace, et ce qu'il ne remplace pas

Il se substitue à **la boucle interne**, au même point d'insertion :

```
avant   accel = Kp_a*err + Ki_a*angleIntegral + Kd_a*gyroFilt;
après   accel = politique_accel(...);
        wheel_sps += accel * dt;              ← inchangé
```

**Il remplace aussi la boucle externe.** L'agent a l'écart de position et la
consigne de vitesse dans son observation — il fait déjà ce travail. Donc quand
il pilote : pas d'`outerLoop()`, pas d'auto-trim.

L'auto-trim est le point à ne pas rater : il déplace `angleOffset`, or l'agent
lit `pitch - angleOffset`. Le laisser courir reviendrait à faire bouger sa
référence de verticale sous ses pieds pendant qu'il travaille.

**Tout le reste ne bouge pas** : le timer, l'ISR, l'homme mort, le filtre
complémentaire, la calibration gyro, la télémétrie, le chien de garde, les
sécurités I2C.

### Ce que l'agent n'a pas, et ce que ça coûte

Ton firmware lit 8 octets depuis `0x3D` : `AY AZ TEMP GX`. Donc pas de gyro Z,
et un seul timer donc pas d'odométrie par roue. Trois entrées sur quinze sont
câblées à zéro. Mesuré en simulation :

| | survie | erreur d'angle |
|---|---|---|
| toutes les entrées | 100 % | 0,086° |
| sans lacet ni cap — **ton robot** | 100 % | 0,085° |

Et lier les deux roues (un seul timer) : 0,089° contre 0,088°. **Aucun coût.**

---

## 3. LE test à faire en premier : le signe

C'est le seul point qui peut détruire le robot, et il ne se vérifie pas en
simulation. Si le signe de l'angle est inversé, l'agent accélère du mauvais
côté et le robot part par terre en une demi-seconde, à pleine vitesse.

**Roues en l'air, robot tenu à la main. Commande `3`.**

Le mode test n'envoie rien aux moteurs : il affiche seulement ce que l'agent
*voudrait* faire.

```
penche le robot vers l'AVANT   →  accel doit être POSITIVE
penche le robot vers l'ARRIÈRE →  accel doit être NÉGATIVE
```

C'est contre-intuitif à l'arrêt mais c'est bien ça : pour rattraper un robot
qui tombe en avant, il faut accélérer les roues **vers l'avant**, pour ramener
le point d'appui sous le centre de masse.

**La table de référence**, mesurée sur l'export, gyro et vitesse nuls. C'est
exactement ce que le mode `3` doit afficher :

| angle | accel attendue |
|---|---|
| −10° | −14 830 pas/s² |
| −6° | −14 042 |
| −3° | −10 694 |
| −1° | −4 133 |
| 0° | +444 |
| +1° | +4 775 |
| +3° | +10 404 |
| +6° | +13 215 |
| +10° | +13 556 |

Si tu lis des signes opposés, le signe est inversé — voir § 6. Si tu lis des
valeurs très différentes **au même signe**, c'est que `angleOffset` n'est pas
au bon endroit : refais un `Z` robot bien vertical.

---

## 4. Les garde-fous

Trois, tous vérifiés sur le C compilé.

| code | déclencheur | pourquoi ce seuil |
|---|---|---|
| `POL_DEFAUT_ANGLE` | \|angle\| > **14°** | l'agent rattrape 6/6 à 10°, **0/6 à 12°**. Au-delà c'est perdu, autant couper que le laisser s'acharner. |
| `POL_DEFAUT_SATURE` | \|action\| > 0,98 pendant > **120 ms** | en marche normale l'action ne sature **jamais** : 0,0 % du temps, même très perturbé. Aucun faux positif possible. |
| `POL_DEFAUT_NAN` | entrée ou sortie non numérique | voir ci-dessous, c'est le plus important. |

### Pourquoi le test NaN n'est pas une précaution de principe

Aucune comparaison avec `NaN` n'est vraie. Si `wheel_sps` devient `NaN`, dans
`applyMotorSpeed()` :

```c
if (a < MIN_SPS)  → faux   →  il ne coupe pas les moteurs
if (a > lim)      → faux   →  il ne borne pas non plus
half = (uint32_t)(500000.0f / NaN)   →  indéfini
if (half < 20) half = 20;            →  20 µs de demi-période
```

**Soit 25 000 pas/s, moteurs à fond, toutes les bornes traversées en silence.**
Le garde-fou est en amont *et* en aval du réseau.

Ce risque existe aussi avec ta cascade — il est juste moins probable.

### Ce qu'il faut en faire

`politique_accel()` retourne **0** quand un garde-fou mord, et pose le code
dans `politique_defaut`. Le traitement recommandé, dans `innerLoop()` :

```c
if (politique_defaut != POL_OK) {
  pidEnabled = false;
  dualPid = false;
  pendingArm = autoRearm;        // il se relèvera seul si H est actif
  resetLoops();                  // coupe les moteurs, remet tout à zéro
  Serial.print(F(">>> AGENT COUPE, defaut "));
  Serial.println(politique_defaut);
}
```

`resetLoops()` appelle déjà `applyMotorSpeed(0)` : **les moteurs s'arrêtent
dans le même tour de boucle.**

### Ce qui protège déjà, et qui reste actif

| | |
|---|---|
| `FALL_LIMIT` 30° | la chute franche |
| `SAT_MAX_MS` 1500 ms | vitesse saturée = roues dans le vide |
| `i2cFailStreak >= 10` | capteur muet |
| chien de garde IMU 40 ms | plus de données → moteurs coupés |
| `DEADMAN_TICKS` dans l'ISR | la boucle se bloque → l'ISR s'arrête seule |
| chien de garde matériel 2 s | blocage → redémarrage |

Les garde-fous de l'agent viennent **avant** ceux-là, pas à leur place.

---

## 5. Le patch, bloc par bloc

### a) en tête de fichier

```c
#include "politique.h"

bool useAgent = false;      // false = ta cascade, true = l'agent
bool testSigne = false;     // mode verification roues en l air
```

Mets `politique.h` **dans le dossier du croquis**, à côté du `.ino`.

### b) dans `resetLoops()`, à la fin

```c
  politique_reset();        // vide a(t-1), a(t-2) et la cible de position
```

Sans ça, l'agent redémarre après une chute avec l'action qui la précédait et
une cible de position périmée : il corrige une erreur qui n'existe plus. C'est
le même piège que les échéances en date absolue après un `mj_resetData`.

### c) dans `innerLoop()`, remplacer le calcul de `accel`

```c
void innerLoop(float dt) {
  float err = pitch - target_angle;
  float accel;

  if (useAgent) {
    // La politique decide a 100 Hz, l integrateur reste a 200 Hz.
    static bool tickPol = false;
    static float accelPol = 0.0f;
    tickPol = !tickPol;
    if (tickPol) {
      int32_t pos;
      portENTER_CRITICAL(&timerMux);
      pos = stepCount;
      portEXIT_CRITICAL(&timerMux);
      accelPol = politique_accel(pitch - angleOffset, gyroFilt,
                                 wheel_sps, (long)pos, target_speed_sps,
                                 millis());
      if (politique_defaut != POL_OK) {
        pidEnabled = false; dualPid = false;
        pendingArm = autoRearm;
        resetLoops();
        Serial.print(F(">>> AGENT COUPE, defaut "));
        Serial.println(politique_defaut);
        return;
      }
    }
    accel = accelPol;
    angleIntegral = 0.0f;          // l integrale de la cascade ne sert plus
  } else {
    angleIntegral += err * dt;
    angleIntegral = constrain(angleIntegral, -20.0f, 20.0f);
    accel = Kp_a * err + Ki_a * angleIntegral + Kd_a * gyroFilt;
  }

  // ... tout le reste de innerLoop() est INCHANGE :
  //     accelLimit, wheel_sps += accel*dt, anti-emballement,
  //     surveillance de saturation, applyMotorSpeed(wheel_sps)
}
```

### d) dans `loop()`, ne pas appeler `outerLoop()` quand l'agent pilote

```c
        if (dualPid && !useAgent) {
          if (++outerCounter >= OUTER_DIV) { outerCounter = 0; outerLoop(dt * OUTER_DIV); }
        } else {
          target_angle = angleOffset;
          speed_angle_corr = 0.0f;
        }
        innerLoop(dt);
```

### e) trois commandes série

Toutes les lettres sont prises dans ton firmware — j'utilise des chiffres.

```c
    case '1':
      useAgent = false; testSigne = false; resetLoops();
      Serial.println(F("pilote = CASCADE"));
      break;

    case '2':
      useAgent = true; testSigne = false; resetLoops();
      Serial.println(F("pilote = AGENT"));
      break;

    case '3':
      testSigne = !testSigne;
      pidEnabled = false; dualPid = false; resetLoops();
      Serial.println(testSigne
        ? F("TEST DE SIGNE : roues en l air. Penche le robot, lis 'a'.")
        : F("test de signe termine"));
      break;
```

### f) le mode test de signe, dans `loop()`

À placer dans le bloc `if (mpuDataReady)`, juste avant la télémétrie :

```c
    if (testSigne) {
      float a = politique_accel(pitch - angleOffset, gyroFilt,
                               0.0f, 0L, 0.0f, millis());
      if (millis() - lastSigneMsg > 200) {
        lastSigneMsg = millis();
        Serial.print(F("angle ")); Serial.print(pitch - angleOffset, 2);
        Serial.print(F(" deg   ->   a "));  Serial.print(a, 0);
        Serial.println(F(" pas/s2   (avant => POSITIF attendu)"));
      }
    }
```

Rien n'est envoyé aux moteurs dans ce mode.

### g) ajouter le pilote à la télémétrie

```c
    Serial.print(useAgent ? F(" AGENT") : F(" PID"));
```

Sans ça tu ne sauras pas qui pilote en relisant un log — et on s'est déjà fait
prendre par un mode qui avait basculé silencieusement.

---

## 6. Si le signe est inversé

Une seule ligne à changer, dans le patch :

```c
politique_accel(-(pitch - angleOffset), -gyroFilt, ...)
```

**Les deux ensemble.** L'angle et sa dérivée doivent garder le même signe
relatif, sinon le terme dérivé travaille à contresens.

---

## 7. L'ordre des essais

| | quoi | critère de passage |
|---|---|---|
| 1 | `12_valider_c.py` sur le PC | écart < 2×10⁻⁵ |
| 2 | téléverser, `?` pour vérifier que ça démarre | pas de reset, `hz:200`, `f:0` |
| 3 | **`3`, roues en l'air, pencher à la main** | avant → `a` positif |
| 4 | `1` puis `K` — la cascade, comme d'habitude | il tient, référence de comparaison |
| 5 | **`2` puis `K`, robot tenu à la main**, prêt à le rattraper | il ne s'emballe pas |
| 6 | `2`, `K`, lâché, au sol, sur une surface dégagée | il tient |
| 7 | poussées légères, puis `N 300` pour un déplacement | |

Ne saute pas l'étape 3. C'est la seule qui protège le matériel.

Et garde `1` sous la main : une frappe et tu es revenu sur la cascade, sans
réarmer.

---

## 8. Ce qui reste incertain, honnêtement

**Le coût en temps sur la carte n'est pas mesuré.** 1 568 MAC ≈ 13 µs à
240 MHz, mais les 64 `tanhf()` viennent de newlib et coûtent probablement
1 à 2 µs chacun — soit **100 à 140 µs par pas de politique**, une fois sur deux.
Ton log actuel montre `Lus:2172` au pire pour un budget de 5 000 µs : la marge
est de 2 828 µs, donc même 140 µs ne représentent que **5 %**. Mais regarde
`Lus` après le téléversement, c'est la mesure qui tranche.

Si c'est trop cher : `python 11_export_c.py --tanh-rapide` remplace `tanhf` par
une approximation de Padé, environ dix fois plus rapide. Relance
`12_valider_c.py` derrière pour chiffrer ce que l'approximation coûte en
précision.

**Le modèle n'a toujours pas été validé contre la mesure du pendule suspendu.**
Les masses restent estimées. L'agent a été entraîné avec ±30 % dessus, donc il
devrait encaisser — mais ce n'est pas une preuve.

**Un point que ton log de ce soir a révélé :** ton `Ki_spd` vaut 0,0005 et non
0,0008, et l'auto-trim absorbait une correction de position de −0,92° au rythme
de 0,045°/s. Sans importance pour l'agent, qui n'utilise ni l'un ni l'autre.
