# Données brutes du robot

`logs_robot_2026-08-28_au_2026-09-12.zip` (3,7 Mo compressés, 31 Mo bruts) :
tout le port série du robot, horodaté à la milliseconde par
`tools/serial_monitor.ps1`. C'est la source de chaque chiffre mesuré sur le
matériel dans ce dépôt.

| fichier | période | contenu |
|---|---|---|
| `robot-avant.log` | jusqu'au 28 août | la mise au point de la cascade PID (carnet) |
| `robot.log.old` | jusqu'au 10 sept, 19 h 12 | crash I2C, identification du moteur, premier vol de l'agent |
| `robot.log` | 10 au 12 sept | direction, pupitre, la prise vidéo du 10 sept à 21 h 06 |

**Piège :** le journal pivote à 20 Mo. Tout le 9 septembre est donc dans
`robot.log.old`, et une analyse faite sur `robot.log` seul donne des chiffres
faux. C'est arrivé.

Pour s'en servir : décompresser dans `logs/` à la racine du dépôt (le dossier
est ignoré par git).

## Lire une ligne de télémétrie

```
10:53:31.710 P:-90.06 tgt:-90.46 e:0.40 g:6.2 PID DUAL sps:-19 c:-0.62 off:-91.08 pos:-165 hz:199 f:0 Lus:1225 Ius:1170 Tp:0 Tr:0
```

Le détail de chaque champ est dans [`firmware/README.md`](../firmware/README.md#télémétrie).
