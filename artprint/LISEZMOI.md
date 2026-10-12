# ArtPrint : slicer Unlayered + Klipper

Ce dossier contient ce qu'il faut ajouter au slicer Unlayered pour imprimer sur **ArtPrint**, avec une **carte classique sous Klipper** au lieu de la carte Duet d'Unlayered.

## Pourquoi un « traducteur » ?

Le slicer Unlayered écrit, pour chaque point, **où se trouve la pointe de la buse sur le plateau**, ainsi que les angles **B** (la tête penche) et **C** (le plateau tourne). Sur la machine Unlayered, c'est le firmware de la carte Duet qui calcule ensuite où doit aller chaque moteur. Klipper ne sait pas faire ce calcul.

Le traducteur `traducteur/unlayered_to_klipper.py` fait **le même calcul que le firmware Unlayered**, mais sur l'ordinateur, avant l'impression :

1. il tourne la position selon l'angle C du plateau ;
2. il corrige le décalage dû au bras de la tête quand elle penche (angle B) ;
3. s'il y a des paires de moteurs qui travaillent ensemble (comme chez Unlayered), il calcule la position de chaque moteur ;
4. il découpe en petits morceaux (1° maximum) les mouvements où la tête penche ou le plateau tourne.

Le résultat est un G-code que Klipper peut exécuter directement.

## Fichiers

| Fichier | Rôle |
|---|---|
| `traducteur/unlayered_to_klipper.py` | le traducteur (Python 3, sans bibliothèque à installer) |
| `traducteur/machine.json` | **les mesures de la machine** (à remplir avec celles d'ArtPrint) |
| `klipper/artprint-axes.cfg` | brouillon de configuration Klipper pour les 5 axes |
| `tests/test_traducteur.py` | tests automatiques |

## Utilisation

### À la main
```
python artprint/traducteur/unlayered_to_klipper.py ma_piece.gcode -o ma_piece_artprint.gcode
```

### Automatiquement depuis le slicer
Dans le slicer : **Process ▸ Others ▸ Post-processing scripts** (mode Avancé), mets :
```
python "C:\chemin\vers\artprint\traducteur\unlayered_to_klipper.py"
```
Le slicer appelle alors le traducteur à chaque export de G-code.

## Réglages (`machine.json`)

| Réglage | Signification | Valeur Unlayered |
|---|---|---|
| `kinematics` | `cartesian` = un moteur par axe (**choix d'ArtPrint**) ; `corexbyc` = paires de moteurs comme Unlayered | `corexbyc` |
| `d6` | distance pivot de la tête → pointe de la buse (mm) | 46,4 |
| `a5` | petit décalage de côté entre le pivot et la buse (mm) | 2,5 |
| `b_home` | angle B quand la buse est verticale | −90 |
| `b_ratio`, `c_ratio` | réduction des axes B et C | 3 et 5,5 |
| `output_axes` | noms des axes écrits dans le G-code (ArtPrint : X Y Z B C) | X Y Z U V |
| `max_segment_deg` | rotation maximale par petit morceau (°) | 1 |

## Tester
```
python -m unittest discover -s artprint/tests -v
```
Un des tests compare le traducteur au calcul exact du firmware Unlayered (`FiveAxisKinematics.cpp`, `M669 K16`), recopié ligne par ligne.

## Ce qui reste à faire
- Remplir `machine.json` avec les mesures d'ArtPrint.
- Choisir la carte et compléter `klipper/artprint-axes.cfg` (broches, pas, limites).
- Écrire le profil d'imprimante ArtPrint dans le slicer (G-code de début et de fin pour Klipper).
- **Prise d'origine** des axes B et C (inclinaison et rotation).
