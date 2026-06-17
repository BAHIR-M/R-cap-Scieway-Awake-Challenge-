# R-cap-Scieway-Awake-Challenge-

## Présentation

Ce dépôt contient un prototype de véhicule autonome utilisant un capteur LiDAR LD19 et un ESP32. Le projet combine la lecture de données de distance et d’angle du LiDAR pour détecter un « gap » libre devant le robot, puis pilote un servo et un variateur électronique (ESC) pour diriger la trajectoire et contrôler la vitesse.

## Contenu du dépôt

- `Codes_ScieWay/ESP_follow_gap.ino` : code Arduino/ESP32 qui reçoit des commandes série de l’ordinateur, contrôle un servo de direction, un ESC et mesure la vitesse via un capteur à effet Hall.
- `Codes_ScieWay/lidar_ld19.py` : script Python de lecture du LiDAR LD19, traitement des paquets, calcul du meilleur secteur de passage et envoi des commandes vers l’ESP32.

## Fonctionnement

1. Le LiDAR LD19 envoie des paquets série contenant plusieurs mesures de distance.
2. Le script Python analyse ces paquets, filtre et regroupe les points de mesure devant le robot.
3. Il identifie le meilleur « gap » libre dans une plage avant de ±90°.
4. Il envoie l’angle du gap et une consigne de vitesse à l’ESP32 via une liaison série.
5. L’ESP32 oriente le servo selon l’angle reçu et ajuste l’ESC grâce à un PID pour maintenir une vitesse cible.

## Installation et usage

1. Connecter le LiDAR LD19 à l’ordinateur sur le port série configuré dans `lidar_ld19.py` (`COM11` par défaut).
2. Connecter l’ESP32 à un autre port série (`COM7` par défaut) avec le servo, l’ESC et le capteur Hall correctement câblés.
3. Ouvrir le projet Arduino et téléverser `ESP_follow_gap.ino` sur l’ESP32.
4. Installer Python si nécessaire et les dépendances suivantes :
   - `pyserial`
5. Lancer le script Python :
   ```bash
   python Codes_ScieWay/lidar_ld19.py
   ```

> Ajuster `LIDAR_PORT` et `ESP32_PORT` dans `lidar_ld19.py` selon les ports COM réels de votre machine.

## Fichiers importants

- `ESP_follow_gap.ino`
  - Contrôle du servo de direction
  - Contrôle de l’ESC via PWM
  - Mesure de la vitesse moteur avec un capteur à effet Hall
  - Réception de commandes série au format `<angle_deg>,<rpm>`

- `lidar_ld19.py`
  - Lecture des paquets LD19
  - Vérification CRC
  - Sélection du secteur de passage le plus libre
  - Envoi des commandes à l’ESP32

## Notes techniques

- Le script Python dimensionne l’angle du gap par un facteur 0.75 pour limiter la course du servo.
- Le code ESP32 applique un PID sur la vitesse de rotation mesurée pour stabiliser le moteur.

## Améliorations possibles

- Interfaces visuelles pour visualiser le nuage de points LiDAR.
- Calibration automatique des angles et seuils de distance.
- Gestion des obstacles dynamiques et des trajectoires plus complexes.

## Auteur
BAHIR EL Mahdi - AKBIB BUKRABA Youssef - AZZAL Haitam
Projet réalisé dans le cadre du challenge Awake.
