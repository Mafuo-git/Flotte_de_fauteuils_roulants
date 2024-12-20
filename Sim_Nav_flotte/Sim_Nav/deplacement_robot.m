function [v_gauche, v_droite] = deplacement_robot(Distance_Lidar, Direction_degrees, K_dir, K_dist)
    % Fonction pour calculer les vitesses des roues gauche et droite
    % Entrées :
    %   Distance_Lidar          : Distance actuelle vers l'objectif (en mètres)
    %   Direction_Balise_degrees: Angle de l'objectif en degrés (0 à 60°)
    %   K_dir                   : Gain proportionnel pour le contrôle de direction
    %   K_dist                  : Gain proportionnel pour le contrôle de distance
    % Sorties :
    %   v_gauche                : Vitesse de la roue gauche (en m/s)
    %   v_droite                : Vitesse de la roue droite (en m/s)

    % Étape 1 : Centrer l'angle autour de 30°
    angle_centered_degrees = Direction_degrees - 30;

    % Étape 2 : Convertir en radians pour les calculs
    Direction_obj = double(angle_centered_degrees) * pi / 180;

    % Calcul de l'erreur de direction (angle ajusté centré sur 0)
    erreur_direction = Direction_obj;

    % Calcul de l'erreur de distance
    erreur_distance = Distance_Lidar - 0.5; % Suivi à 0.5 mètres

    % Calcul de la vitesse linéaire Vx en fonction de l'erreur de distance
    Vx = K_dist * erreur_distance;

    % Calcul des vitesses des roues
    v_gauche = Vx - double(K_dir * erreur_direction);  % Roue gauche
    v_droite = Vx + double(K_dir * erreur_direction);  % Roue droite


    % Affichage des résultats pour debug 
    %fprintf('Distance: %.2f m, Direction: %.2f rad (%.2f°), Vx: %.2f m/s, Vg: %.2f m/s, Vd: %.2f m/s\n', ...
    %        Distance_Lidar, Direction_obj, Direction_degrees, Vx, v_gauche, v_droite);
end
