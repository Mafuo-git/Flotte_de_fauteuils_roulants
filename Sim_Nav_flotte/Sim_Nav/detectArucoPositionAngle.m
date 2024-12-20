function angle = detectArucoPositionAngle(inputImage, cameraFOV)
    % detectArucoPositionAngle - Détecte un marqueur ArUco et retourne son angle
    %                           par rapport à l'axe central de la caméra
    %
    % Syntax: angle = detectArucoPositionAngle(inputImage, cameraFOV)
    %
    % Inputs:
    %   inputImage - Image en couleur (RGB) ou en niveaux de gris contenant un marqueur ArUco
    %   cameraFOV  - Champ de vision horizontal de la caméra en degrés
    %
    % Outputs:
    %   angle - Angle (en degrés) indiquant si le marqueur est à gauche (-) ou à droite (+)

    % Détecter les marqueurs ArUco dans l'image
    [ids, corners, pose] = readArucoMarker(inputImage, ...
                                           'MarkerSizeRange', [0.01, 0.8], ... %tolérance sur la taille du marqueur (en m)
                                           'SquarenessTolerance', 0.1); %tolérance sur la déformation du marqueur (en %)

    % Vérifier si des marqueurs ont été détectés
    if isempty(ids)
        angle = -180; %si pas de marqueur trouvé, angle =-180 <=> valeur inatteignable
    else
        
        detectedCorners = corners;
    
        % Calculer le centre du marqueur
        markerCenter = mean(detectedCorners, 1); % Moyenne des coordonnées des coins
    
        % Obtenir les dimensions de l'image
        [imgHeight, imgWidth, ~] = size(inputImage);
    
        % Calculer la position horizontale du centre par rapport à l'axe de l'image
        centerOffset = markerCenter(1) - imgWidth / 2;
    
        % Convertir cette position en angle en utilisant le champ de vision de la caméra
        angle = (centerOffset / (imgWidth / 2)) * (cameraFOV / 2);
    
        % Afficher les résultats
        % figure(1);
        % imshow(inputImage);
        % hold on;
        % plot(detectedCorners(:, 1), detectedCorners(:, 2), 'r-', 'LineWidth', 2); % Contour du marqueur
        % plot(markerCenter(1), markerCenter(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Centre du marqueur
        % title(sprintf('Angle de position du marqueur : %.2f°', angle));
        % hold off;
    end
end
