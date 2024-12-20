function recup_data(sim, clientID, leftMotor, rightMotor, lidar,cam,numrobot)
    %récupérer les données du lidar
    [returnCode, lidarData] = sim.simxGetStringSignal(clientID, lidar, sim.simx_opmode_buffer);
    if returnCode == sim.simx_return_ok
    % Convertir la chaîne en tableau de flottants
        lidarPoints = sim.simxUnpackFloats(lidarData);
    else
        disp('Aucune donnée disponible pour le moment pour le robot '+ numrobot);
    end

    % Récupérer les données d'image
    [~, imageSignal] = sim.simxGetStringSignal(clientID, cam, sim.simx_opmode_buffer);
    if ~isempty(imageSignal)
        % Décompresser l'image
        image = sim.simxUnpackFloats(imageSignal);
        resolution = [640,480]; %résolution de la caméra à obtenir dans coppéliasim

        % Convertir en matrice de pixels
        imgMatrix = reshape(image, [3, resolution(1), resolution(2)]);  % Résolution (RGB, width, height)
        imgMatrix = permute(imgMatrix, [3, 2, 1]);  % Convertir en (height, width, RGB)

        % Corriger le retournement vertical
        imgMatrix = flipud(imgMatrix);
        imgMatrix = (imgMatrix + 1) / 2;
        
        %convertir l'angle pour qu'il soit centré correctement
        angle = detectArucoPositionAngle(imgMatrix,60);
        if(angle ~= -180)
            angle1 = int32(angle);
            angle2 = -180;
            if(angle -angle1 <= 0)
                angle2 = angle1 + 1;
            else
                angle2 = angle1 - 1;
            end
            angle2 = angle2*-1 +30;
            angle1 = angle1*-1 +30;
            dist1 = lidarPoints(angle1);
            dist2 = lidarPoints(angle2);
            dist = 0;
            anglefinal = 0;
            if(dist1 < dist2 && dist1 ~= 0)
                dist = dist1;
                anglefinal = angle1;
            else
                dist = dist2;
                anglefinal = angle2;
            end
            
            %caalcul de la vitesse des roues
            [vg,vr] = deplacement_robot(dist,anglefinal,5,12);

            %application de la vitesse aux moteurs de roues
            sim.simxSetJointTargetVelocity(clientID, leftMotor, vg, sim.simx_opmode_oneshot);
            sim.simxSetJointTargetVelocity(clientID, rightMotor, vr, sim.simx_opmode_oneshot);

        else
            disp("Pas d'angle car pas de marqueur pour le robot " + numrobot);
            sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot);
            sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot);

        end

    else
        disp('Aucune donnée disponible pour le moment pour le robot '+numrobot);
    end

end
