function controlRobot2()
    sim = remApi('remoteApi');
    sim.simxFinish(-1); % Fermer toutes les connexions ouvertes
    robotIndex = 2;
    clientID = sim.simxStart('127.0.0.1', 19998, true, true, 5000, 5);
    if clientID > -1
        fprintf('Connexion établie avec le Robot %d\n', robotIndex);
        
        % Initialiser les handles moteurs
        [~, leftMotor] = sim.simxGetObjectHandle(clientID, 'moteurg2', sim.simx_opmode_blocking);
        [~, rightMotor] = sim.simxGetObjectHandle(clientID, 'moteurd2', sim.simx_opmode_blocking);

        % Initialiser la récupération des signaux
        sim.simxGetStringSignal(clientID, 'lidar_data2', sim.simx_opmode_streaming);
        sim.simxGetStringSignal(clientID, 'visionSensorImage2', sim.simx_opmode_streaming);
        
        pause(0.1);  % Pause initiale pour établir la communication

        % Boucle principale : continuer tant que la simulation est en cours
        while isSimulationRunning(sim, clientID)
            recup_data(sim,clientID,leftMotor,rightMotor,'lidar_data2','visionSensorImage2',2);
            pause(0.1);  % Pause entre les lectures
        end

        fprintf('Simulation terminée pour le Robot %d\n', robotIndex);
        % Arrêter les moteurs à la fin de la simulation
        sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot);
        sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot);

        % Fermer la connexion proprement
        sim.simxFinish(clientID);
    else
        fprintf('Erreur de connexion pour le Robot %d\n', robotIndex);
    end
end
