function controlRobot1()
    sim = remApi('remoteApi');
    sim.simxFinish(-1); % Fermer toutes les connexions ouvertes
    robotIndex = 1;
    clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
    if clientID > -1
        fprintf('Connexion établie avec le Robot %d\n', robotIndex);
        
        % Initialiser les handles moteurs
        [~, leftMotor] = sim.simxGetObjectHandle(clientID, 'moteurg', sim.simx_opmode_blocking);
        [~, rightMotor] = sim.simxGetObjectHandle(clientID, 'moteurd', sim.simx_opmode_blocking);

        % Initialiser la récupération des signaux
        sim.simxGetStringSignal(clientID, 'lidar_data1', sim.simx_opmode_streaming);
        sim.simxGetStringSignal(clientID, 'visionSensorImage1', sim.simx_opmode_streaming);
        
        pause(0.1);  % Pause initiale pour établir la communication

        % Boucle principale : continuer tant que la simulation est en cours
        while isSimulationRunning(sim, clientID)
            recup_data(sim,clientID,leftMotor,rightMotor,'lidar_data1','visionSensorImage1',1);
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
