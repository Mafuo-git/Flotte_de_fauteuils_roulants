function running = isSimulationRunning(sim, clientID)
    % Vérifier l'état de la simulation via le signal 'simulationState'
    [returnCode, state] = sim.simxGetInt32Signal(clientID, 'simulationState', sim.simx_opmode_blocking);
    
    % Vérifier si le signal a été reçu correctement
    if returnCode == sim.simx_return_ok
        running = true;  % Simulation en cours (état = 1)
    else
        running = false;  % Si erreur de lecture, la simulation n'est pas en cours
    end
    
end
