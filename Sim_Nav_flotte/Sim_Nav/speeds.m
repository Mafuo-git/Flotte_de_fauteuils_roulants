% Initialiser la connexion avec CoppeliaSim
sim = remApi('remoteApi');
sim.simxFinish(-1); % Fermer toutes les connexions ouvertes
clientID = sim.simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID > -1
    disp('Connexion établie avec CoppeliaSim');
    
    % Obtenez les handles des objets
    [~, robot1_handle] = sim.simxGetObjectHandle(clientID, 'robot1', sim.simx_opmode_blocking);
    [~, robot2_handle] = sim.simxGetObjectHandle(clientID, 'robot2', sim.simx_opmode_blocking);
    [~, target_handle] = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking);
    
    % Initialiser les données de vitesse
    time = [];
    robot1_lin = []; %robot1_ang = [];
    robot2_lin = []; %robot2_ang = [];
    target_lin = []; %target_ang = [];
    
    % Initialiser le streaming des données
    sim.simxGetObjectVelocity(clientID, robot1_handle, sim.simx_opmode_streaming);
    sim.simxGetObjectVelocity(clientID, robot2_handle, sim.simx_opmode_streaming);
    sim.simxGetObjectVelocity(clientID, target_handle, sim.simx_opmode_streaming);
    
    % Configurer les figures
    figure(1); hold on; % Vitesse linéaire
    h1_lin = plot(0, 0, 'r', 'DisplayName', 'Robot1 Linéaire');
    h2_lin = plot(0, 0, 'g', 'DisplayName', 'Robot2 Linéaire');
    h3_lin = plot(0, 0, 'b', 'DisplayName', 'Target Linéaire');
    legend('show');
    xlabel('Temps (s)'); ylabel('Vitesse linéaire (m/s)');
    title('Vitesse linéaire');

    % figure(2); hold on; % Vitesse angulaire
    % h1_ang = plot(0, 0, 'r--', 'DisplayName', 'Robot1 Angulaire');
    % h2_ang = plot(0, 0, 'g--', 'DisplayName', 'Robot2 Angulaire');
    % h3_ang = plot(0, 0, 'b--', 'DisplayName', 'Target Angulaire');
    % legend('show');
    % xlabel('Temps (s)'); ylabel('Vitesse angulaire (rad/s)');
    % title('Vitesse angulaire');
    
    % Paramètres de la fenêtre glissante
    t_window = 30; % Fenêtre de 20 secondes
    t0 = tic; % Temps de début de simulation
    
    % Boucle principale
    while isSimulationRunning(sim,clientID) % Simulation sur 60 secondes (modifiable)
        currentTime = toc(t0);
        
        % Obtenez les vitesses
        [~, linVel1, angVel1] = sim.simxGetObjectVelocity(clientID, robot1_handle, sim.simx_opmode_buffer);
        [~, linVel2, angVel2] = sim.simxGetObjectVelocity(clientID, robot2_handle, sim.simx_opmode_buffer);
        [~, linVelT, angVelT] = sim.simxGetObjectVelocity(clientID, target_handle, sim.simx_opmode_buffer);
        
        % Stockez les données
        time = [time; currentTime];
        robot1_lin = [robot1_lin; norm(linVel1)];
        %robot1_ang = [robot1_ang; norm(angVel1)];
        robot2_lin = [robot2_lin; norm(linVel2)];
        %robot2_ang = [robot2_ang; norm(angVel2)];
        target_lin = [target_lin; norm(linVelT)];
        %target_ang = [target_ang; norm(angVelT)];
        
        % Définir les indices de la plage de temps
        if currentTime > t_window
            time_indices = time >= currentTime - t_window;
        else
            time_indices = true(size(time));
        end
        
        % Mettre à jour les courbes
        figure(1); % Vitesse linéaire
        set(h1_lin, 'XData', time(time_indices), 'YData', robot1_lin(time_indices));
        set(h2_lin, 'XData', time(time_indices), 'YData', robot2_lin(time_indices));
        set(h3_lin, 'XData', time(time_indices), 'YData', target_lin(time_indices));
        
        % figure(2); % Vitesse angulaire
        % set(h1_ang, 'XData', time(time_indices), 'YData', robot1_ang(time_indices));
        % set(h2_ang, 'XData', time(time_indices), 'YData', robot2_ang(time_indices));
        % set(h3_ang, 'XData', time(time_indices), 'YData', target_ang(time_indices));
        
        % Rafraîchir les figures
        drawnow;
    end
    
    % Fin de la simulation
    sim.simxFinish(clientID);
else
    disp('Impossible de se connecter à CoppeliaSim');
end

sim.delete();
