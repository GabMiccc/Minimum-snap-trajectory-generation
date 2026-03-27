function simulate_flight(times, c_opt, waypoints, config)
    % SIMULATE_FLIGHT Esegue la simulazione ad anello chiuso del quadrirotore
    
    %% --- 1. SETUP INIZIALE ---
    dt = config.dt; % Passo di simulazione (100 Hz)
    t_end = times(end);
    
    % Inizializzazione dello stato reale (Struct)
    state_real.pos = waypoints(1, 1:3)'; % Parte dal primo waypoint
    state_real.vel = [0; 0; 0];
    
    % Matrice di rotazione iniziale basata sullo yaw del primo waypoint
    yaw0 = waypoints(1, 4);
    state_real.Rbw = [cos(yaw0), -sin(yaw0), 0; 
                      sin(yaw0),  cos(yaw0), 0; 
                              0,          0, 1];
                              
    state_real.omega_BW = [0; 0; 0];
    
    % Pre-allocazione per salvare i dati per il plot
    time_history = 0:dt:t_end;
    N_steps = length(time_history);
    pos_history = zeros(3, N_steps);
    
   %% --- 2. SETUP GRAFICA 3D ---
    figure('Name', 'Simulazione Volo 3D', 'Color', [0.1 0.1 0.1]); 
    hold on; grid on; set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    view(45, 30); daspect([1 1 1]);
    
    % Limiti asse
    xlim([min(waypoints(:,1))-1, max(waypoints(:,1))+1]);
    ylim([min(waypoints(:,2))-1, max(waypoints(:,2))+1]);
    zlim([0, max(waypoints(:,3))+1]);
    
    % A. Disegna i Keyframes (Waypoints) - (w.o = white circles)
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    
    %% ==========================================================
    % B. PRE-PLOT TRAIETTORIA DESIDERATA (LINEA TRATTEGGIATA BIANCA)
    % ===========================================================
    % Generiamo una linea ad alta risoluzione da eval_traj prima del loop
    fprintf('Generazione e pre-plot della traiettoria desiderata...\n');
    t_ref = linspace(0, t_end, 500);
    pos_ref = zeros(3, length(t_ref));
    
    for k = 1:length(t_ref)
        t = t_ref(k);
        % Valutiamo eval_traj separatamente per le 3 dimensioni spaziali
        [px, ~, ~, ~, ~] = eval_traj(c_opt.coeff_x, times, t, config.n_pos, config);
        [py, ~, ~, ~, ~] = eval_traj(c_opt.coeff_y, times, t, config.n_pos, config);
        [pz, ~, ~, ~, ~] = eval_traj(c_opt.coeff_z, times, t, config.n_pos, config);
        
        pos_ref(:, k) = [px; py; pz];
    end
    
    % Disegniamo la linea target fissa (w-- = white dashed)
    plot3(pos_ref(1,:), pos_ref(2,:), pos_ref(3,:), 'w--', 'LineWidth', 0.8, 'Color', [0.7 0.7 0.7]); % Grigio chiaro per non distrarre
    
    %% ==========================================================
    
    % C. Handle per il drone e la scia (aggiornati nel loop)
    h_drone = plot3(state_real.pos(1), state_real.pos(2), state_real.pos(3), 'cx', 'MarkerSize', 10, 'LineWidth', 3);
    h_scia = plot3(state_real.pos(1), state_real.pos(2), state_real.pos(3), 'c-', 'LineWidth', 1.5);
    
    disp('Avvio Simulazione...');

    %% --- 3. LOOP DI SIMULAZIONE ---
    for i = 1:N_steps
        t = time_history(i);
        
        % A. VALUTAZIONE TRAIETTORIA (Estraiamo lo stato differenzialmente piatto)
        % Chiamiamo eval_traj per ogni dimensione separatamente
        [px, vx, ax, jx, ~] = eval_traj(c_opt.coeff_x, times, t, config.n_pos, config);
        [py, vy, ay, jy, ~] = eval_traj(c_opt.coeff_y, times, t, config.n_pos, config);
        [pz, vz, az, jz, ~] = eval_traj(c_opt.coeff_z, times, t, config.n_pos, config);
        [pyaw, vyaw, ~, ~, ~] = eval_traj(c_opt.coeff_psi, times, t, config.n_yaw, config);
        
        % Assembliamo la struct desiderata
        state_des.pos = [px; py; pz];
        state_des.vel = [vx; vy; vz];
        state_des.acc = [ax; ay; az];
        state_des.jerk = [jx; jy; jz];
        state_des.yaw = pyaw;
        state_des.yaw_dot = vyaw;
        
        % B. CONTROLLORE GEOMETRICO
        u = geometric_controller(state_real, state_des, config);
        
        % C. DINAMICA E INTEGRAZIONE
        state_dot = quadrotor_dynamics(state_real, u, config);
        
        % Step di Eulero
        state_real.pos = state_real.pos + state_dot.pos * dt;
        state_real.vel = state_real.vel + state_dot.vel * dt;
        state_real.omega_BW = state_real.omega_BW + state_dot.omega_BW * dt;
        
        % Aggiornamento Matrice di Rotazione (con ri-ortogonalizzazione SVD)
        state_real.Rbw = state_real.Rbw + state_dot.Rbw * dt;
        [U, ~, V] = svd(state_real.Rbw);
        state_real.Rbw = U * V'; % Garantisce che rimanga una pura rotazione ortogonale
        
        % Salvataggio scia
        pos_history(:, i) = state_real.pos;
        
        % D. AGGIORNAMENTO GRAFICO (a 20 FPS per non bloccare tutto)
        if mod(t, 0.05) < dt/2 
            set(h_drone, 'XData', state_real.pos(1), 'YData', state_real.pos(2), 'ZData', state_real.pos(3));
            set(h_scia, 'XData', pos_history(1, 1:i), 'YData', pos_history(2, 1:i), 'ZData', pos_history(3, 1:i));
            drawnow;
        end
    end
    
    disp('Simulazione Terminata!');
end