function simulate_flight(times, c_opt, waypoints, config)
    % SIMULATE_FLIGHT Esegue la simulazione ad anello chiuso del quadrirotore
    
    %% --- 1. SETUP INIZIALE ---
    dt = config.sim.dt;
    t_end = times(end);
    n_pos = config.planner.n_pos;
    n_yaw = config.planner.n_yaw;
    
    % Inizializzazione dello stato reale (Struct)
    state_real.pos = waypoints(1, 1:3)'; % Parte dal primo waypoint
    state_real.vel = [0; 0; 0];
    
    % Matrice di rotazione iniziale basata sullo yaw del primo waypoint
    yaw0 = waypoints(1, 4);
    state_real.Rbw = [cos(yaw0), -sin(yaw0), 0; 
                      sin(yaw0),  cos(yaw0), 0; 
                              0,          0, 1]; % parte orizzontale
                              
    state_real.omega_BW = [0; 0; 0];
    
    % Pre-allocazione per salvare i dati per il plot e timeline interattiva
    time_history = 0:dt:t_end;
    N_steps = length(time_history);
    pos_history = zeros(3, N_steps);
    pos_des_history = zeros(3, N_steps);
    Rbw_history = zeros(3, 3, N_steps);
    
    % Pre-allocazione per i log di performance
    err_pos_history = zeros(3, N_steps);
    vel_real_history = zeros(1, N_steps); 
    vel_des_history = zeros(1, N_steps);

   %% --- 2. SETUP GRAFICA 3D ---
    h_fig_3d = figure('Name', 'Simulazione Volo 3D', 'Color', [0.1 0.1 0.1]); 
    h_ax_3d = axes('Parent', h_fig_3d, 'Position', [0.10, 0.13, 0.82, 0.80]);
    hold(h_ax_3d, 'on'); grid(h_ax_3d, 'on');
    set(h_ax_3d, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    view(h_ax_3d, 45, 30); daspect(h_ax_3d, [1 1 1]);
    
    % Limiti asse
    xlim([min(waypoints(:,1))-1, max(waypoints(:,1))+1]);
    ylim([min(waypoints(:,2))-1, max(waypoints(:,2))+1]);
    zlim([0, max(waypoints(:,3))+1]);
    
    % A. Disegna i Keyframes (Waypoints) - (w.o = white circles)
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w');

    % A2. Disegna eventuali Safe Corridors 3D (statici, costo zero nel loop)
    % Salviamo anche la geometria per il check di non-intersezione con l'hitbox
    corridors_geom = struct('seg_idx', {}, 'p1', {}, 'u_dir', {}, 'L_seg', {}, ...
                            'n1', {}, 'n2', {}, 'delta', {}, 't_start', {}, 't_end', {});
    if isfield(config, 'mission') && isfield(config.mission, 'corridor_delta') && ~isempty(config.mission.corridor_delta)
        deltas = config.mission.corridor_delta;
        for idx_c = 1:(size(waypoints, 1) - 1)
            d_c = deltas(idx_c);
            if ~isinf(d_c) && ~isnan(d_c)
                p1_c = waypoints(idx_c, 1:3)';
                p2_c = waypoints(idx_c+1, 1:3)';
                v_c = p2_c - p1_c;
                L_seg_c = norm(v_c);
                if L_seg_c > 1e-6
                    u_dir = v_c / L_seg_c;
                    if abs(u_dir(3)) < 0.9
                        ref_vec = [0; 0; 1];
                    else
                        ref_vec = [1; 0; 0];
                    end
                    n1 = cross(u_dir, ref_vec); n1 = n1 / norm(n1);
                    n2 = cross(u_dir, n1);
                    offsets = d_c * [n1 + n2, -n1 + n2, -n1 - n2, n1 - n2];
                    V_corr = [p1_c + offsets, p2_c + offsets];
                    F_corr = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
                    patch('Faces', F_corr, 'Vertices', V_corr', ...
                        'FaceColor', [1 0.85 0.1], 'FaceAlpha', 0.10, ...
                        'EdgeColor', [1 0.85 0.1], 'EdgeAlpha', 0.35, 'LineStyle', '--');

                    c_entry.seg_idx = idx_c;
                    c_entry.p1      = p1_c;
                    c_entry.u_dir   = u_dir;
                    c_entry.L_seg   = L_seg_c;
                    c_entry.n1      = n1;
                    c_entry.n2      = n2;
                    c_entry.delta   = d_c;
                    c_entry.t_start = times(idx_c);
                    c_entry.t_end   = times(idx_c + 1);
                    corridors_geom(end+1) = c_entry; %#ok<AGROW>
                end
            end
        end
    end
    
    %% ==========================================================
    % B. PRE-PLOT TRAIETTORIA DESIDERATA
    % ===========================================================
    % Generiamo una linea ad alta risoluzione da eval_traj prima del loop
    t_ref = linspace(0, t_end, 500);
    pos_ref = zeros(3, length(t_ref));
    
    for k = 1:length(t_ref)
        t = t_ref(k);
        % Valutiamo eval_traj separatamente per le 3 dimensioni spaziali
        [px, ~, ~, ~, ~] = eval_traj(c_opt.coeff_x, times, t, n_pos, config);
        [py, ~, ~, ~, ~] = eval_traj(c_opt.coeff_y, times, t, n_pos, config);
        [pz, ~, ~, ~, ~] = eval_traj(c_opt.coeff_z, times, t, n_pos, config);
        
        pos_ref(:, k) = [px; py; pz];
    end
    
    % Disegniamo la linea target fissa (w-- = white dashed)
    plot3(pos_ref(1,:), pos_ref(2,:), pos_ref(3,:), 'w--', 'LineWidth', 0.8, 'Color', [0.7 0.7 0.7]); % Grigio chiaro per non distrarre
    
    %% ==========================================================
    
    % C. Handle per il drone (centro + hitbox 3D), e la scia (aggiornati nel loop)
    h_drone = plot3(state_real.pos(1), state_real.pos(2), state_real.pos(3), 'cx', 'MarkerSize', 10, 'LineWidth', 3);
    h_scia = plot3(state_real.pos(1), state_real.pos(2), state_real.pos(3), 'c-', 'LineWidth', 1.5);

    % Hitbox 3D del drone: parallelepipedo a sezione quadrata orizzontale di diagonale 2*L
    L_arm = config.drone.L;
    half_side = L_arm / sqrt(2);
    half_h = max(0.03, 0.15 * L_arm);
    V_hitbox_body = [
        -half_side,  half_side,  half_side, -half_side, -half_side,  half_side,  half_side, -half_side;
        -half_side, -half_side,  half_side,  half_side, -half_side, -half_side,  half_side,  half_side;
        -half_h,    -half_h,    -half_h,    -half_h,     half_h,     half_h,     half_h,     half_h
    ];
    F_hitbox = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];
    V_hitbox_world = state_real.Rbw * V_hitbox_body + state_real.pos;
    h_hitbox = patch('Faces', F_hitbox, 'Vertices', V_hitbox_world', ...
        'FaceColor', [0 0.9 1], 'FaceAlpha', 0.25, ...
        'EdgeColor', 'c', 'LineWidth', 1.2);
    
    disp('Avvio Simulazione...');

    %% --- 3. LOOP DI SIMULAZIONE ---
    for i = 1:N_steps
        t = time_history(i);
        
        % A. VALUTAZIONE TRAIETTORIA (Estraiamo lo stato differenzialmente piatto)
        % Chiamiamo eval_traj per ogni dimensione separatamente
        [px, vx, ax, jx, ~] = eval_traj(c_opt.coeff_x, times, t, n_pos, config);
        [py, vy, ay, jy, ~] = eval_traj(c_opt.coeff_y, times, t, n_pos, config);
        [pz, vz, az, jz, ~] = eval_traj(c_opt.coeff_z, times, t, n_pos, config);
        [pyaw, vyaw, ~, ~, ~] = eval_traj(c_opt.coeff_psi, times, t, n_yaw, config);
        
        % Assembliamo la struct desiderata
        state_des.pos = [px; py; pz];
        state_des.vel = [vx; vy; vz];
        state_des.acc = [ax; ay; az];
        state_des.jerk = [jx; jy; jz];
        state_des.yaw = pyaw;
        state_des.yaw_dot = vyaw;
        
        % B. CONTROLLORE GEOMETRICO
        u_ideal = controller_selector(state_real, state_des, config, t);
        % FILTRO saturazione motori
        u_real = motor_mixing(u_ideal, config);
        
        % C. DINAMICA E INTEGRAZIONE
        state_dot = quadrotor_dynamics(state_real, u_real, config, t);
        
        % Step di Eulero
        state_real.pos      = state_real.pos + state_dot.pos * dt;
        state_real.vel      = state_real.vel + state_dot.vel * dt;
        state_real.omega_BW = state_real.omega_BW + state_dot.omega_BW * dt;
        
        % Aggiornamento Matrice di Rotazione (con ri-ortogonalizzazione
        % SVD) problema di Procruste Ortogonale
        state_real.Rbw = state_real.Rbw + state_dot.Rbw * dt;
        [U, ~, V] = svd(state_real.Rbw); % elimina le deformazioni di scala accumulatesi in Rbw
        state_real.Rbw = U * V'; % Garantisce che rimanga una pura rotazione ortogonale
        
        % Salvataggio scia e assetto
        pos_history(:, i) = state_real.pos;
        pos_des_history(:, i) = state_des.pos;
        Rbw_history(:, :, i) = state_real.Rbw;
        
        % Salvataggio dati per il Sanity Check
        err_pos_history(:, i) = state_real.pos - state_des.pos;
        vel_real_history(i) = norm(state_real.vel); % Modulo della velocità reale
        vel_des_history(i) = norm(state_des.vel);   % Modulo della velocità desiderata

        % D. AGGIORNAMENTO GRAFICO (a 20 FPS per non bloccare tutto)
        if mod(t, 0.05) < dt/2 
            set(h_drone, 'XData', state_real.pos(1), 'YData', state_real.pos(2), 'ZData', state_real.pos(3));
            set(h_scia, 'XData', pos_history(1, 1:i), 'YData', pos_history(2, 1:i), 'ZData', pos_history(3, 1:i));
            V_hitbox_world = state_real.Rbw * V_hitbox_body + state_real.pos;
            set(h_hitbox, 'Vertices', V_hitbox_world');
            drawnow;
        end
    end
    
    disp('Simulazione Terminata!');

    %% --- 4. CHECK DI NON-INTERSEZIONE HITBOX <-> SAFE CORRIDORS ---
    collision_mask = false(1, N_steps);
    max_penetration = 0;
    worst_step = 1;
    worst_seg = 0;
    if ~isempty(corridors_geom)
        for i = 1:N_steps
            t_i = time_history(i);
            V_world_i = Rbw_history(:, :, i) * V_hitbox_body + pos_history(:, i);
            for k_c = 1:length(corridors_geom)
                cg = corridors_geom(k_c);
                % Controlla durante la finestra temporale del segmento (con piccolo margine)
                if t_i >= (cg.t_start - 0.1) && t_i <= (cg.t_end + 0.1)
                    rel_V = V_world_i - cg.p1; % 3 x 8
                    s_proj = cg.u_dir' * rel_V; % 1 x 8
                    % Vertici effettivamente dentro la lunghezza longitudinale del corridoio
                    in_len = (s_proj >= 0) & (s_proj <= cg.L_seg);
                    if any(in_len)
                        d1 = abs(cg.n1' * rel_V(:, in_len));
                        d2 = abs(cg.n2' * rel_V(:, in_len));
                        pen = max(max(d1), max(d2)) - cg.delta;
                        if pen > 0
                            collision_mask(i) = true;
                            if pen > max_penetration
                                max_penetration = pen;
                                worst_step = i;
                                worst_seg = cg.seg_idx;
                            end
                        end
                    end
                end
            end
        end

        if any(collision_mask)
            n_coll = sum(collision_mask);
            fprintf(2, ' [FAILURE CORRIDOIO] Hitbox in collisione per %d step (%.3f s)! Sconfinamento max: %.3f m al seg. %d (t = %.2f s)\n', ...
                n_coll, n_coll * dt, max_penetration, worst_seg, time_history(worst_step));
            plot3(h_ax_3d, pos_history(1, worst_step), pos_history(2, worst_step), pos_history(3, worst_step), ...
                'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'Collisione Corridoio');
        else
            fprintf(' [SAFE CORRIDOR CHECK] PASSED: nessun contatto tra hitbox e pareti dei corridoi.\n');
        end
    end

    %% --- 5. BARRA TEMPORALE INTERATTIVA (SCRUBBING A QUALSIASI dt) ---
    h_time_lbl = uicontrol('Parent', h_fig_3d, 'Style', 'text', ...
        'String', sprintf('t = %.3f / %.2f s', time_history(end), t_end), ...
        'Units', 'normalized', 'Position', [0.78, 0.02, 0.20, 0.04], ...
        'BackgroundColor', [0.1 0.1 0.1], 'ForegroundColor', 'w', ...
        'FontSize', 9, 'HorizontalAlignment', 'left');

    uicontrol('Parent', h_fig_3d, 'Style', 'slider', ...
        'Min', 1, 'Max', N_steps, 'Value', N_steps, ...
        'SliderStep', [1/max(1, N_steps-1), min(0.05, 50/max(1, N_steps-1))], ...
        'Units', 'normalized', 'Position', [0.08, 0.02, 0.68, 0.04], ...
        'Callback', @(src, ~) update_timeline(src));

    function update_timeline(src)
        idx = max(1, min(N_steps, round(get(src, 'Value'))));
        set(h_drone, 'XData', pos_history(1, idx), 'YData', pos_history(2, idx), 'ZData', pos_history(3, idx));
        set(h_scia, 'XData', pos_history(1, 1:idx), 'YData', pos_history(2, 1:idx), 'ZData', pos_history(3, 1:idx));
        V_hw = Rbw_history(:, :, idx) * V_hitbox_body + pos_history(:, idx);
        set(h_hitbox, 'Vertices', V_hw');
        if collision_mask(idx)
            set(h_hitbox, 'FaceColor', [1 0.2 0.2], 'EdgeColor', 'r');
            set(h_time_lbl, 'String', sprintf('t = %.3f s [COLLISIONE]', time_history(idx)), 'ForegroundColor', [1 0.3 0.3]);
        else
            set(h_hitbox, 'FaceColor', [0 0.9 1], 'EdgeColor', 'c');
            set(h_time_lbl, 'String', sprintf('t = %.3f / %.2f s', time_history(idx), t_end), 'ForegroundColor', 'w');
        end
        drawnow limitrate;
    end
        %% --- PLOT PERFORMANCE ---
    figure('Name', 'Analisi Performance', 'Color', 'w', 'Position', [100, 100, 800, 300]);
    
    % Riquadro 1: Errore di Posizione (X, Y, Z)
    subplot(1, 2, 1);
    plot(time_history, err_pos_history(1,:), 'g--', 'LineWidth', 1.5); hold on;
    plot(time_history, err_pos_history(2,:), 'r--', 'LineWidth', 1.5);
    plot(time_history, err_pos_history(3,:), 'b--', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Error (m)');
    legend('x', 'y', 'z', 'Location', 'best');
    title('Position Error');
    
    % Riquadro 2: Tracking della Velocità
    subplot(1, 2, 2);
    plot(time_history, vel_des_history, 'b', 'LineWidth', 2); hold on;
    plot(time_history, vel_real_history, 'r', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)'); ylabel('Velocity (m/s)');
    legend('desired', 'actual', 'Location', 'best');
    title('Velocity Profile');


    metrics = compute_metrics(time_history, pos_history, pos_des_history, vel_real_history, vel_des_history, config);

end