function plot_trajectory(waypoints, times, c, config) 
    ts = linspace(times(1), times(end), 500); % time samples (discretization)
    pos_ref = zeros(length(ts), 4); % [x, y, z, yaw]
    
    for i = 1:length(ts) % discretized time steps
        t = ts(i);
        [pos_ref(i,1), ~, ~] = eval_traj(c.coeff_x, times, t, config.n_pos, config);
        [pos_ref(i,2), ~, ~] = eval_traj(c.coeff_y, times, t, config.n_pos, config);
        [pos_ref(i,3), ~, ~] = eval_traj(c.coeff_z, times, t, config.n_pos, config);
        [pos_ref(i,4), ~, ~] = eval_traj(c.coeff_psi, times, t, config.n_yaw, config);
    end
    
    m = length(times) - 1; % Numero di segmenti
    
    figure('Name', 'Traiettoria Iniziale (Uniform Times)');
    
    % --- Grafico 2D (Vista dall'alto X-Y) ---
    subplot(2,1,1); hold on;
    
    % DISEGNO CORRIDOI (X-Y)
    if isfield(config, 'corridor_delta')
        for i = 1:m
            d = config.corridor_delta(i);
            if ~isinf(d) && ~isnan(d)
                % Punti estremi del segmento
                p1 = waypoints(i, 1:2);
                p2 = waypoints(i+1, 1:2);
                
                % Vettore direzione e normale per l'offset
                v = p2 - p1;
                n = [-v(2), v(1)] / norm(v); % Normale al segmento
                
                % Calcolo rette del corridoio
                line_up = [p1 + n*d; p2 + n*d];
                line_down = [p1 - n*d; p2 - n*d];
                
                % Plot delle pareti (bianche tratteggiate)
                plot(line_up(:,1), line_up(:,2), 'w--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
                plot(line_down(:,1), line_down(:,2), 'w--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
                
                % Area colorata del corridoio
                patch([line_up(:,1); flipud(line_down(:,1))], ...
                      [line_up(:,2); flipud(line_down(:,2))], 'y', ...
                      'FaceAlpha', 0.05, 'EdgeColor', 'none', 'DisplayName', 'Safe Corridor');
            end
        end
    end
    
    % Tracciamento traiettoria e waypoints (Colori Dark Mode)
    plot(pos_ref(:,1), pos_ref(:,2), 'c-', 'LineWidth', 2, 'DisplayName', 'Traiettoria'); 
    plot(waypoints(:,1), waypoints(:,2), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w', 'DisplayName', 'Keyframes');
    
    grid on; xlabel('X (m)'); ylabel('Y (m)');
    title('Vista Piano X-Y (Minimum Snap)');
    legend('Location', 'best');
    axis equal;
    % Aggiungiamo un po' di margine "padding" attorno all'area di volo
    xlim([min(waypoints(:,1)) - 0.5, max(waypoints(:,1)) + 1.0]); 
    ylim([min(waypoints(:,2)) - 0.5, max(waypoints(:,2)) + 0.5]);
    daspect([1 1 1]); % Mantiene proporzioni corrette anche in 2D
    
    % --- Grafico 3D ---
    subplot(2,1,2); hold on;
    plot3(pos_ref(:,1), pos_ref(:,2), pos_ref(:,3), 'c-', 'LineWidth', 2); 
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Traiettoria 3D');
    view(45, 30);
    
    % Mantiene la proporzione fisica 1:1 tra X e Y, ma comprime Z
    daspect([1, 1, 2]); 
    
    % Riduce i limiti della "scatola" verticale attorno a Z=1
    zlim([0.5, 1.5]);   
end