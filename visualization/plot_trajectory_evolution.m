function plot_trajectory_evolution(waypoints, times_init, c_init, times_final, c_final, config)
    % PLOT_TRAJECTORY_EVOLUTION Sovrappone la traiettoria prima e dopo l'ottimizzazione
    
    dt = 0.05; % Risoluzione del campionamento
    M = size(waypoints, 1);
    m = M - 1;
    
    % 1. Campionamento Traiettoria Iniziale
    t_query_init = times_init(1):dt:times_init(end);
    pos_init = zeros(length(t_query_init), 3);
    for i = 1:length(t_query_init)
        pos_init(i, :) = sample_pos(t_query_init(i), times_init, c_init, config);
    end
    
    % 2. Campionamento Traiettoria Finale (Ottimizzata)
    t_query_final = times_final(1):dt:times_final(end);
    pos_final = zeros(length(t_query_final), 3);
    for i = 1:length(t_query_final)
        pos_final(i, :) = sample_pos(t_query_final(i), times_final, c_final, config);
    end
    
    % 2. Grafica
    figure('Name', 'Analisi Traiettoria e Corridoi', 'Position', [100, 100, 1200, 600]);
    
    % --- VISTA 2D (X-Y) ---
    subplot(1, 2, 1); hold on;
    
    % Disegno Corridoi se presenti
    if isfield(config, 'corridor_delta')
        for i = 1:m
            d = config.corridor_delta(i);
            if ~isinf(d)
                % Punti estremi del segmento
                p1 = waypoints(i, 1:2);
                p2 = waypoints(i+1, 1:2);
                
                % Vettore direzione e normale per l'offset
                v = p2 - p1;
                n = [-v(2), v(1)] / norm(v); % Normale al segmento
                
                % Calcolo rette del corridoio
                line_up = [p1 + n*d; p2 + n*d];
                line_down = [p1 - n*d; p2 - n*d];
                
                % Plot delle pareti (grigie tratteggiate)
                plot(line_up(:,1), line_up(:,2), 'w--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
                plot(line_down(:,1), line_down(:,2), 'w--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
                
                % Opzionale: coloriamo leggermente l'area del corridoio
                patch([line_up(:,1); flipud(line_down(:,1))], ...
                      [line_up(:,2); flipud(line_down(:,2))], 'y', ...
                      'FaceAlpha', 0.05, 'EdgeColor', 'none', 'DisplayName', 'Safe Corridor');
            end
        end
    end

    plot(pos_init(:,1), pos_init(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', 'Init (Uniform)');
    plot(pos_final(:,1), pos_final(:,2), 'c-', 'LineWidth', 2, 'DisplayName', 'Optimized');
    plot(waypoints(:,1), waypoints(:,2), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w', 'DisplayName', 'Keyframes');
    
    grid on; xlabel('X (m)'); ylabel('Y (m)'); title('Vista Piano X-Y');
    legend('Location', 'best'); daspect([1 1 1]);
    
    % --- VISTA 3D ---
    subplot(1, 2, 2); hold on;
    plot3(pos_init(:,1), pos_init(:,2), pos_init(:,3), 'r--', 'LineWidth', 1.2);
    plot3(pos_final(:,1), pos_final(:,2), pos_final(:,3), 'c-', 'LineWidth', 2);
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'wo', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    
    grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Traiettoria 3D');
    daspect([1 1 2]); zlim([0.5, 1.5]); view(45, 30);
end

%% --- FUNZIONI HELPER LOCALI ---
function pos = sample_pos(t, times, c, config)
    % Trova il segmento corretto e valuta il polinomio per x, y, z
    m = length(times) - 1;
    if t >= times(end)
        idx = m; t_eval_raw = times(end); % raw perchè non è ancora adimensionalizzato
    else
        idx = find(times <= t, 1, 'last');
        if isempty(idx) || idx > m, idx = m; end
        t_eval_raw = t;
    end

    % Recupera parametri segmento
    T_start = times(idx);
    T_end = times(idx+1);
    T_dur = T_end - T_start;
    
    use_scaling = isfield(config, 'use_scaling') && config.use_scaling;
    if use_scaling
        % Se adimensionalizzato, mappa il tempo nell'intervallo [0, 1]
        if t >= times(end)
            t_eval = 1;
        else
            t_eval = (t_eval_raw - T_start) / T_dur;
        end
    else
        % Altrimenti usa il tempo assoluto
        t_eval = t_eval_raw;
    end

    basis = poly_basis(config.n_pos, 0, t_eval, T_dur, use_scaling);
    pos(1) = basis * c.coeff_x(:, idx);
    pos(2) = basis * c.coeff_y(:, idx);
    pos(3) = basis * c.coeff_z(:, idx);
end