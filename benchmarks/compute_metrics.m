function metrics = compute_metrics(time_history, pos_real_history, pos_des_history, vel_real_history, vel_des_history, config, motor_load_history)
    % COMPUTE_METRICS Calcola le metriche prestazionali di inseguimento e stabilità
    %
    % INPUT:
    %   time_history       - Vettore dei tempi di simulazione [1 x N]
    %   pos_real_history   - Matrice posizioni reali [3 x N]
    %   pos_des_history    - Matrice posizioni desiderate [3 x N]
    %   vel_real_history   - Matrice/vettore velocità reali [3 x N] o [1 x N]
    %   vel_des_history    - Matrice/vettore velocità desiderate [3 x N] o [1 x N]
    %   config             - Struct della configurazione di volo
    %   motor_load_history - (Opzionale) Percentuale di carico motori [4 x N] in [0, 100]
    %
    % OUTPUT:
    %   metrics            - Struct contenente RMSE, errori posizionali assoluti e saturazione motori

    N = length(time_history);
    
    % 1. Errori di Posizione 3D (e = p_real - p_des)
    err_pos_vec = pos_real_history - pos_des_history; % Matrice 3 x N
    err_pos_abs = abs(err_pos_vec);                   % Errore assoluto per asse [3 x N]
    err_pos_norm = sqrt(sum(err_pos_vec.^2, 1));      % Norma 3D errore posizionale assoluto [1 x N]
    
    % RMSE Posizione Totale e sui Singoli Assi (X, Y, Z)
    metrics.rmse_pos_total = sqrt(mean(err_pos_norm.^2));
    metrics.rmse_pos_x     = sqrt(mean(err_pos_vec(1, :).^2));
    metrics.rmse_pos_y     = sqrt(mean(err_pos_vec(2, :).^2));
    metrics.rmse_pos_z     = sqrt(mean(err_pos_vec(3, :).^2));

    % Errori Posizionali Assoluti (MAE, Max, P95, Finale)
    metrics.mae_pos_total   = mean(err_pos_norm);
    metrics.mae_pos_x       = mean(err_pos_abs(1, :));
    metrics.mae_pos_y       = mean(err_pos_abs(2, :));
    metrics.mae_pos_z       = mean(err_pos_abs(3, :));
    metrics.max_pos_error_x = max(err_pos_abs(1, :));
    metrics.max_pos_error_y = max(err_pos_abs(2, :));
    metrics.max_pos_error_z = max(err_pos_abs(3, :));
    sorted_err = sort(err_pos_norm);
    metrics.p95_pos_error   = sorted_err(max(1, ceil(0.95 * N)));
    metrics.final_pos_error = err_pos_norm(end);
    
    % Picco Massimo Errore di Posizione Assoluto 3D
    [metrics.max_pos_error, idx_max_pos] = max(err_pos_norm);
    metrics.time_max_pos_error = time_history(idx_max_pos);

    % 2. Errori di Velocità
    if size(vel_real_history, 1) == 3 && size(vel_des_history, 1) == 3
        err_vel_vec = vel_real_history - vel_des_history;
        err_vel_norm = sqrt(sum(err_vel_vec.^2, 1));
    else
        % Se sono già norme scalari 1 x N
        err_vel_norm = abs(vel_real_history - vel_des_history);
    end
    metrics.rmse_vel = sqrt(mean(err_vel_norm.^2));
    metrics.max_vel_error = max(err_vel_norm);

    % 3. Statistiche di Volo
    metrics.flight_duration = time_history(end) - time_history(1);
    
    % Calcolo Lunghezza Traiettoria Integrata (differenze di posizione)
    dpos = diff(pos_real_history, 1, 2);
    metrics.trajectory_length = sum(sqrt(sum(dpos.^2, 1)));

    % 4. Metriche Saturazione Motori (opzionale)
    has_motor_data = (nargin >= 7) && ~isempty(motor_load_history);
    if has_motor_data
        metrics.motor_load_mean_per_motor = mean(motor_load_history, 2);
        metrics.motor_load_max_per_motor  = max(motor_load_history, [], 2);
        metrics.motor_load_peak           = max(motor_load_history(:));
        sat_mask = any(motor_load_history >= 99.9 | motor_load_history <= 0.1, 1);
        dt_est = mean(diff(time_history));
        metrics.motor_saturation_time = sum(sat_mask) * dt_est;
        metrics.motor_saturation_pct  = 100 * mean(sat_mask);
    end

    % 5. Stampa Report Sintetico a Console
    fprintf('\n=======================================================\n');
    fprintf('           REPORT BENCHMARK PRESTAZIONI DI VOLO        \n');
    fprintf('=======================================================\n');
    if isfield(config, 'control') && isfield(config.control, 'type')
        fprintf('  Controllore Utilizzato:   %s\n', upper(config.control.type));
    end
    fprintf('  Durata Volo:              %.2f s\n', metrics.flight_duration);
    fprintf('  Lunghezza Traiettoria:    %.2f m\n', metrics.trajectory_length);
    fprintf('  -----------------------------------------------------\n');
    fprintf('  [ERRORE POSIZIONALE ASSOLUTO]\n');
    fprintf('  MAE Posizione 3D:         %.4f m (X: %.4fm, Y: %.4fm, Z: %.4fm)\n', ...
        metrics.mae_pos_total, metrics.mae_pos_x, metrics.mae_pos_y, metrics.mae_pos_z);
    fprintf('  RMSE Posizione 3D:        %.4f m (X: %.4fm, Y: %.4fm, Z: %.4fm)\n', ...
        metrics.rmse_pos_total, metrics.rmse_pos_x, metrics.rmse_pos_y, metrics.rmse_pos_z);
    fprintf('  Errore Assoluto Max 3D:   %.4f m (al tempo t = %.2fs)\n', ...
        metrics.max_pos_error, metrics.time_max_pos_error);
    fprintf('  Errore Assoluto Max Assi: X: %.4fm, Y: %.4fm, Z: %.4fm\n', ...
        metrics.max_pos_error_x, metrics.max_pos_error_y, metrics.max_pos_error_z);
    fprintf('  95° Percentile Errore 3D: %.4f m | Errore Finale: %.4f m\n', ...
        metrics.p95_pos_error, metrics.final_pos_error);
    fprintf('  -----------------------------------------------------\n');
    fprintf('  RMSE Velocità:            %.4f m/s (Max: %.4f m/s)\n', ...
        metrics.rmse_vel, metrics.max_vel_error);
    if has_motor_data
        fprintf('  -----------------------------------------------------\n');
        fprintf('  [SATURAZIONE MOTORI (%% CARICO)]\n');
        fprintf('  Carico Medio (M1..M4):    %.1f%% | %.1f%% | %.1f%% | %.1f%%\n', ...
            metrics.motor_load_mean_per_motor(1), metrics.motor_load_mean_per_motor(2), ...
            metrics.motor_load_mean_per_motor(3), metrics.motor_load_mean_per_motor(4));
        fprintf('  Carico Max   (M1..M4):    %.1f%% | %.1f%% | %.1f%% | %.1f%%\n', ...
            metrics.motor_load_max_per_motor(1), metrics.motor_load_max_per_motor(2), ...
            metrics.motor_load_max_per_motor(3), metrics.motor_load_max_per_motor(4));
        fprintf('  Tempo in Saturazione:     %.3f s (%.1f%% del volo)\n', ...
            metrics.motor_saturation_time, metrics.motor_saturation_pct);
    end
    fprintf('=======================================================\n\n');
end
