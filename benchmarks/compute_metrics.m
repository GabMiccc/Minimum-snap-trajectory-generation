function metrics = compute_metrics(time_history, pos_real_history, pos_des_history, vel_real_history, vel_des_history, config)
    % COMPUTE_METRICS Calcola le metriche prestazionali di inseguimento e stabilità
    %
    % INPUT:
    %   time_history     - Vettore dei tempi di simulazione [1 x N]
    %   pos_real_history - Matrice posizioni reali [3 x N]
    %   pos_des_history  - Matrice posizioni desiderate [3 x N]
    %   vel_real_history - Matrice/vettore velocità reali [3 x N] o [1 x N]
    %   vel_des_history  - Matrice/vettore velocità desiderate [3 x N] o [1 x N]
    %   config           - Struct della configurazione di volo
    %
    % OUTPUT:
    %   metrics          - Struct contenente RMSE, picchi d'errore e metriche di volo

    N = length(time_history);
    
    % 1. Errori di Posizione 3D
    err_pos_vec = pos_real_history - pos_des_history; % Matrice 3 x N
    err_pos_norm = sqrt(sum(err_pos_vec.^2, 1));       % Vettore 1 x N norma 3D
    
    % RMSE Posizione Totale e sui Singoli Assi (X, Y, Z)
    metrics.rmse_pos_total = sqrt(mean(err_pos_norm.^2));
    metrics.rmse_pos_x     = sqrt(mean(err_pos_vec(1, :).^2));
    metrics.rmse_pos_y     = sqrt(mean(err_pos_vec(2, :).^2));
    metrics.rmse_pos_z     = sqrt(mean(err_pos_vec(3, :).^2));
    
    % Picco Massimo Errore di Posizione
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
    
    % Calcolo Lunghezza Traiettoria Perchè integrata (differenze di posizione)
    dpos = diff(pos_real_history, 1, 2);
    metrics.trajectory_length = sum(sqrt(sum(dpos.^2, 1)));

    % 4. Stampa Report Sintetico a Console
    fprintf('\n=======================================================\n');
    fprintf('           REPORT BENCHMARK PRESTAZIONI DI VOLO        \n');
    fprintf('=======================================================\n');
    if isfield(config, 'controller_type')
        fprintf('  Controllore Utilizzato:   %s\n', upper(config.controller_type));
    end
    fprintf('  Durata Volo:              %.2f s\n', metrics.flight_duration);
    fprintf('  Lunghezza Traiettoria:    %.2f m\n', metrics.trajectory_length);
    fprintf('  -----------------------------------------------------\n');
    fprintf('  RMSE Posizione Totale:    %.4f m (X: %.4fm, Y: %.4fm, Z: %.4fm)\n', ...
        metrics.rmse_pos_total, metrics.rmse_pos_x, metrics.rmse_pos_y, metrics.rmse_pos_z);
    fprintf('  Errore di Posizione Max:  %.4f m (al tempo t = %.2fs)\n', ...
        metrics.max_pos_error, metrics.time_max_pos_error);
    fprintf('  RMSE Velocità:            %.4f m/s\n', metrics.rmse_vel);
    fprintf('=======================================================\n\n');
end
