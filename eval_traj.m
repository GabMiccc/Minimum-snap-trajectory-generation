function [pos, vel, acc] = eval_traj(coeff, times, t, n, config)
    % coeff: matrice (n+1) x m dei coefficienti calcolati
    % times: vettore dei tempi dei keyframes [t0, t1, ..., tm]
    % t: istante di tempo attuale
    % n: ordine del polinomio
    
    m = length(times) - 1;
    
    % 1. Trova in quale segmento i si trova il tempo t
    if t >= times(end)
        idx = m; t_eval_raw = times(end);
    else
        idx = find(times <= t, 1, 'last');
        if isempty(idx) || idx > m, idx = m; end
        t_eval_raw = t;
    end
    % % % if t <= times(1)
    % % %     idx = 1;
    % % %     t_local = times(1);
    % % % elseif t >= times(end)
    % % %     idx = m;
    % % %     t_local = times(end);
    % % % else
    % % %     idx = find(times <= t, 1, 'last');
    % % %     if idx > m, idx = m; end
    % % %     t_local = t;
    % % % end
    
    % Recupera parametri segmento
    T_start = times(idx);
    T_end = times(idx+1);
    T_dur = T_end - T_start;

    use_temporal_scaling = isfield(config, 'use_temporal_scaling') && config.use_temporal_scaling;
    if use_temporal_scaling
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

    
    % 3. Valuta Posizione, Velocità e Accelerazione
    % Basi polinomiali per posizione, velocità e accelerazione
    basis_p = poly_basis(n, 0, t_eval, T_dur, use_temporal_scaling);
    basis_v = poly_basis(n, 1, t_eval, T_dur, use_temporal_scaling);
    basis_a = poly_basis(n, 2, t_eval, T_dur, use_temporal_scaling);

    pos = basis_p * coeff(:, idx);
    vel = basis_v * coeff(:, idx);
    acc = basis_a * coeff(:, idx);
end