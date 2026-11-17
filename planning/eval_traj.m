function [pos, vel, acc, jerk, snap] = eval_traj(coeff, times, t, n, config)
    % coeff: matrice [m x (n+1)] dei coefficienti calcolati (riga = segmento)
    % times: vettore colonna [M x 1] dei tempi dei keyframes [t0, t1, ..., tm]
    % t: istante di tempo attuale
    % n: ordine del polinomio
    
    times = times(:);
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

    use_scaling = config.planner.use_scaling;
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

    
    % 3. Valuta Posizione, Velocità e Accelerazione
    % Basi polinomiali per posizione, velocità e accelerazione
    basis_p = poly_basis(n, 0, t_eval, T_dur, use_scaling); % Posizione
    basis_v = poly_basis(n, 1, t_eval, T_dur, use_scaling); % Velocità
    basis_a = poly_basis(n, 2, t_eval, T_dur, use_scaling); % Accelerazione
    basis_j = poly_basis(n, 3, t_eval, T_dur, use_scaling); % Jerk
    basis_s = poly_basis(n, 4, t_eval, T_dur, use_scaling); % Snap

    c_seg = coeff(idx, :)';
    pos = basis_p * c_seg;
    vel = basis_v * c_seg;
    acc = basis_a * c_seg;
    jerk = basis_j * c_seg;
    snap = basis_s * c_seg;
end