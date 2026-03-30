function is_feasible = check_feasibility(times, c_opt, config)
    % CHECK_FEASIBILITY Scansiona la traiettoria per prevenire saturazioni catastrofiche
    
    dt_check = 0.05; % Campionamento temporale per il controllo
    t_eval = times(1) : dt_check : times(end);
    is_feasible = true;
    
    % Calcolo dei limiti fisici di spinta del quadrirotore
    % La spinta totale massima è data da tutti e 4 i motori al massimo regime
    max_total_thrust = 4 * config.kF * config.w_max^2;
    min_total_thrust = 4 * config.kF * config.w_min^2;
    
    fprintf('--- Analisi di Fattibilità Traiettoria ---\n');
    
    for i = 1:length(t_eval)
        t = t_eval(i);
        
        % Estrazione delle accelerazioni desiderate dai polinomi
        [~, ~, ax, ~, ~] = eval_traj(c_opt.coeff_x, times, t, config.n_pos, config);
        [~, ~, ay, ~, ~] = eval_traj(c_opt.coeff_y, times, t, config.n_pos, config);
        [~, ~, az, ~, ~] = eval_traj(c_opt.coeff_z, times, t, config.n_pos, config);
        
        acc_des = [ax; ay; az];
        
        % Inversione della dinamica: Calcolo della forza totale richiesta
        % F_des = m * a_des + m * g * z_world
        F_des = config.mass * acc_des + [0; 0; config.mass * config.g];
        
        % La spinta propulsiva (u1) è la norma del vettore forza
        u1_req = norm(F_des);
        
        % Controllo saturazione
        if u1_req > max_total_thrust
            fprintf('❌ ERRORE: Saturazione massima al tempo t=%.2fs. Richiesti %.1f N (Max: %.1f N)\n', t, u1_req, max_total_thrust);
            is_feasible = false;
            break; % Inutile continuare, la traiettoria fallirà
        elseif u1_req < min_total_thrust
            fprintf('❌ ERRORE: Saturazione minima (Free-fall) al tempo t=%.2fs.\n', t);
            is_feasible = false;
            break;
        end
    end
    
    if is_feasible
        fprintf('✅ Traiettoria fisicamente realizzabile. Spinta nei limiti.\n');
    end
    fprintf('------------------------------------------\n');
end