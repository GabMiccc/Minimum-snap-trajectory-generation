function is_feasible = check_feasibility(times, c_opt, config)
    % CHECK_FEASIBILITY Scansiona la traiettoria per prevenire saturazioni catastrofiche
    % è un controllo ad anello aperto, calcola l'accelerazione desiderata
    % dalla curva polinomiale, inverte la dinamica per trovare quale forza
    % verticale u1 realizza quella accelerazione e controlla che quella u1
    % rientra nei limiti di spinta dei motori

    % COSA NON VEDE?
    % - non vede il tempo necessaro per ruotare e portare la forza
    % verticale nella direzione desiderata
    % - non vede i momenti torcenti sui singoli motori, che potrebbero
    % saturare per generare momenti di inclinazione sul piano X-Y (momenti
    % di rollio e beccheggio)


    dt_check = 0.05; % Campionamento temporale per il controllo
    t_eval = times(1) : dt_check : times(end);
    is_feasible = true;
    
    % Estrazione parametri costruttivi
    kF = config.kF; 
    kM = config.kM; 
    L  = config.L;  

    % Matrice di Mixing (Configurazione standard a '+')
    M = [ kF,    kF,    kF,    kF;
           0,  kF*L,     0, -kF*L;
        -kF*L,    0,  kF*L,     0;
          kM,   -kM,    kM,   -kM ];

    % Calcolo dei limiti fisici di spinta del quadrirotore
    % La spinta totale massima è data da tutti e 4 i motori al massimo regime
    max_total_thrust = 4 * config.kF * config.w_max^2;
    min_total_thrust = 4 * config.kF * config.w_min^2;
    
    max_total_thrust = max_total_thrust * 0.90; % TODO: vedi se serve questo safe factor, per riservare

    fprintf('--- Analisi di Fattibilità Traiettoria ---\n');
    
    for i = 1:length(t_eval)
        t = t_eval(i);
        
        % Estrazione delle accelerazioni e dei jerk desiderate dai polinomi
        [~, ~, ax, jx, ~] = eval_traj(c_opt.coeff_x, times, t, config.n_pos, config);
        [~, ~, ay, jy, ~] = eval_traj(c_opt.coeff_y, times, t, config.n_pos, config);
        [~, ~, az,  ~, ~] = eval_traj(c_opt.coeff_z, times, t, config.n_pos, config);
        
        acc_des = [ax; ay; az];

        % Inversione della dinamica: Calcolo della forza totale richiesta
        % F_des = m * a_des + m * g * z_world
        F_des = config.mass * acc_des + [0; 0; config.mass * config.g];
        
        % La spinta propulsiva (u1) è la norma del vettore forza
        u1_req = norm(F_des);
        
        % stima momenti
        tau_x_est = config.mass * jy * L; % Stima momento rollio
        tau_y_est = -config.mass * jx * L; % Stima momento beccheggio

        % vettore completo degli input idealmente da realizzare 
        u_ideal = [u1_req; tau_x_est; tau_y_est; 0];
        
        % 3. Calcolo dei regimi dei 4 singoli motori
        omega_sq_req = M \ u_ideal; % sarebbe M^-1

        % Controllo saturazione spinta totale
        if u1_req > max_total_thrust
            fprintf('❌ ERRORE: Saturazione massima al tempo t=%.2fs. Richiesti %.1f N (Max: %.1f N)\n', t, u1_req, max_total_thrust);
            is_feasible = false;
            break; % Inutile continuare, la traiettoria fallirà
        elseif u1_req < min_total_thrust
            fprintf('❌ ERRORE: Saturazione minima (Free-fall) al tempo t=%.2fs.\n', t);
            is_feasible = false;
            break;
        end
        % Controllo saturazione singoli motori
        if any(omega_sq_req > config.w_max^2)
            fprintf('❌ ERRORE: Saturazione del singolo motore al tempo t=%.2fs!\n', t);
            is_feasible = false;
            break;
        end
    end
    
    if is_feasible
        fprintf('✅ Traiettoria fisicamente realizzabile. Spinta nei limiti.\n');
    end
    fprintf('------------------------------------------\n');
end