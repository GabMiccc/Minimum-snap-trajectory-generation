function is_feasible = check_feasibility_advanced(times, c_opt, config)
    % CHECK_FEASIBILITY_ADVANCED Controllo rigoroso di fattibilità fisica basato su Differential Flatness
    %
    % Calcola analiticamente la spinta totale u1 e i momenti torcenti esatti tau 
    % derivanti dall'inerzia del drone J e dalle accelerazioni angolari (Jerk/Snap),
    % invertendo la matrice di mixing per verificare la saturazione di ciascuno dei 4 motori.

    % nota: È un controllo a priori in anello aperto (condizioni nominali ideali):
    % non vede i contributi extra di spinta/coppia che il controllore in anello chiuso genererà
    % per correggere errori di inseguimento (-Kp*ep - Kv*ev, -KR*eR -
    % Kw*ew), nè tantomeno i disturbi.

    % nota2: rispetto a check_feasibility (legacy) usa l'inerzia J e la
    % differential flatness per le coppie. Inoltre fa il checksui singoli
    % motori per la saturazione invece che sulla spinta totale. Facendo
    % così risulta superfluo mettere un headroom, almeno per quanto
    % riguarda questo check a condizioni ideali. In questo modo però si
    % accettano traiettorie che in codnizioni ideali arrivano vicinissimi
    % al 100% di spinta di un singolo motore, e potrebbero esserci
    % situazioni di saturazione nell'entrare nella fisica reale (errori) +
    % disturbi

    dt_check = 0.02; % Campionamento fine (50 Hz) per catturare i picchi di Jerk/Coppia
    t_eval = times(1) : dt_check : times(end);
    N = length(t_eval);
    is_feasible = true;

    % Estrazione parametri costruttivi del velivolo
    d = config.drone;
    m = d.mass;
    g = d.g;
    J = d.J;
    e3 = [0; 0; 1];

    % Parametri costruttivi e Matrice di Mixing (Configurazione standard a '+')
    kF = d.kF; 
    kM = d.kM; 
    L  = d.L;  
    M = [ kF,    kF,    kF,    kF;
           0,  kF*L,     0, -kF*L;
        -kF*L,    0,  kF*L,     0;
          kM,   -kM,    kM,   -kM ];

    w_min = d.w_min;
    w_max = d.w_max;
    w_min_sq = w_min^2;
    w_max_sq = w_max^2;

    % Headroom sul singolo rotore (margine sulla capacità in omega^2 per feedback e disturbi)
    if isfield(config, 'planner') && isfield(config.planner, 'motor_headroom')
        headroom = config.planner.motor_headroom;
    else
        headroom = 0.05; % Default: 5% di margine
    end
    w_max_sq_eff = w_max_sq * (1 - headroom);
    w_min_sq_eff = w_min_sq * (1 + headroom);
    w_max_eff = sqrt(w_max_sq_eff);
    w_min_eff = sqrt(w_min_sq_eff);

    % Preallocazione serie temporali
    u1_arr     = zeros(1, N);
    omega_arr  = zeros(3, N);
    z_B_arr    = zeros(3, N);

    % --- FASE 1: Calcolo cinematico di Spinta (u1), Assi Corpo e Velocità Angolare (omega) ---
    for i = 1:N
        t = t_eval(i);

        % Estrazione ordini polinomi
        n_pos = config.planner.n_pos;
        n_yaw = config.planner.n_yaw;

        % Estrazione stati desiderati dai polinomi
        [~, ~, ax, jx, ~] = eval_traj(c_opt.coeff_x, times, t, n_pos, config);
        [~, ~, ay, jy, ~] = eval_traj(c_opt.coeff_y, times, t, n_pos, config);
        [~, ~, az, jz, ~] = eval_traj(c_opt.coeff_z, times, t, n_pos, config);
        [yaw, yaw_dot, ~, ~, ~] = eval_traj(c_opt.coeff_psi, times, t, n_yaw, config);

        acc_des  = [ax; ay; az];
        jerk_des = [jx; jy; jz];

        % Forza desiderata e Spinta totale
        F_des = m * acc_des + m * g * e3;
        u1 = norm(F_des);
        u1_arr(i) = u1;

        if u1 < 1e-3
            u1 = 1e-3;
        end

        % Asse Z del corpo
        z_B = F_des / u1;
        z_B_arr(:, i) = z_B;

        % Terna ortonormale del corpo con Yaw desiderato
        x_C = [cos(yaw); sin(yaw); 0];
        cross_zx = cross(z_B, x_C);
        norm_cross = norm(cross_zx);

        if norm_cross < 1e-4
            y_B = [0; 1; 0];
        else
            y_B = cross_zx / norm_cross;
        end
        x_B = cross(y_B, z_B);

        % Velocità angolari ideali da Differential Flatness (Eq. 7 Paper Mellinger)
        h_omega = (m / u1) * (jerk_des - dot(z_B, jerk_des) * z_B);
        p = -dot(h_omega, y_B);
        q =  dot(h_omega, x_B);
        r =  yaw_dot * dot(e3, z_B);

        omega_arr(:, i) = [p; q; r];
    end

    % --- FASE 2: Calcolo Accelerazioni Angolari e Verifica Dinamica di Coppia ---
    % Derivata temporale di omega tramite differenze finite centrate
    d_omega_arr = zeros(3, N);
    for i = 2:N-1
        d_omega_arr(:, i) = (omega_arr(:, i+1) - omega_arr(:, i-1)) / (2 * dt_check);
    end
    d_omega_arr(:, 1) = (omega_arr(:, 2) - omega_arr(:, 1)) / dt_check;
    d_omega_arr(:, N) = (omega_arr(:, N) - omega_arr(:, N-1)) / dt_check;

    fprintf('--- Analisi di Fattibilità Avanzata (Differential Flatness) ---\n');

    max_rpm_seen = 0;
    worst_time = 0;
    worst_motor = 0;

    for i = 1:N
        t = t_eval(i);
        u1 = u1_arr(i);
        w_des = omega_arr(:, i);
        dw_des = d_omega_arr(:, i);

        % Vera coppia di Newton-Eulero richiesta (Inerzia + Effetto Giroscopico)
        tau = J * dw_des + cross(w_des, J * w_des);

        % Input ideali
        u_ideal = [u1; tau];

        % Inversione della matrice di mixing per i 4 motori
        omega_sq = M \ u_ideal;

        % Monitoraggio del picco massimo
        max_sq = max(omega_sq);
        if max_sq > max_rpm_seen^2
            max_rpm_seen = sqrt(max(0, max_sq));
            worst_time = t;
            [~, worst_motor] = max(omega_sq);
        end

        % Verifica Saturazione Massima di ciascun singolo motore (con headroom)
        if any(omega_sq > w_max_sq_eff)
            fprintf('❌ ERRORE al tempo t=%.2fs: Motore %d richiede %.1f rad/s (Limite con headroom %.0f%%: %.1f rad/s, Max fisico: %.1f rad/s)!\n', ...
                    t, worst_motor, sqrt(max_sq), headroom*100, w_max_eff, w_max);
            fprintf('   -> Coppia richiesta: Roll=%.2f Nm, Pitch=%.2f Nm, Yaw=%.2f Nm\n', ...
                    tau(1), tau(2), tau(3));
            is_feasible = false;
            break;
        end

        % Verifica Saturazione Minima (Idle / Spegnimento con headroom)
        if any(omega_sq < w_min_sq_eff)
            fprintf('❌ ERRORE al tempo t=%.2fs: Rischio spegnimento motore (richiesti < %.1f rad/s, Min fisico: %.1f rad/s)!\n', ...
                    t, w_min_eff, w_min);
            is_feasible = false;
            break;
        end
    end

    if is_feasible
        fprintf('✅ Traiettoria fisicamente realizzabile! Picco motore: %.1f rad/s (limite con headroom %.0f%%: %.1f rad/s, max fisico: %.1f rad/s).\n', ...
                max_rpm_seen, headroom*100, w_max_eff, w_max);
    else
        fprintf('⚠️  Traiettoria NON realizzabile: richiesta coppia e RPM oltre le capacità fisiche dei motori.\n');
    end
    fprintf('----------------------------------------------------------------\n');
end
