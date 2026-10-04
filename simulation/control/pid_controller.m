function u = pid_controller(state, state_des, config, t)
    % PID_CONTROLLER Controllore PID Classico basato su Angoli di Eulero
    %
    % INPUT:
    %   state     - Struct stato attuale reale (pos, vel, Rbw, omega_BW)
    %   state_des - Struct stato desiderato (pos, vel, acc, jerk, yaw, yaw_dot)
    %   config    - Struct parametri di configurazione (m, g, Kp, Kv, ecc.)
    %   t         - Istante di tempo attuale
    %
    % OUTPUT:
    %   u         - Vettore 4x1 comandi ideali [u1; tau_x; tau_y; tau_z]

    m = config.mass;
    g = config.g;
    e3 = [0; 0; 1];
    
    % Variabili persistenti per l'integrale dell'errore (anti-windup)
    persistent int_ep int_e_att last_t;
    if isempty(last_t) || t < last_t
        int_ep = [0; 0; 0];
        int_e_att = [0; 0; 0];
        last_t = t;
    end
    dt = max(1e-4, t - last_t);
    last_t = t;

    %% 1. OUTER LOOP (Posizione - PID)
    ep = state.pos - state_des.pos;
    ev = state.vel - state_des.vel;
    
    % Aggiornamento Integrale Posizione con Anti-Windup
    int_ep = int_ep + ep * dt;
    int_ep = max(-2.0, min(2.0, int_ep)); % Clamping anti-windup
    
    % Guadagni Posizione (derivati da config)
    Kp_pos = config.Kp;
    Kv_pos = config.Kv;
    Ki_pos = Kp_pos * 0.05; % Componente Integrale di posizione
    
    % Forza Desiderata
    F_des = -Kp_pos * ep - Kv_pos * ev - Ki_pos * int_ep + m * g * e3 + m * state_des.acc;
    
    % Tilt Limit (Anti-Ribaltamento)
    fz = F_des(3);
    fxy_norm = norm(F_des(1:2));
    if isfield(config, 'max_tilt_angle_deg')
        max_tilt = deg2rad(config.max_tilt_angle_deg);
    else
        max_tilt = deg2rad(45);
    end
    max_fxy = fz * tan(max_tilt);
    if fxy_norm > max_fxy && fz > 0
        F_des(1:2) = F_des(1:2) * (max_fxy / fxy_norm);
    end
    
    % Spinta Totale u1
    z_B = state.Rbw * e3;
    u1 = max(1e-3, dot(F_des, z_B));

    %% 2. CONVERSIONE DA FORZA DESIDERATA AD ANGOLI DI EULERO DESIDERATI
    % Estrazione Yaw desiderato
    psi_des = state_des.yaw;
    
    % Approssimazione lineare degli angoli di Roll (phi) e Pitch (theta) desiderati
    u_acc = F_des / m;
    phi_des   = (u_acc(1) * sin(psi_des) - u_acc(2) * cos(psi_des)) / g;
    theta_des = (u_acc(1) * cos(psi_des) + u_acc(2) * sin(psi_des)) / g;
    
    % Clamping sugli angoli desiderati
    phi_des   = max(-max_tilt, min(max_tilt, phi_des));
    theta_des = max(-max_tilt, min(max_tilt, theta_des));

    %% 3. ESTRAZIONE ANGOLI DI EULERO REALI DALLA MATRICE R_BW
    R = state.Rbw;
    phi_real   = atan2(R(3,2), R(3,3));
    theta_real = asin(max(-1, min(1, -R(3,1))));
    psi_real   = atan2(R(2,1), R(1,1));
    
    %% 4. INNER LOOP (Assetto PID sugli Angoli di Eulero)
    e_phi   = phi_real - phi_des;
    e_theta = theta_real - theta_des;
    
    % Gestione discontinuità Yaw (-pi a +pi)
    e_psi = psi_real - psi_des;
    e_psi = atan2(sin(e_psi), cos(e_psi));
    
    e_att = [e_phi; e_theta; e_psi];
    
    % Integrale errore assetto con Anti-Windup
    int_e_att = int_e_att + e_att * dt;
    int_e_att = max(-0.5, min(0.5, int_e_att));
    
    % Velocità angolari reali (p, q, r)
    w_real = state.omega_BW;
    w_des  = [0; 0; state_des.yaw_dot];
    ew_att = w_real - w_des;
    
    % Guadagni PID Assetto (derivati da KR e Komega di config)
    KR = config.KR;
    Kw = config.Komega;
    Ki_att = KR * 0.05;
    
    % Calcolo dei Momenti Torcenti (tau_x, tau_y, tau_z)
    tau = -KR * e_att - Kw * ew_att - Ki_att * int_e_att;

    u = [u1; tau];
end
