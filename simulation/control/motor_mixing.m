function [u_real, motor_info] = motor_mixing(u_ideal, config)
    % MOTOR_MIXING Converte i comandi ideali [u1; tau_x; tau_y; tau_z] in RPM reali
    % applicando Prioritized Control Allocation (PCA) analitica in caso di saturazione.
    % Gerarchia di priorità: Roll/Pitch (tau_x, tau_y) > Spinta (u1) > Yaw (tau_z).
    
    % Estrazione parametri costruttivi
    d = config.drone;
    kF = d.kF; 
    kM = d.kM; 
    L  = d.L;  
    
    % Limiti operativi dei motori (rad/s)^2
    w_min_sq = d.w_min^2;
    w_max_sq = d.w_max^2;
    
    % Matrice di Mixing (Configurazione standard a '+')
    M = [ kF,    kF,    kF,    kF;
           0,  kF*L,     0, -kF*L;
        -kF*L,    0,  kF*L,     0;
          kM,   -kM,    kM,   -kM ];
          
    % 1. Fast path: nessuna saturazione -> nessuna allocazione necessaria
    omega_sq_ideal = M \ u_ideal;
    if all(omega_sq_ideal >= w_min_sq) && all(omega_sq_ideal <= w_max_sq)
        omega_sq_real = omega_sq_ideal;
        u_real = M * omega_sq_real;
        if nargout > 1
            span_avail = max(eps, w_max_sq - w_min_sq);
            motor_info.omega_sq_ideal = omega_sq_ideal;
            motor_info.omega_sq_real  = omega_sq_real;
            motor_info.load_pct_ideal = 100 * (omega_sq_ideal - w_min_sq) / span_avail;
            motor_info.load_pct_real  = 100 * (omega_sq_real  - w_min_sq) / span_avail;
            motor_info.is_saturated   = false;
        end
        return;
    end

    % 2. Decomposizione ortogonale delle richieste sui 4 rotori
    omega_sq_u1  = M \ [u_ideal(1); 0; 0; 0];
    omega_sq_rp  = M \ [0; u_ideal(2); u_ideal(3); 0];
    omega_sq_yaw = M \ [0; 0; 0; u_ideal(4)];

    % Step 1 (Priorità 3 - Yaw): riduciamo tau_z preservando u1 e (tau_x, tau_y)
    omega_sq_base = omega_sq_u1 + omega_sq_rp; % omega di spinta e omega di assetto
    if all(omega_sq_base >= w_min_sq) && all(omega_sq_base <= w_max_sq) % se le omeghe senza quella di direzione da sole non saturerebbero
        alpha_z = 1.0;
        for rotor = 1:4 % fisso un rotore i-esimo
            dz = omega_sq_yaw(rotor); 
            if dz > 0 % se lo yaw sta spingendo al limite superiore il motore si richia di violare w_max_sq
                alpha_z = min(alpha_z, (w_max_sq - omega_sq_base(rotor)) / dz);
            elseif dz < 0
                alpha_z = min(alpha_z, (w_min_sq - omega_sq_base(rotor)) / dz);
            end
        end % avendo sempre fatto min tra alpha e la "percentuale" troviamo il masssimo valore di alpha che mette tutti i motori in sicurezza
        alpha_z = max(0.0, alpha_z); % evito eroori numerici di infinitesimi negativi come successo in testing
        omega_sq_real = omega_sq_base + alpha_z * omega_sq_yaw;
    else % entriamo qui se si satura anche senza yaw
        % Step 2 (Priorità 2 - Spinta u1): azzeriamo yaw e adattiamo u1 preservando Roll/Pitch
        c_des = omega_sq_u1(1); % contributo uniforme su tutti e 4 i motori, ovviamente
        r_min = min(omega_sq_rp); % motore piu frenato per la omega di assetto
        r_max = max(omega_sq_rp); % motore piu spinto per la omega di assetto
        span_rp = r_max - r_min;            % escursione richiesta
        span_avail = w_max_sq - w_min_sq;   % escursione disponibile

        if span_rp <= span_avail % le coppie di assetto (pitch e roll) fisicamente accettabili , purchè la spinta uniforme c faccia sì che:
            % il motore piu basso non scenda sotto il min  : c + r_min > w_min^2
            % il motore piu alto non superi il max         : c + r_max < w_max^2
            c_real = min(w_max_sq - r_max, max(w_min_sq - r_min, c_des));  % clampa c_des tra [ w_min^2 - r_min, w_max^2 - r_max ]
            omega_sq_real = c_real + omega_sq_rp;
        else 
            % Step 3 (Caso estremo - Roll/Pitch saturano da soli senza yaw e spinta u1): scala proporzionalmente (tau_x, tau_y)
            alpha_rp = span_avail / span_rp;
            c_real = w_min_sq - alpha_rp * r_min; 
            omega_sq_real = c_real + alpha_rp * omega_sq_rp;
            % nota: questo aggiustamento fitta/scala lo span di omega
            % richiesto nello span disponibile Hardware del drone, portando
            % di fatto il motore più scarico a girare a w_min e quello più
            % carico a girare a w_max
        end
    end

    % Protezione numerica da arrotondamenti floating-point
    omega_sq_real = max(w_min_sq, min(w_max_sq, omega_sq_real));
    
    % 3. Ricalcolo delle forze e momenti effettivamente generati
    u_real = M * omega_sq_real;

    if nargout > 1
        span_avail = max(eps, w_max_sq - w_min_sq);
        motor_info.omega_sq_ideal = omega_sq_ideal;
        motor_info.omega_sq_real  = omega_sq_real;
        motor_info.load_pct_ideal = 100 * (omega_sq_ideal - w_min_sq) / span_avail;
        motor_info.load_pct_real  = 100 * (omega_sq_real  - w_min_sq) / span_avail;
        motor_info.is_saturated   = any(omega_sq_ideal < w_min_sq | omega_sq_ideal > w_max_sq);
    end
end