function u = geometric_controller(state, state_des, config)
    % GEOMETRIC_CONTROLLER Calcola i comandi per i motori (Spinta e Momenti)
    % state     : struct attuale (pos, vel, Rbw, omega_BW)
    % state_des : struct desiderata (pos, vel, acc, yaw, yaw_dot) serve
    % anche jerk
    % config: struct parametri (m, g, Kp, Kv, KR, Komega)

    % --- Estrazione Parametri ---
    m = config.mass;
    g = config.g;
    e3 = [0; 0; 1]; % Versore Z del mondo
    
    % Matrici di guadagno, possono essere matrici del tipo scalare*I se il
    % drone reagisce con stessa forza su tutti gli assi
    Kp = config.Kp; Kv = config.Kv;
    KR = config.KR; Kw = config.Komega;

    %% ==========================================================
    % 1. CONTROLLO DI POSIZIONE (Calcolo Forza e Spinta u1)
    % ===========================================================
    
    % Errori di tracciamento traslazionale
    ep = state.pos - state_des.pos;
    ev = state.vel - state_des.vel;
    
    % Forza Desiderata (Eq. Mellinger: F_des = -Kp*ep - Kv*ev + mg*e3 + m*a_des)
    F_des = -Kp * ep - Kv * ev + m * g * e3 + m * state_des.acc;
    
    % Asse Z attuale del drone (Body frame Z proiettato nel World frame)
    z_B = state.Rbw * e3; 
    
    % Spinta Totale (Thrust u1)
    % Proiezione della forza desiderata sull'asse Z in cui il drone sta effettivamente puntando
    u1 = dot(F_des, z_B); 
    
    % Sicurezza: il drone non può generare spinta negativa (non può tirarsi giù)
    u1 = max(1e-3, u1); %u1 = max(0, u1);

    %% ==========================================================
    % 2. CALCOLO ASSETTO DESIDERATO (R_des)
    % matrice tale da avere
    %     R_des * e3 = direzione spinta desiderata
    % ===========================================================
    
    % Asse Z desiderato: allineato con la direzione della Forza Desiderata
    z_B_des = F_des / norm(F_des);
    
    % Direzione ausiliaria X_C basata sull'angolo di imbardata (yaw) desiderato
    x_C = [cos(state_des.yaw);
           sin(state_des.yaw);
                    0        ]; % yaw desiderato, direzione xc desiderata
    
    % Asse Y desiderato: ortogonale a Z_des e alla direzione di prua (prodotto vettoriale)
    y_B_des = cross(z_B_des, x_C);
    y_B_des = y_B_des / norm(y_B_des); % Normalizzazione necessaria
    
    % Asse X desiderato: completa la terna destrorsa ortonormale
    x_B_des = cross(y_B_des, z_B_des); %TODO: check che così esce già normalizzato
    
    % Matrice di Rotazione Desiderata
    R_des = [x_B_des, y_B_des, z_B_des];

    %% ==========================================================
    % 3. VERIFICA STABILITÀ ESPONENZIALE ---
    % Psi = 1/2 * trace(I - R_des^T * R)

    Psi = 0.5 * trace(eye(3) - R_des' * state.Rbw);
    if Psi >= 2.0
        warning('ATTENZIONE: Stabilità esponenziale persa! Errore di assetto > 180 gradi (Psi = %.2f)', Psi);
    elseif Psi >= 1.0
        warning(['ATTENZIONE: Stabilità esponenziale persa! Errore di assetto > 90 gradi (Psi = %.2f)' ...
            ' \n I guadagni lineari potrebbero non bastare a recuperare l assetto '], Psi);
    end

    %% ==========================================================
    % 4. CONTROLLO DI ASSETTO E MOMENTI
    % ===========================================================
    
    % Errore di Orientamento (eR)
    % La formula del paper usa l'operatore "vee" che estrae il vettore da una matrice emisimmetrica.
    % err_matrix = 1/2 * (R_des' * R_attuale - R_attuale' * R_des)
    err_matrix = 0.5 * (R_des' * state.Rbw - state.Rbw' * R_des);
    
    % Estrazione manuale (operatore vee) dei 3 componenti
    %  VEE MAP   [3x3 ASYM] :-> [3x1] in R3
    eR = [err_matrix(3,2);      % [ _ _ 2 ]
          err_matrix(1,3);      % [ 3 _ _ ]
          err_matrix(2,1)];     % [ _ 1 _ ]
    % questo è un vettore la cui direzione è l'asse attorno cui dovrebbe ruotare il drone per correggere la direzione
          
    % Velocità angolare desiderata  ( TODO: necessità di JERK input)
    p_des = -(m / u1) * dot(state_des.jerk, y_B_des);
    q_des =  (m / u1) * dot(state_des.jerk, x_B_des);
    r_des = state_des.yaw_dot * dot(e3, z_B_des);
     
    % Velocità angolare desiderata con l'utilizzo del vettore h_omega, come
    % da paper
    h_omega = ( m / u1) * ( state_des.jerk - (dot(z_B_des,state_des.jerk) * z_B_des)  );
    p_des_check = -1 * dot(h_omega,y_B_des);
    q_des_check =      dot(h_omega,x_B_des);
    
    if abs(p_des - p_des_check) > 1e-3
        warning('calcolo di omega diverso dal paper')
    end

    if abs(q_des - q_des_check) > 1e-3
        warning('calcolo di omega diverso dal paper')
    end

    omega_des = [p_des; q_des; r_des];

    % Errore di Velocità Angolare (ew)
    ew = state.omega_BW - (state.Rbw' * R_des * omega_des);
    
    %% CLOSING THE LOOP, calcolo dei momenti desiderati
    % Momenti Torcenti (Torques: u2, u3, u4)
    tau = -KR * eR - Kw * ew;

    %% ==========================================================
    % OUTPUT FINALE , servirà poi la conversione in comandi
    % ===========================================================
    u = [u1; tau];
    
end