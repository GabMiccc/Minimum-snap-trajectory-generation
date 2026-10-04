function state_dot = quadrotor_dynamics(state, u, config)
    % PLANT/ MOTORE FISICO: prende lo stato attuale (posizione, velocità,
    % angoli, ratei angolari) e i comandi dei motori (velocità delle 4 
    % eliche) e calcola le derivate dello stato.
    % Fatto scrivendo le equazioni di Newton-Eulero, 

    %  Calcola la derivata dello stato del drone, che servirà a ottenere lo
    %  stato al tempo t+dt moltiplicando le derivate per il dt
    
    % state: struct con campi pos, vel, Rbw, omega_BW
    % u: vettore colonna 4x1 [u1; u2; u3; u4] (Spinta e Momenti)
    % config: struct con i parametri fisici (m, g, J)

    % --- Estrazione dei Parametri Fisici ---
    m = config.mass;
    g = config.g;
    J = config.J; % Matrice d'inerzia 3x3

    % --- Decomposizione degli Input di Controllo ---
    u1 = u(1);            % Spinta totale (Thrust) lungo z_B
    tau = u(2:4);         % Momenti torcenti (Torques) attorno a x_B, y_B, z_B

    % Versore Z del mondo
    e3 = [0; 0; 1];

    % --- Equazioni di Newton-Eulero (Differential Flatness based) ---
    
    % 1. Cinematica e Dinamica Traslazionale
    state_dot.pos = state.vel;
    
    % F = m*a -> a = -g*e3 + (u1/m) * (R_BW * e3)
    state_dot.vel = -g * e3 + (u1 / m) * (state.Rbw * e3);
    
    % 2. Cinematica Rotazionale
    omega_BW = state.omega_BW;
    
    % Matrice antisimmetrica (skew-symmetric) di omega
    omega_hat = [    0        ,  -omega_BW(3) ,  omega_BW(2);
                  omega_BW(3) ,      0        , -omega_BW(1);
                 -omega_BW(2) ,   omega_BW(1) ,    0        ];
                
    % Derivata della matrice di rotazione: R_dot = R * omega_hat
    state_dot.Rbw = state.Rbw * omega_hat;
    
    % 3. Dinamica Rotazionale (Equazione di Eulero)
    % J*w_dot = tau - w x (J*w)
    state_dot.omega_BW = J \ (tau - cross(omega_BW, J * omega_BW));
end