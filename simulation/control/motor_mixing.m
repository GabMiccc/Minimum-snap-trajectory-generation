function u_real = motor_mixing(u_ideal, config)
    % MOTOR_MIXING Converte i comandi ideali in RPM reali applicando i limiti fisici
    
    % Estrazione parametri costruttivi
    kF = config.kF; 
    kM = config.kM; 
    L  = config.L;  
    
    % Limiti operativi dei motori (rad/s)
    w_min = config.w_min;
    w_max = config.w_max;
    
    % Matrice di Mixing (Configurazione standard a '+')
    M = [ kF,    kF,    kF,    kF;
           0,  kF*L,     0, -kF*L;
        -kF*L,    0,  kF*L,     0;
          kM,   -kM,    kM,   -kM ];
          
    % 1. Calcolo delle velocità al quadrato ideali (Inversione)
    omega_sq_ideal = M \ u_ideal;
    
    % 2. Saturazione (Clamping) dei motori ai limiti fisici
    omega_sq_real = max(w_min^2, min(w_max^2, omega_sq_ideal));
    
    % 3. Ricalcolo delle forze e momenti effettivamente generati
    u_real = M * omega_sq_real;
end