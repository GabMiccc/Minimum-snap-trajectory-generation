function ref = get_reference(t, ts, coeff_x, coeff_y, coeff_z, coeff_psi, n_pos, n_yaw)
% funzione di estrazione della traiettoria, dai coefficienti (forma
% "statica") a un flusso dati dinamico per il controller
% nota: il controller ha bisogno di posizione ma anche di velocità r_dot e
% accelerazione r_ddot
% e anche di jerk (r_dddot) e snap (r_ddddot) desiderati per il calcolo dei momenti

    % t: tempo attuale della simulazione
    % ts: vettore dei tempi dei keyframes
    
    % Inizializziamo la struttura del riferimento
    ref.pos = zeros(3,1);
    ref.vel = zeros(3,1);
    ref.acc = zeros(3,1);
    ref.jerk = zeros(3,1);
    ref.snap = zeros(3,1);
    ref.yaw = 0;
    ref.yaw_dot = 0;

    % 1. Trova il segmento i-esimo corrispondente al tempo t
    m = length(ts) - 1;
    if t >= ts(end)
        idx = m;
        t_eval = ts(end); % Resta fermo sull'ultimo punto
    else
        idx = find(ts <= t, 1, 'last');
        t_eval = t;
    end

    % 2. Calcolo Posizione e Derivate per X, Y, Z (n=7, k=4)
    % X
    c_x = coeff_x(:, idx);
    ref.pos(1)  = poly_basis(n_pos, 0, t_eval) * c_x;
    ref.vel(1)  = poly_basis(n_pos, 1, t_eval) * c_x;
    ref.acc(1)  = poly_basis(n_pos, 2, t_eval) * c_x;
    ref.jerk(1) = poly_basis(n_pos, 3, t_eval) * c_x;
    ref.snap(1) = poly_basis(n_pos, 4, t_eval) * c_x; % Necessario per u2, u3

    % Y (stessa logica)
    c_y = coeff_y(:, idx);
    ref.pos(2)  = poly_basis(n_pos, 0, t_eval) * c_y;
    ref.vel(2)  = poly_basis(n_pos, 1, t_eval) * c_y;
    ref.acc(2)  = poly_basis(n_pos, 2, t_eval) * c_y;
    ref.jerk(2) = poly_basis(n_pos, 3, t_eval) * c_y;
    ref.snap(2) = poly_basis(n_pos, 4, t_eval) * c_y;

    % Z (stessa logica)
    c_z = coeff_z(:, idx);
    ref.pos(3)  = poly_basis(n_pos, 0, t_eval) * c_z;
    ref.vel(3)  = poly_basis(n_pos, 1, t_eval) * c_z;
    ref.acc(3)  = poly_basis(n_pos, 2, t_eval) * c_z;
    ref.jerk(3) = poly_basis(n_pos, 3, t_eval) * c_z;
    ref.snap(3) = poly_basis(n_pos, 4, t_eval) * c_z;

    % 3. Calcolo Yaw e derivata (n=3, k=2)
    c_psi = coeff_psi(:, idx);
    ref.yaw     = poly_basis(n_yaw, 0, t_eval) * c_psi;
    ref.yaw_dot = poly_basis(n_yaw, 1, t_eval) * c_psi;
end