function [c, total_cost] = trajectoryGen(times, waypoints, config)
% trajectory generator function
    M = size(waypoints, 1);  % Numero di keyframes
    m = M - 1;               % Numero di segmenti tra i keyframes
    n_pos = config.n_pos;
    n_yaw = config.n_yaw;
    k_pos = config.k_pos;
    k_yaw = config.k_yaw;

    %TODO: metti qualcosa per scegliere quali segmenti hanno il corridoio
                   
    % Struttura per memorizzare i coefficienti calcolati
    % coeff_x, coeff_y, coeff_z saranno matrici (n+1) x m
    c.coeff_x = zeros(n_pos + 1, m);
    c.coeff_y = zeros(n_pos + 1, m);
    c.coeff_z = zeros(n_pos + 1, m);
    c.coeff_psi = zeros(n_yaw + 1, m);
    
    total_cost = 0;
    % DEBUG STRUCTURE
    debugStruct = struct();
    
    %% --- : RISOLUZIONE OTTIMIZZAZIONE (MINIMUM SNAP) ---
    % Risolviamo separatamente per ogni dimensione (Proprietà di Disaccoppiamento)
    
    for dim = 1:4 % [x y z yaw]
        if dim < 4  % [x y z]
            n = n_pos;
            k = k_pos; % k_r = 4 (Snap)
            w_points = waypoints(:, dim);
        else
            n = n_yaw;
            k = k_yaw; % k_psi = 2 (Accelerazione Yaw)
            w_points = waypoints(:, dim);
        end
    
        % H matrix expected dimensions
        debugStruct.AmatrixExpectedRows = (2*m) + (k-1)*(m-1) + 2*(k-1);
        debugStruct.AmatrixExpectedCols = (n+1)*m;
    
        % Costruzione del problema Quadratic Programming (QP): min c'Hc
        H_total = [];        Aeq = [];        beq = [];
        
        % Per ogni segmento i...
        for i = 1:m             % i=1   i=2   i=m
            T_start = times(i);    % t0    t1    tm-1
            T_end = times(i+1);    % t1    t2    tm
            
            % 1. Costruzione Matrice H (Costo dell'integrale della derivata k-esima al quadrato)
            H_seg = compute_H_matrix(n, k, T_start, T_end);
            H_total = blkdiag(H_total, H_seg);
            
            % 2. Vincoli di Posizione ai Keyframes 
            % Costruisce le prime 2m righe della matrice
            %   All'inizio del segmento: poly(T_start) = w_points(i)
            new_row_start = zeros(1, (n+1)*m);
            new_row_start((i-1)*(n+1)+1 : i*(n+1)) = poly_basis(n, 0, T_start);
            Aeq = [Aeq; new_row_start];
            beq = [beq; w_points(i)];
            
            %   Alla fine del segmento: poly(T_end) = w_points(i+1)
            new_row_end = zeros(1, (n+1)*m);
            new_row_end((i-1)*(n+1)+1 : i*(n+1)) = poly_basis(n, 0, T_end);
            Aeq = [Aeq; new_row_end];
            beq = [beq; w_points(i+1)];
        end
        
        % 3. Vincoli di Continuità ai Keyframes Intermedi
        % Costruisce le successive (k-1)(m-1) righe della matrice
        % Derivate fino a k-1 devono essere uguali tra segmento i e i+1
        for i = 1:m-1
            t_inter = times(i+1);
            for deriv = 1:(k-1)
                new_row_cont = zeros(1, (n+1)*m);
                basis = poly_basis(n, deriv, t_inter);
                new_row_cont((i-1)*(n+1)+1 : i*(n+1)) = basis;
                new_row_cont(i*(n+1)+1 : (i+1)*(n+1)) = -basis;
                Aeq = [Aeq; new_row_cont];
                beq = [beq; 0];  % troverai gli stessi valori ma invertiti di segno x-x=0
            end
        end
        
        % 4. Vincoli di Bordo (Velocità/Accelerazione zero all'inizio e alla fine)
        % Costruisce le successive 2(k-1) righe della matrice
        for deriv = 1:(k-1)
            % Inizio traiettoria (t0)
            row_start = zeros(1, (n+1)*m);
            row_start(1:n+1) = poly_basis(n, deriv, times(1));
            Aeq = [Aeq; row_start];
            beq = [beq; 0];
            
            % Fine traiettoria (tm)
            row_end = zeros(1, (n+1)*m);
            row_end((m-1)*(n+1)+1 : end) = poly_basis(n, deriv, times(end));
            Aeq = [Aeq; row_end];
            beq = [beq; 0];
        end
        
        % --- 5. Vincoli di Diseguaglianza (Safe Corridors) ---
        A = []; 
        b = [];
        if isfield(config, 'corridor_delta') && ~isempty(config.corridor_delta)
            deltas = config.corridor_delta;       % Vettore Tolleranza lungo m
            num_samples = config.corridor_samples; % Quanti punti controllare per segmento
            
            for i = 1:m
                delta_i = deltas(i);
                % Se il delta è Inf, o NaN, salta questo segmento (nessun corridoio)
                if isinf(delta_i) || isnan(delta_i)
                    continue; 
                end
                
                T_start = times(i);
                T_end = times(i+1);
                
                % Creiamo dei punti di controllo intermedi nel segmento (escludendo gli estremi)
                t_samples = linspace(T_start, T_end, num_samples + 2);
                t_samples = t_samples(2:end-1); 
                
                % Valori del waypoint di partenza e arrivo per la dimensione corrente
                w_start = w_points(i);
                w_end = w_points(i+1);
                
                for j = 1:length(t_samples)
                    t_samp = t_samples(j);
                    
                    % Calcolo della posizione sulla linea retta ideale al tempo t_samp
                    alpha = (t_samp - T_start) / (T_end - T_start);
                    line_val = w_start + alpha * (w_end - w_start);
                    
                    % Vettore base del polinomio per questo istante
                    basis = zeros(1, (n+1)*m);
                    basis((i-1)*(n+1)+1 : i*(n+1)) = poly_basis(n, 0, t_samp);
                    
                    % 1. Vincolo Superiore: P(t) <= line_val + delta
                    A = [A; basis];
                    b = [b; line_val + delta_i];
                    
                    % 2. Vincolo Inferiore: P(t) >= line_val - delta
                    % In forma standard QP (Ac <= b) diventa: -P(t) <= -(line_val - delta)
                    A = [A; -basis];
                    b = [b; -(line_val - delta_i)];
                end
            end
        end

        % % Verifica dimensionamento matrice
        % assert( size(H_total,1) == debugStruct.AmatrixExpectedCols ...
             % && size(H_total,2) == debugStruct.AmatrixExpectedCols ...
             % && size(Aeq,1) == debugStruct.AmatrixExpectedRows ...
             % && size(Aeq,2) == debugStruct.AmatrixExpectedCols)
    
        % Soluzione QP tramite quadprog
        options = optimoptions('quadprog', 'Display', 'off');
        c_all = quadprog(H_total, [],  A,  b, Aeq, beq, [], [], [], options);
        %%% x = quadprog(      H,  f,  A,  b, Aeq, beq, lb, ub, x0, options)
        
        % CALCOLO DEL COSTO (Energia del segmento)
        % Il costo è la forma quadratica valutata con i coefficienti ottimali
        total_cost = total_cost + (c_all' * H_total * c_all);
        
        % Salvataggio coefficienti
        if dim == 1, c.coeff_x = reshape(c_all, n+1, m); end
        if dim == 2, c.coeff_y = reshape(c_all, n+1, m); end
        if dim == 3, c.coeff_z = reshape(c_all, n+1, m); end
        if dim == 4, c.coeff_psi = reshape(c_all, n+1, m); end
    end % for dim = 1:4
end    
