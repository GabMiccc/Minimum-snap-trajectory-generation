function [c, total_cost] = trajectoryGen(times, waypoints, config)
% trajectory generator function
    M = size(waypoints, 1);  % Numero di keyframes
    m = M - 1;               % Numero di segmenti tra i keyframes
    n_pos = config.n_pos;
    n_yaw = config.n_yaw;
    k_pos = config.k_pos;
    k_yaw = config.k_yaw;

    %TODO: metti qualcosa per scegliere quali segmenti hanno il corridoio
        
    % Leggiamo il flag per l'adimensionalizzazione temporale
    use_scaling = isfield(config, 'use_scaling') && config.use_scaling;

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
            k = k_pos; % k_r = 4 (Se è 4 stiamo derivando 4 volte fino allo Snap)
            w_points = waypoints(:, dim);
        else
            n = n_yaw;
            k = k_yaw; % k_psi = 2 (Accelerazione Yaw)
            w_points = waypoints(:, dim);
        end
    
        % use_scaling = isfield(config, 'use_scaling') && config.use_scaling;

        % --- PARAMETRI DEL DOCUMENTO (beta1, beta2) ---
        if use_scaling
            beta1 = mean(w_points); % Traslazione (Shift) impostato come media delle coordinate della dimensione selezionata
            beta2 = max(abs(w_points - beta1)); % Scala spaziale
            if beta2 < 1e-4, beta2 = 1.0; end
            w_points_scaled = (w_points - beta1) / beta2; % Variabile w tilde
        else
            beta1 = 0; beta2 = 1.0; w_points_scaled = w_points;
        end
        % H matrix expected dimensions
        debugStruct.AmatrixExpectedRows = (2*m) + (k-1)*(m-1) + 2*(k-1);
        debugStruct.AmatrixExpectedCols = (n+1)*m;
    
        % Costruzione del problema Quadratic Programming (QP): min c'Hc
        H_total = [];        Aeq = [];        beq = [];
        scaling_factors = zeros(1, m); % Salviamo i fattori di scala per osservarli

        % Per ogni segmento i...
        for i = 1:m                  % i=1   i=2   i=m
            T_start = times(i);      % t0    t1    tm-1
            T_end   = times(i+1);    % t1    t2    tm
            alpha_i = T_end - T_start;
            
            % --- FATTORE DI SCALA DEL COSTO (Dalla foto del paper) ---
            cost_scaling_factor = (beta2^2) / (alpha_i^(2*k - 1));
            scaling_factors(i) = cost_scaling_factor;

            % 1. Costruzione Matrice H (Costo dell'integrale della derivata k-esima al quadrato)
            if use_scaling
                % Mondo adimensionale: integrale puro da 0 a 1 (T_dur = 1)
                H_seg_nondim = compute_H_matrix(n, k, 0, 1);
                % Questo è esattamente il termine J_real = J_nondim * (beta2^2 / alpha^(2k-1))
                H_seg = cost_scaling_factor * H_seg_nondim;
            else
                H_seg = compute_H_matrix(n, k, T_start, T_end);
            end            
            H_total = blkdiag(H_total, H_seg);
            
            % % Imposta i tempi di valutazione (0 e 1 se scalato)
            % t_eval_start = T_start; t_eval_end = T_end;
            % if use_scaling
            %     t_eval_start = 0; t_eval_end = 1;
            % end

            % 2. Vincoli di Posizione ai Keyframes 
            % Costruisce le prime 2m righe della matrice
            %   All'inizio del segmento: poly(T_start) = w_points(i)
            new_row_start = zeros(1, (n+1)*m);
            new_row_start((i-1)*(n+1)+1 : i*(n+1)) = poly_basis(n, 0, 0, alpha_i, use_scaling);
            Aeq = [Aeq; new_row_start];
            beq = [beq; w_points_scaled(i)];
            
            %   Alla fine del segmento: poly(T_end) = w_points(i+1)
            new_row_end = zeros(1, (n+1)*m);
            new_row_end((i-1)*(n+1)+1 : i*(n+1))   = poly_basis(n, 0, 1, alpha_i, use_scaling);
            Aeq = [Aeq; new_row_end];
            beq = [beq; w_points_scaled(i+1)];
        end
        
        % 3. Vincoli di Continuità ai Keyframes Intermedi
        % Costruisce le successive (k-1)(m-1) righe della matrice
        % Derivate fino a k-1 devono essere uguali tra segmento i e i+1
        for i = 1:m-1
            alpha_1 = times(i+1) - times(i);
            alpha_2 = times(i+2) - times(i+1);
            % t_eval_end =   times(i+1);
            % t_eval_start = times(i+1);
            % if use_scaling
            %     t_eval_end = 1; t_eval_start = 0;
            % end

            % T_dur_1 = times(i+1) - times(i);
            % T_dur_2 = times(i+2) - times(i+1);

            for deriv = 1:(k-1)
                new_row_cont = zeros(1, (n+1)*m);
                basis_end   = poly_basis(n, deriv, 1, alpha_1, use_scaling);
                basis_start = poly_basis(n, deriv, 0, alpha_2, use_scaling);

                new_row_cont((i-1)*(n+1)+1 : i*(n+1)) = basis_end;
                new_row_cont(i*(n+1)+1 : (i+1)*(n+1)) = -basis_start;
                Aeq = [Aeq; new_row_cont];
                beq = [beq; 0];  % troverai gli stessi valori ma invertiti di segno x-x=0
            end
        end
        
        % 4. Vincoli di Bordo (Velocità/Accelerazione zero all'inizio e alla fine)
        % Costruisce le successive 2(k-1) righe della matrice
        for deriv = 1:(k-1)
            alpha_first = times(2) - times(1);
            % T_dur_first = times(2) - times(1);
            % t_eval_start = times(1);
            % if use_scaling, t_eval_start = 0; end

            % Inizio traiettoria (t0)
            row_start = zeros(1, (n+1)*m);
            row_start(1:n+1)             = poly_basis(n, deriv, 0, alpha_first, use_scaling);
            Aeq = [Aeq; row_start];
            beq = [beq; 0];
            
            alpha_last = times(end) - times(end-1);
            % T_dur_last = times(end) - times(end-1);
            % t_eval_end = times(end);
            % if use_scaling, t_eval_end = 1; end

            % Fine traiettoria (tm)
            row_end = zeros(1, (n+1)*m);
            row_end((m-1)*(n+1)+1 : end) = poly_basis(n, deriv, 1, alpha_last, use_scaling);
            Aeq = [Aeq; row_end];
            beq = [beq; 0];
        end
        
        % --- 5. Vincoli di Diseguaglianza (Safe Corridors) ---
        A = [];         b = [];
        if isfield(config, 'corridor_delta') && ~isempty(config.corridor_delta)
            deltas = config.corridor_delta;       % Vettore Tolleranza lungo m
            num_samples = config.corridor_samples; % Quanti punti controllare per segmento
            
            for i = 1:m
                delta_i = deltas(i);
                % Se il delta è Inf, o NaN, salta questo segmento (nessun corridoio)
                if isinf(delta_i) || isnan(delta_i)
                    continue; 
                end

                % Scaliamo anche l'ampiezza del corridoio!
                if use_scaling
                    delta_i = delta_i / beta2;
                end

                T_start = times(i);
                T_end = times(i+1);
                alpha_i = T_end - T_start;  % sarebbe ALFA, la durata temporale del dcouemnto abs(t)

                % Creiamo dei punti di controllo intermedi nel segmento (escludendo gli estremi)
                t_samples_nondim = linspace(0, 1, num_samples + 2);  % il +2 so gli estremi
                t_samples_nondim = t_samples_nondim(2:end-1); 
                
                % Valori del waypoint di partenza e arrivo per la dimensione corrente
                w_start = w_points_scaled(i);
                w_end   = w_points_scaled(i+1);
                
                for j = 1:length(t_samples_nondim)
                    tau = t_samples_nondim(j);
                    
                    % Calcolo della posizione sulla linea retta ideale al tempo tau
                    line_val = w_start + tau * (w_end - w_start);
                    
                    % t_eval = tau;
                    % if use_scaling, t_eval = alpha; end % Se scalato, usa 's' (che è alpha)

                    % Vettore base del polinomio per questo istante
                    basis = zeros(1, (n+1)*m);
                    basis((i-1)*(n+1)+1 : i*(n+1)) = poly_basis(n, 0, tau, alpha_i, use_scaling);
                    
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
        c_all_nondim = quadprog(H_total, [],  A,  b, Aeq, beq, [], [], [], options);
        %%% x = quadprog(      H,  f,  A,  b, Aeq, beq, lb, ub, x0, options)

        % --- CALCOLO DEL COSTO TOTALE ---
        % H_total contiene già i fattori fisici, quindi il prodotto dà il costo reale!
        total_cost = total_cost + (c_all_nondim' * H_total * c_all_nondim);
        
        % --- RITORNO AL MONDO REALE (Solo per i coefficienti) ---
        c_all_real = zeros(size(c_all_nondim)); % pre-cast

        for i = 1:m
            idx_start = (i-1)*(n+1) + 1;
            idx_end = i*(n+1);
            
            % Coefficienti simulati (adimensionali) per questo segmento
            c_seg_nondim = c_all_nondim(idx_start:idx_end);
            
            % Trasformazione spaziale inversa: P_real(tau) = beta1 + beta2 * P_nondim(tau)
            c_seg_real = c_seg_nondim * beta2;
            c_seg_real(1) = c_seg_real(1) + beta1; % Trasliamo il termine noto c_0 (posizione iniziale)
            
            c_all_real(idx_start:idx_end) = c_seg_real;
        end
        
        % Salvataggio coefficienti
        if dim == 1, c.coeff_x   = reshape(c_all_real, n+1, m); end
        if dim == 2, c.coeff_y   = reshape(c_all_real, n+1, m); end
        if dim == 3, c.coeff_z   = reshape(c_all_real, n+1, m); end
        if dim == 4, c.coeff_psi = reshape(c_all_real, n+1, m); end
    end % for dim = 1:4
end    
