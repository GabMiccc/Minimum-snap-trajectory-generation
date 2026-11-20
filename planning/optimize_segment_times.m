function [times_opt, c_opt, cost_history] = optimize_segment_times(times, waypoints, config)
% OPTIMIZE_SEGMENT_TIMES Ottimizza la durata dei segmenti tramite Discesa del Gradiente

% Estrazione parametri ottimizzazione tempi
p_opt = config.planner.opt_times;
max_iter = p_opt.max_iter;
learning_rate = p_opt.learning_rate;
use_backtracking = p_opt.use_backtracking;
h = p_opt.h;

times = times(:);
% Passiamo dalla forma "tempi assoluti" a "durata dei segmenti" T (vettore colonna [m x 1])
T = diff(times);
m = length(T);
cost_history = zeros(max_iter, 1);

% Prepariamo il salvataggio della migliore iterazione
best_cost = inf;          % Inizializziamo il record a infinito
best_times = times;       % Array dei tempi [M x 1]
best_coeffs = struct();   % Struttura dei coefficienti

fprintf('\n--- Inizio ottimizzazione tempi dei segmenti ---\n');

for iter = 1:max_iter
    % 1. Ricostruisci i tempi assoluti dai segmenti T attuali [M x 1]
    times_current = [times(1); times(1) + cumsum(T)];

    % 2. Calcola il costo base f(T) della iterazione attuale
    [c_current, cost_base] = trajectoryGen(times_current, waypoints, config);
    cost_history(iter) = cost_base;

    % ---> SALVATAGGIO BEST STATE <---
    % Se questo costo base è il migliore mai visto, salvalo
    if cost_base < best_cost
        best_cost = cost_base;
        best_times = times_current;
        best_coeffs = c_current;
    end


    fprintf('Iterazione %d - Costo totale: %.2f\n', iter, cost_base);

    % 3. Calcolo del gradiente direzionale numerico
    grad = zeros(m, 1);
    for i = 1:m % per ogni segmento i-esimo
        % Costruisci il vettore direzionale g_i per ridistribuire il
        % tempo aggiungendo h all' i-esimo segmento
        g_i = ones(m, 1) * (-1 / (m - 1));
        g_i(i) = 1;

        % Perturba T
        T_perturbed = T + h * g_i;
        times_perturbed = [times(1); times(1) + cumsum(T_perturbed)];

        % Calcola il nuovo costo
        [~, cost_perturbed] = trajectoryGen(times_perturbed, waypoints, config);

        % Derivata direzionale
        grad(i) = (cost_perturbed - cost_base) / h;
        % Se grad(i) < 0: aggiungere tempo al segmento i fa diminuire il costo
        % Se grad(i) > 0: aggiungere tempo al segmento i fa aumentare il costo
    end

    % 4. Aggiorna i tempi T
    T_min = 0.05; % Durata minima fisica di un segmento (s)
    grad_norm = norm(grad);
    if grad_norm < 1e-12
        fprintf('Gradiente nullo: ottimo locale raggiunto.\n');
        cost_history(iter:end) = cost_base;
        break;
    end
    step_dir = grad / grad_norm; % Direzione normalizzata per evitare esplosioni su segmenti corti

    if use_backtracking
        % --- VERO BACKTRACKING --- (monotono)
        lr = learning_rate;  % Parte con un passo coraggioso
        accepted = false;
        while lr >= 1e-6
            T_new = T - lr * step_dir;
            if any(T_new < T_min)
                lr = lr / 2;
                continue;
            end
            T_new = T_new * (sum(T) / sum(T_new)); % Mantieni il tempo totale costante
            if any(T_new < T_min)
                lr = lr / 2;
                continue;
            end

            % Ricalcola il costo con i nuovi tempi ipotetici
            times_new = [times(1); times(1) + cumsum(T_new)];
            [~, cost_new] = trajectoryGen(times_new, waypoints, config);

            % Accetta solo se il costo è finito e migliora effettivamente
            if isfinite(cost_new) && cost_new < cost_base
                T = T_new;
                accepted = true;
                break;
            else
                lr = lr / 2;
            end
        end
        if ~accepted
            fprintf('Backtracking: nessun miglioramento ulteriore possibile (lr < 1e-6). Stop.\n');
            cost_history(iter:end) = cost_base;
            break;
        end
    else
        % --- DISCESA DEL GRADIENTE A PASSO FISSO ---
        T_new = T - learning_rate * step_dir;
        T_new = max(T_new, T_min);
        T = T_new * (sum(T) / sum(T_new));
    end
end

% --- RESTITUZIONE RISULTATI MIGLIORI ---
times_opt = best_times(:);
c_opt = best_coeffs;

fprintf('--- Ottimizzazione Completata ---\n\n');
fprintf('Miglior costo applicato alla traiettoria finale: %.2f\n\n', best_cost);
end




% NOTA: controlla se il paper lascia libero il tempo totale
% nell'ottimizzaizone dei segmenti temporali. Può essere che l'algoritmo
% ecide il tempo di volo