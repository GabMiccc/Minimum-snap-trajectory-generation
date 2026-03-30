function [times_opt, c_opt, cost_history] = optimize_segment_times(times, waypoints, config)
    % OPTIMIZE_SEGMENT_TIMES Ottimizza la durata dei segmenti tramite Discesa del Gradiente
    
    % Estrazione parametri dalla struttura config (con valori di default)
    if isfield(config, 'opt_max_iter'), max_iter = config.opt_max_iter; else max_iter = 10; end
    if isfield(config, 'opt_learning_rate'), learning_rate = config.opt_learning_rate; else learning_rate = 0.5; end
    if isfield(config, 'opt_use_backtracking'), use_backtracking = config.opt_use_backtracking; else use_backtracking = true; end
    if isfield(config, 'opt_h'), h = config.opt_h; else h = 1e-4; end

    % Passiamo dalla forma "tempi assoluti" a "durata dei segmenti" T
    T = diff(times); 
    m = length(T);
    cost_history = zeros(1, max_iter);
    
    % Prepariamo il salvataggio della migliore iterazione
    best_cost = inf;          % Inizializziamo il record a infinito
    best_times = times;       % Array dei tempi
    best_coeffs = struct();   % Struttura dei coefficienti

    fprintf('\n--- Inizio ottimizzazione tempi dei segmenti ---\n');
    
    for iter = 1:max_iter
        % 1. Ricostruisci i tempi assoluti dai segmenti T attuali
        times_current = [0, cumsum(T)]; % T viene sovrascritta
        
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
        grad = zeros(1, m);
        for i = 1:m
            % Costruisci il vettore g_i per ridistribuire il tempo
            g_i = ones(1, m) * (-1 / (m - 1));   
            g_i(i) = 1;  
            
            % Perturba T
            T_perturbed = T + h * g_i;
            times_perturbed = [0, cumsum(T_perturbed)];
            
            % Calcola il nuovo costo
            [~, cost_perturbed] = trajectoryGen(times_perturbed, waypoints, config);   
            
            % Derivata direzionale
            grad(i) = (cost_perturbed - cost_base) / h;
        end
        
        % 4. Aggiorna i tempi T 
        if use_backtracking
            % --- VERO BACKTRACKING --- (monotono)
            lr = learning_rate;  % Parte con un passo coraggioso
            while true
                T_new = T - lr * grad;
                T_new = max(T_new, 0.1); % Evita tempi negativi o nulli
                T_new = T_new * (sum(T) / sum(T_new)); % Mantieni il tempo totale costante
        
                % Ricalcola il costo con i nuovi tempi ipotetici
                times_new = [0, cumsum(T_new)];
                [~, cost_new] = trajectoryGen(times_new, waypoints, config);
        
                % Se il costo migliora (scende) o il passo è diventato microscopico, accetta e procedi
                if cost_new < cost_base || lr < 1e-5
                    T = T_new;
                    break; % Esci dal ciclo while e vai alla prossima iterazione del gradiente
                else
                    % Se il costo esplode, il passo era troppo lungo: dimezzalo e riprova
                    lr = lr / 2;
                end
            end
        else
            % --- DISCESA DEL GRADIENTE A PASSO FISSO --- (caotico)
            % Sottraiamo il gradiente moltiplicato per il learning rate
            T_new = T - learning_rate * grad;
            % Vincolo: T_i >= 0 (Nessun segmento può avere tempo negativo)
            T_new = max(T_new, 0.1); 
            % Ripristina la somma esatta di T per compensare arrotondamenti
            T = T_new * (sum(T) / sum(T_new)); 
        end

        

    end
    
    % --- RESTITUZIONE RISULTATI MIGLIORI ---
    times_opt = best_times;
    c_opt = best_coeffs;
    
    fprintf('--- Ottimizzazione Completata ---\n\n');
    fprintf('Miglior costo applicato alla traiettoria finale: %.2f\n\n', best_cost);
end