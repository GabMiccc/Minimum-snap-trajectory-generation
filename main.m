% SCRIPT DI GENERAZIONE TRAIETTORIA - QUADRICOTTERO
clear; clc; close all;

% ---  DEFINIZIONE MANUALE KEYFRAMES E TEMPI ---
% Ogni riga di waypoints: [x, y, z, yaw]
waypoints = [ 0,    0,   1,   0;        % Partenza (Hovering)
              1,    0,   1, pi/4;     % Punto intermedio 1
              1,    2,   1, 3*pi/4;     % Punto intermedio 2
              0,    2,   1,  pi ];      % Ritorno

% times: Vettore dei tempi di arrivo ai keyframes (t0, t1, ..., tm)
% Nota: t0 deve essere 0.
times = [0, 2.0, 5.0, 7.0]; 

% --- CONFIGURAZIONE OTTIMIZZAZIONE ---
config.n_pos = 7; % Ordine polinomi posizione (minimo snap richiede derivate fino alla 4a)
% poichè ogni segmento (intervallo tra due wp) ha 2 estremità e se serve
% imporre la continuità di posizione velocità, accelerazione e jerk su
% entrambi i lati servono 8 condizioni al contorno, 4 per lato. Un
% polinomio di ordine 7 ha 8 coefficienti. Potenzialmente si potrebbe
% imporre la continuità anche del jerk con n=9.
config.n_yaw = 3; % Ordine polinomi yaw (minimo accel. richiede derivate fino alla 2a)
config.k_pos = 4; % (4, snap)
config.k_yaw = 2; % (2, Accelerazione Yaw)

%  -- Parametri per Safe Corridors
% Inf = nessun vincolo, numero = ampiezza massima deviazione (metri)
config.corridor_delta = [Inf, 0.5, Inf]; 

% Quanti punti controllare all'interno di un segmento attivo
config.corridor_samples = 3;

%% generazione traiettoria minimum snap
[c_init, ~] = trajectoryGen(times, waypoints, config);

%% PLOT TRAJECTORY
plot_trajectory(waypoints, times, c_init, config)

%% OPTIMAL SEGMENT TIMES

% Passiamo dalla forma "tempi assoluti" a "durata dei segmenti" T
T = diff(times); 
m = length(T);

% Parametri discesa del gradiente
max_iter = 10;
h = 1e-4; % Perturbazione per il gradiente numerico
learning_rate = 0.5; % Passo di discesa
use_backtracking = true;                                        % SET

cost_history = zeros(1, max_iter);

fprintf('Inizio ottimizzazione tempi dei segmenti...\n');

for iter = 1:max_iter
    % 1. Ricostruisci i tempi assoluti dai segmenti T attuali
    times_current = [0, cumsum(T)];

    % 2. Calcola il costo base f(T)
    [c_current, cost_base] = trajectoryGen(times_current, waypoints, config);
    cost_history(iter) = cost_base;
    
    fprintf('Iterazione %d - Costo totale: %.2f\n', iter, cost_base);
    
    % 3. Calcolo del gradiente direzionale
    grad = zeros(1, m);
    for i = 1:m
        % Costruisci il vettore g_i
        g_i = ones(1, m) * (-1 / (m - 1));   % NEL DOC DICE m-2 al den
        g_i(i) = 1;  
        
        % Perturba T: T_new = T + h * g_i
        T_perturbed = T + h * g_i;
        times_perturbed = [0, cumsum(T_perturbed)];
        
        % Calcola il nuovo costo f(T + h*g_i)
        [~, cost_perturbed] = trajectoryGen(times_perturbed, waypoints, config);   

        % Derivata direzionale
        grad(i) = (cost_perturbed - cost_base) / h;
    end
    
    % 4. Aggiorna i tempi T (Backtracking Line Search)
    if use_backtracking
        % VERO BACKTRACKING (monotono)
        lr = 0.5; % Partiamo con un passo coraggioso
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
        % DISCESA DEL GRADIENTE A PASSO FISSO (caotico)
        % Sottraiamo il gradiente moltiplicato per il learning rate
        T_new = T - learning_rate * grad;
    
        % Vincolo: T_i >= 0 (Nessun segmento può avere tempo negativo)
        T_new = max(T_new, 0.1); 
    
        % Ripristina la somma esatta di T per compensare arrotondamenti
        T = T_new * (sum(T) / sum(T_new)); 
    end
end

%% Visualizzazione dell'Evoluzione del Costo
% se ci sono output enormi ci sarà overshoot (passo apprendimento troppo
% greedy)
figure('Name', 'Ottimizzazione Tempi');
plot(1:max_iter, cost_history, '-co', 'LineWidth', 2, ...
    'MarkerSize', 6, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'w');
xlabel('Iterazione');
ylabel('Funzione di Costo f(T)');
title('Convergenza Discesa del Gradiente (Rif. Fig. 3)');
grid on;
% Miglioriamo i limiti dell'asse Y per centrare bene la curva
ylim([min(cost_history)*0.95, max(cost_history)*1.05]);

%% Visualizzazione Confronto Traiettorie
% times_current e c_current contengono i valori dell'ultima iterazione
plot_trajectory_evolution(waypoints, times, c_init, times_current, c_current, config);