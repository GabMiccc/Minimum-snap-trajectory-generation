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

assert(size(waypoints,1) == length(times))

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
config.corridor_delta = [Inf, 0.1, Inf]; 

assert( size(waypoints,1) == length(times) && size(waypoints,1) == length(config.corridor_delta)+1 )

% Quanti punti controllare all'interno di un segmento attivo
config.corridor_samples = 3;

% -- Temporal scaling
config.use_temporal_scaling = true;

%% generazione traiettoria minimum snap
[c_init, ~] = trajectoryGen(times, waypoints, config);

%% PLOT TRAJECTORY
plot_trajectory(waypoints, times, c_init, config)

%% OPTIMAL SEGMENT TIMES

% Parametri discesa del gradiente
config.opt_max_iter = 10;
config.opt_h = 1e-4; % Perturbazione per il gradiente numerico
config.opt_learning_rate = 0.5; % Passo di discesa
config.opt_use_backtracking = true;                                        % SET

% OTTIMIZZAZIONE
[times_current, c_current, cost_history] = optimize_segment_times(times, waypoints, config);

%% Visualizzazione dell'Evoluzione del Costo
% se ci sono output enormi ci sarà overshoot (passo apprendimento troppo
% greedy)
figure('Name', 'Ottimizzazione Tempi');
plot(1:config.opt_max_iter, cost_history, '-co', 'LineWidth', 2, ...
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