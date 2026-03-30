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
times = [0, 2.0, 5.0, 7.0]/2; 

assert(size(waypoints,1) == length(times))

% --- CONFIGURAZIONE OTTIMIZZAZIONE ---
config.n_pos = 7; % Grado/order polinomi posizione (minimo snap richiede derivate fino alla 4a)
% poichè ogni segmento (intervallo tra due wp) ha 2 estremità e se serve
% imporre la continuità di posizione velocità, accelerazione e jerk su
% entrambi i lati servono 8 condizioni al contorno, 4 per lato. Un
% polinomio di ordine 7 ha 8 coefficienti. Potenzialmente si potrebbe
% imporre la continuità anche del jerk con n=9.
config.n_yaw = 3; % Grado/order polinomi yaw (minimo accel. richiede derivate fino alla 2a)
config.k_pos = 4; % (4, snap)
config.k_yaw = 2; % (2, Accelerazione Yaw)

%  -- Parametri per Safe Corridors
% Inf = nessun vincolo, numero = ampiezza massima deviazione (metri)
config.corridor_delta = [Inf, 0.05, Inf]; 

assert( size(waypoints,1) == length(times) && size(waypoints,1) == length(config.corridor_delta)+1 )

% Quanti punti controllare all'interno di un segmento attivo
config.corridor_samples = 3;

% -- Temporal scaling
config.use_scaling = true;

%% generazione traiettoria minimum snap
[c_init, ~] = trajectoryGen(times, waypoints, config);

%% PLOT TRAJECTORY
plot_trajectory(waypoints, times, c_init, config)

%% OPTIMAL SEGMENT TIMES

% Parametri discesa del gradiente
config.opt_max_iter = 20;
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

%% ====================    SIMULAZIONE  ====================

% PARAMETRI FISICI DEL QUADRIROTORE (PLANT)
config.mass = 1.0; % Massa in kg
config.g = 9.81;   % Accelerazione di gravità (m/s^2)

% Matrice di inerzia J (kg * m^2) lungo gli assi x_B, y_B, z_B
config.J = diag([0.01, 0.01, 0.02]); 

% GUADAGNI DEL CONTROLLORE GEOMETRICO 
% ===========================================================
% Tuning del loop di Posizione (Traslazionale)
config.Kp = 15.0; % Reattività all'errore di posizione
config.Kv = 6.0;  % Smorzamento all'errore di velocità

% Tuning del loop di Assetto (Rotazionale)
config.KR = 8.0;     % Reattività all'errore di orientamento (Roll, Pitch, Yaw)
config.Komega = 1.5; % Smorzamento all'errore di velocità angolare (p, q, r)

% Parametri aerodinamici e geometrici
config.L = 0.17;       % Lunghezza braccio (m)
config.kF = 8.548e-6;  % Coefficiente di spinta (N / (rad/s)^2)
config.kM = 1.36e-7;   % Coefficiente di drag (Nm / (rad/s)^2)

% Limiti dei motori (Saturazione)
config.w_min = 150;    % Idle speed minima (rad/s)
config.w_max = 800;    % Max RPM (~7600 RPM convertiti in rad/s)

%% ========= CONTROLLO DI SICUREZZA DI REALIZZABILITà DELLA TRAIETTORIA ===
mission_possible = check_feasibility(times_current, c_current, config);

%% ==========================================================
% SIMULAZIONE AD ANELLO CHIUSO
% ===========================================================

% Parametri di simulazione
config.dt = 0.001; 
config.warning_cooldown = 0.03;

if mission_possible
    fprintf(['--- Avvio Simulazione 3D in corso ---\n' ...
             '  - Traiettoria consentita dai limiti prestazionali -   \n   ']);
    simulate_flight(times_current, c_current, waypoints, config);
else
    disp('SIMULAZIONE ABORTITA: La traiettoria richiede prestazioni oltre i limiti dei motori.');
    disp('Suggerimento: Allenta l''ottimizzazione temporale o riduci l''aggressività della manovra.');
    
    % TODO: plottare la traiettoria puramente geometrica
    % per vedere cosa avrebbe dovuto fare il drone ???
end