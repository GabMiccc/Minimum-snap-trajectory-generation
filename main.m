% SCRIPT DI GENERAZIONE TRAIETTORIA - QUADRICOTTERO
clear; clc; close all;
addpath(genpath(pwd));

% ---  DEFINIZIONE MANUALE KEYFRAMES E TEMPI ---
% Ogni riga di waypoints: [x, y, z, yaw]
waypoints = [ 0,    0,   1,   0;        % Partenza (Hovering)
              4,    0,   2,   pi/6 ;   % Punto intermedio 1
              4,    3,   2, 3*pi/4;     % Punto intermedio 2
              0,    3,   1,  pi ];      % Ritorno

% times: Vettore dei tempi di arrivo ai keyframes (t0, t1, ..., tm)
% Nota: t0 deve essere 0.
times = [0, 5.0, 10.0, 13.0]; 

assert(size(waypoints,1) == length(times))

% --- CONFIGURAZIONE OTTIMIZZAZIONE ---
% -- ORDINE POLINOMI di traiettoria -- 
config.n_pos = 7; % Grado/order polinomi posizione (minimo snap richiede derivate fino alla 4a)
% poichè ogni segmento (intervallo tra due wp) ha 2 estremità e se serve
% imporre la continuità di posizione velocità, accelerazione e jerk su
% entrambi i lati servono 8 condizioni al contorno, 4 per lato. Un
% polinomio di ordine 7 ha 8 coefficienti. Potenzialmente si potrebbe
% imporre la continuità anche del jerk con n=9.
config.n_yaw = 3; % Grado/order polinomi yaw (minimo accel. richiede derivate fino alla 2a, per vincolare posiz e veloc angolare di yaw)
% -- GRADO DELLE DERIVATE DA MINIMIZZARE -- 
config.k_pos = 4; % (4, snap)
config.k_yaw = 2; % (2, Accelerazione Yaw)

%  -- Parametri per Safe Corridors
% Inf = nessun vincolo, numero = ampiezza massima deviazione (metri)
config.corridor_delta = [Inf , 0.2, Inf]; 

assert( size(waypoints,1) == length(times) && size(waypoints,1) == length(config.corridor_delta)+1 )

% Quanti punti controllare all'interno di un segmento attivo
config.corridor_samples = 7;

% -- Temporal scaling
config.use_scaling = true;
% --- DEFINIZIONE SCENARIO ACROBATICO 3D (Anello a 8 Non Complanare) ---
[waypoints, times, config_scenario] = scenario_figure8_3d();
config.corridor_delta = config_scenario.corridor_delta;
config.corridor_samples = config_scenario.corridor_samples;
%% generazione traiettoria minimum snap
[c_init, ~] = trajectoryGen(times, waypoints, config);

%% PLOT TRAJECTORY
plot_trajectory(waypoints, times, c_init, config)

%% OPTIMAL SEGMENT TIMES

% Parametri discesa del gradiente
config.opt_max_iter = 20;
config.opt_h = 1e-4; % Perturbazione (preferibilmente più infinitesima possibile) per il gradiente numerico
config.opt_learning_rate = 0.5; % Passo di discesa/apprendimento iniziale
config.opt_use_backtracking = true;                                        

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
% massa e gravità
config.mass = 5.4; % Massa in kg
config.g = 9.81;   % Accelerazione di gravità (m/s^2)

% Matrice di inerzia J (kg * m^2) lungo gli assi x_B, y_B, z_B
config.J = [0.509560, 0.000070, 0.000125
            0.000070, 0.528320, 0.000015 
            0.000125, 0.000015, 0.933390 ];  % [kg * m^2]

% Parametri aerodinamici e geometrici
config.L = 0.17;                   % Lunghezza braccio (m)
config.kF = 2.4*1e-5; % 8.548e-6;  % Coefficiente di spinta (N / (rad/s)^2)
config.kM = 1.22e-6;  % 1.36e-7;   % Coefficiente di drag (Nm / (rad/s)^2)

% Limiti dei motori (Saturazione)
config.w_min = 150;    % Idle speed minima (rad/s)
config.w_max = 800;    % Max RPM (~7600 RPM convertiti in rad/s)

% =========================================================
% TUNING ANALITICO DEI GUADAGNI (Pole Placement)
% =========================================================
% 1. Parametri di Risposta desiderata
zeta = 1.0;                    % Smorzamento critico (1.0 = massima reattività senza oscillazioni)
wn_pos = 3.0;      % (rad/s)   % Frequenza naturale Posizione (rad/s) -> Reattività lenta e fluida
wn_att = 15.0;     % (rad/s)   % Frequenza naturale Assetto (rad/s) -> Deve essere circa 5x wn_pos
% sistema sottodimensionato, per fare spostamenti laterali deve prima
% inclinarsi e poi spingere -> assetto più veloce di posizione, è l'inner
% loop

% 2. OUTER LOOP (Posizione) - Scalato linearmente sulla Massa
kp_base = config.mass * wn_pos^2;
kv_base = 2 * config.mass * zeta * wn_pos; 

% Creiamo le matrici diagonali (asse Z leggermente più rigido per la gravità)
config.Kp = diag([kp_base, kp_base, kp_base * 1.5]); 
config.Kv = diag([kv_base, kv_base, kv_base * 1.5]);

% 3. INNER LOOP (Assetto) - Scalato matricialmente sull'Inerzia (J)
% Poiché J è già una matrice 3x3 che contiene le inerzie specifiche (Ixx, Iyy, Izz)
% moltiplicando per lo scalare wn_att^2 otteniamo la matrice K_R perfetta.
config.KR = config.J * (wn_att^2); 
config.Komega = config.J * (2 * zeta * wn_att);

% il controllore non lineare SO(3) necessita di queste 4 matrici di
% guadagno

% % 
% % % Tuning del loop di Posizione (Traslazionale)
% % config.Kp = diag([15.0, 15.0, 30.0]); % Reattività all'errore di posizione
% % config.Kv = diag([8.0, 8.0, 15.0]);  % Smorzamento all'errore di velocità
% % 
% % % Tuning del loop di Assetto (Rotazionale)
% % config.KR = diag([3.0, 3.0, 1.5]);     % Reattività all'errore di orientamento (Roll, Pitch, Yaw)
% % config.Komega = diag([0.5, 0.5, 0.3]); % Smorzamento all'errore di velocità angolare (p, q, r)

%% ========= CONTROLLO DI SICUREZZA DI REALIZZABILITà DELLA TRAIETTORIA ===
config.max_tilt_angle_deg = 45;  % deg

mission_possible = check_feasibility(times_current, c_current, config);

%% ==========================================================
% SIMULAZIONE AD ANELLO CHIUSO
% ===========================================================

% Parametri di simulazione
config.dt = 0.001; 
config.warning_cooldown = 0.03;

% --- INIZIO LOOP DI SALVATAGGIO ---
max_retries = 10;
retries = 0;

while ~mission_possible && retries < max_retries
    retries = retries + 1;
    fprintf('\n [Fix missione] Tentativo %d: Dilato il tempo totale del 5%%...\n', retries);
    
    % LA MAGIA DELLO SCALO TEMPORALE: Moltiplichiamo l'intero vettore
    times_current = times_current * 1.05; 
    
    % Poiché i tempi assoluti sono cambiati, dobbiamo ricalcolare 
    % velocemente i coefficienti dimensionali. L'ottimizzatore ci metterà
    % una frazione di secondo perché la forma spaziale è la stessa.
    [c_current, ~] = trajectoryGen(times_current, waypoints, config);
    
    % Ri-testiamo la nuova traiettoria rallentata
    mission_possible = check_feasibility(times_current, c_current, config);
end
% --- FINE LOOP DI SALVATAGGIO ---

if mission_possible
    fprintf(['--- Avvio Simulazione 3D in corso ---\n' ...
             '  - Traiettoria consentita dai limiti prestazionali -   \n   ']);
    simulate_flight(times_current, c_current, waypoints, config);
else
    disp('SIMULAZIONE ABORTITA: Anche allungando i tempi, la traiettoria richiede prestazioni oltre i limiti dei motori.');
    disp('Suggerimento: Allenta l''ottimizzazione temporale o riduci l''aggressività della manovra.');
    
    % TODO: plottare la traiettoria puramente geometrica
    % per vedere cosa avrebbe dovuto fare il drone ???
end