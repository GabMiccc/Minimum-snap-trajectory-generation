% SCRIPT DI GENERAZIONE TRAIETTORIA - QUADRICOTTERO
clear; clc; close all;
addpath(genpath(pwd));

% --- 1. DEFINIZIONE MANUALE KEYFRAMES E TEMPI (Opzionale / Sandbox) ---
% Ogni riga di waypoints: [x, y, z, yaw]
waypoints = [ 0,    0,   1,   0;        % Partenza (Hovering)
              4,    0,   2,   pi/6 ;    % Punto intermedio 1
              4,    3,   2, 3*pi/4;     % Punto intermedio 2
              0,    3,   1,  pi ];      % Ritorno

% times: Vettore colonna [M x 1] dei tempi di arrivo ai keyframes (t0, t1, ..., tm)
% Nota: t0 deve essere 0.
times = [0; 5.0; 10.0; 13.0]; 

assert(size(waypoints, 1) == length(times))

% --- 2. CONFIGURAZIONE OTTIMIZZAZIONE TRAIETTORIA (PLANNER) ---
% -- ORDINE POLINOMI di traiettoria -- 
config.planner.n_pos = 7; % Grado/order polinomi posizione (minimo snap richiede derivate fino alla 4a)
% poichè ogni segmento (intervallo tra due wp) ha 2 estremità e se serve
% imporre la continuità di posizione velocità, accelerazione e jerk su
% entrambi i lati servono 8 condizioni al contorno, 4 per lato. Un
% polinomio di ordine 7 ha 8 coefficienti. Potenzialmente si potrebbe
% imporre la continuità anche del jerk con n=9.
config.planner.n_yaw = 3; % Grado/order polinomi yaw (minimo accel. richiede derivate fino alla 2a, per vincolare posiz e veloc angolare di yaw)

% -- GRADO DELLE DERIVATE DA MINIMIZZARE -- 
config.planner.k_pos = 4; % (4, snap)
config.planner.k_yaw = 2; % (2, Accelerazione Yaw)

%  -- Parametri per Safe Corridors (colonna [m x 1])
% Inf = nessun vincolo, numero = ampiezza massima deviazione (metri)
config.mission.corridor_delta = [Inf; 0.2; Inf]; 
assert( size(waypoints,1) == length(times) && size(waypoints,1) == length(config.mission.corridor_delta)+1 )

assert( size(waypoints,1) == length(times) && size(waypoints,1) == length(config.corridor_delta)+1 )

% Quanti punti controllare all'interno di un segmento attivo
config.mission.corridor_samples = 7;

% -- Temporal scaling
config.planner.use_scaling = true;
config.planner.motor_headroom = 0.15; % Margine 15% sulla capacità in omega^2 dei singoli motori

% --- 3. DEFINIZIONE SCENARIO ACROBATICO 3D (Sovrascrittura con preset missione) ---
[waypoints, times, config.mission] = load_mission('figure8_3d');

%% --- 4. GENERAZIONE TRAIETTORIA MINIMUM SNAP INIZIALE ---
[c_init, ~] = trajectoryGen(times, waypoints, config);

%% --- 5. PLOT TRAJECTORY ---
plot_trajectory(waypoints, times, c_init, config) % TODO : fix total z axis has to see all the traj, also figure the pos vel acc jerk snap profiles

%% --- 6. OPTIMAL SEGMENT TIMES (DISCESA DEL GRADIENTE) ---
% Parametri discesa del gradiente
config.planner.opt_times.max_iter = 20;
config.planner.opt_times.h = 1e-4; % Perturbazione (preferibilmente più infinitesima possibile) per il gradiente numerico
config.planner.opt_times.learning_rate = 0.5; % Passo di discesa/apprendimento iniziale
config.planner.opt_times.use_backtracking = true;                                        

% OTTIMIZZAZIONE
[times_current, c_current, cost_history] = optimize_segment_times(times, waypoints, config); % spara trajectoryGen() in maniera iterativa

%% Visualizzazione dell'Evoluzione del Costo
% se ci sono output enormi ci sarà overshoot (passo apprendimento troppo greedy)
figure('Name', 'Ottimizzazione Tempi');
plot(1:config.planner.opt_times.max_iter, cost_history, '-co', 'LineWidth', 2, ...
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

%% ==================== SETUP SIMULAZIONE  ====================

% --- 7. PARAMETRI FISICI DEL QUADRIROTORE (PLANT) ---
% Possiamo caricarli dal modulo preset del velivolo:
config.drone = load_drone('heavy_5kg');

% O sovrascriverli/ispezionarli esplicitamente in chiaro:
% config.drone.mass = 5.4; % Massa in kg
% config.drone.g = 9.81;   % Accelerazione di gravità (m/s^2)
% config.drone.J = [0.509560, 0.000070, 0.000125;
%                   0.000070, 0.528320, 0.000015;
%                   0.000125, 0.000015, 0.933390];  % [kg * m^2]
% Parametri aerodinamici e geometrici
% config.drone.L = 0.17;                   % Lunghezza braccio (m)
% config.drone.kF = 2.4e-5;                % Coefficiente di spinta (N / (rad/s)^2)
% config.drone.kM = 1.22e-6;               % Coefficiente di drag elica (Nm / (rad/s)^2)
% Limiti dei motori (Saturazione)
% config.drone.w_min = 150;                % Idle speed minima (rad/s)
% config.drone.w_max = 800;                % Max RPM (~7600 RPM convertiti in rad/s)
% config.drone.max_tilt_angle_deg = 45;    % deg

% =========================================================
% --- 8. CONFIGURAZIONE DISTURBI AMBIENTALI (Vento + Drag su v_rel) ---
% =========================================================
config.disturbances.enable = true;                  % Master switch disturbi

% 1. Resistenza Aerodinamica (usa config.drone.k_drag e v_rel = v - v_wind)
config.disturbances.drag.enable = true;

% 2. Vento Costante (definito come frazione del peso in hover -> m/s)
config.disturbances.constant.enable = true;
const_stress_ratio = 0.15;                          % 15% del peso del drone
const_wind_dir     = [1; 1; 0];                     % Direzione nel World Frame
config.disturbances.constant.vel = wind_vel_from_stress(const_stress_ratio, const_wind_dir, config.drone);

% 3. Raffiche di Vento Temporizzate ad Impulsi (si sommano in velocità al vento costante)
config.disturbances.gust.enable = false;
gust_stress_ratio = 0.10;                           % 10% del peso (singola raffica)
gust_wind_dir     = [1; 0.4; -0.2];                 % Direzione raffica
config.disturbances.gust.vel = wind_vel_from_stress(gust_stress_ratio, gust_wind_dir, config.drone);
config.disturbances.gust.t_start = 1.0;              % Tempo inizio primo impulso (s)
config.disturbances.gust.t_end = 2.0;                % Tempo termine treno di impulsi (s)
config.disturbances.gust.period = 4.0;               % Periodo T [s] (porre Inf per singolo impulso)
config.disturbances.gust.duty_cycle = 0.25;          % Frazione attiva dell'impulso [0-1]

% =========================================================
%% --- 9. TUNING DEI GUADAGNI (Pole Placement o Manuale) ---
% =========================================================
% 1. Parametri di Risposta desiderata
zeta = 1.0;                    % Smorzamento critico (1.0 = massima reattività senza oscillazioni)
wn_pos = 3.0;      % (rad/s)   % Frequenza naturale Posizione (rad/s) -> Reattività lenta e fluida
wn_att = 15.0;     % (rad/s)   % Frequenza naturale Assetto (rad/s) -> Deve essere circa 5x wn_pos
% sistema sottodimensionato, per fare spostamenti laterali deve prima
% inclinarsi e poi spingere -> assetto più veloce di posizione, è l'inner loop

% 2. OUTER LOOP (Posizione) - Scalato linearmente sulla Massa
kp_base = config.drone.mass * wn_pos^2;
kv_base = 2 * config.drone.mass * zeta * wn_pos; 

% Creiamo le matrici diagonali (asse Z leggermente più rigido per la gravità)
config.control.Kp = diag([kp_base, kp_base, kp_base * 1.5]); % Reattività all'errore di posizione
config.control.Kv = diag([kv_base, kv_base, kv_base * 1.5]); % Smorzamento all'errore di velocità

% 3. INNER LOOP (Assetto) - Scalato matricialmente sull'Inerzia (J)
% Poiché J è già una matrice 3x3 che contiene le inerzie specifiche (Ixx, Iyy, Izz)
% moltiplicando per lo scalare wn_att^2 otteniamo la matrice K_R perfetta.
config.control.KR = config.drone.J * (wn_att^2); % Reattività all'errore di orientamento (Roll, Pitch, Yaw)
config.control.Komega = config.drone.J * (2 * zeta * wn_att); % Smorzamento all'errore di velocità angolare (p, q, r)

% Tipologia controllore e flag di avvenuto tuning analitico
config.control.type = "geometric";     % "geometric" | "pid" | "lqr"
config.control.tuned = true;

% =========================================================
% --- 10. SETUP MOTORE SIMULAZIONE & PARAMETRI NUMERICI ---
% =========================================================
config.sim.dt = 0.001; 
config.sim.warning_cooldown = 0.03;

%% ========= CONTROLLO DI SICUREZZA DI REALIZZABILITÀ DELLA TRAIETTORIA ===
mission_possible = check_feasibility_advanced(times_current, c_current, config);

%% ==========================================================
% --- 11. SIMULAZIONE AD ANELLO CHIUSO & LOOP DI SALVATAGGIO ---
% ===========================================================
max_retries = 20;
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
    mission_possible = check_feasibility_advanced(times_current, c_current, config);
end

if mission_possible
    fprintf(['--- Avvio Simulazione 3D in corso ---\n' ...
             '  - Traiettoria consentita dai limiti prestazionali -   \n   ']);
    simulate_flight(times_current, c_current, waypoints, config);
else
    disp('SIMULAZIONE ABORTITA: Anche allungando i tempi, la traiettoria richiede prestazioni oltre i limiti dei motori.');
    disp('Suggerimento: Allenta l''ottimizzazione temporale o riduci l''aggressività della manovra.');
end
