%% ========================================================================
%  MINIMUM SNAP TRAJECTORY GENERATION & CONTROL - PLAYGROUND CRUSCOTTO
%  Pannello di controllo minimale e modulare per esperimenti e simulazioni
%% ========================================================================
clear; clc; close all;
addpath(genpath(pwd));

%% --- 1. CRUSCOTTO DI CONTROLLO (SELEZIONE VELOCE) ---
DRONE_NAME      = 'heavy_5kg';       % 'heavy_5kg' (5.4 kg) | 'hummingbird' (0.5 kg)
MISSION_NAME    = 'figure8_3d';      % 'figure8_3d' | 'figure8_smooth' | 'simple_arc'
CONTROLLER_TYPE = 'pid';       % 'geometric' | 'pid'

% Flag di esecuzione
FLAG_OPTIMIZE_TIMES = true;          % Esegue gradient descent su ripartizione temporale
FLAG_ENABLE_WIND    = true;          % Attiva disturbi ambientali (vento/drag)

%% --- 2. CARICAMENTO MODULI E PARAMETRIZZAZIONE ---
% Caricamento piattaforma (fisica e limiti motori)
config.drone = load_drone(DRONE_NAME);

% Caricamento missione (keyframes, tempi e corridoi)
[waypoints, times, config.mission] = load_mission(MISSION_NAME);

% Parametri Trajectory Planner (Minimum Snap)
config.planner.n_pos = 7;            % Ordine polinomi posizione
config.planner.n_yaw = 3;            % Ordine polinomi yaw
config.planner.k_pos = 4;            % Derivata da minimizzare (4 = snap)
config.planner.k_yaw = 2;            % Derivata yaw da minimizzare (2 = accel)
config.planner.use_scaling = true;   % Scalatura temporale [0, 1]
config.planner.motor_headroom = 0.15; % Margine 15% sulla capacità in omega^2 dei singoli motori

% Parametri Ottimizzazione Tempi
config.planner.opt_times.max_iter = 20;
config.planner.opt_times.h = 1e-4;
config.planner.opt_times.learning_rate = 0.5;
config.planner.opt_times.use_backtracking = true;

% Controllore e Guadagni (Tuning Analitico su specifica temporale)
config.control.type = CONTROLLER_TYPE;

% Tuning automatico
% Calcola analiticamente Kp, Kv sulla massa e KR, Komega sull'inerzia J
Ts_pos = 1.0; % tempo di assestamento desiderato per l'errore di posizione
zeta = 1.0;
separation_factor = 5.0;
z_rigidity_factor = 1.5;   % Irrigidimento inerzia asse z
config = tune_gains(config, Ts_pos);  

% Configurazione Disturbi Ambientali (Vento in m/s + Drag su v_rel con drone.k_drag)
config.disturbances.enable = FLAG_ENABLE_WIND;
config.disturbances.drag.enable = true;

config.disturbances.constant.enable = true;
const_stress_ratio = 0.50;                           % 15% del peso del drone in hover
const_wind_dir     = [1; 1; 0];                      % Direzione nel World Frame
config.disturbances.constant.vel = wind_vel_from_stress(const_stress_ratio, const_wind_dir, config.drone);

config.disturbances.gust.enable = false;             % Treno di raffiche (si sommano al vento costante)
gust_stress_ratio = 0.10;                            % 10% del peso (singola raffica)
gust_wind_dir     = [1; 0.5; -0.2];                  % Direzione raffica
config.disturbances.gust.vel = wind_vel_from_stress(gust_stress_ratio, gust_wind_dir, config.drone);
config.disturbances.gust.t_start = 1.0;
config.disturbances.gust.t_end = 8.0;
config.disturbances.gust.period = 4.0;
config.disturbances.gust.duty_cycle = 0.25;

% Motore di Simulazione
config.sim.dt = 0.001;               % Integrazione 1000 Hz
config.sim.warning_cooldown = 0.03;

%% --- 3. PIANIFICAZIONE DELLA TRAIETTORIA ---
fprintf('[1/4] Calcolo traiettoria Minimum Snap iniziale...\n');
[c_init, ~] = trajectoryGen(times, waypoints, config);
plot_trajectory(waypoints, times, c_init, config);

times_current = times;
c_current = c_init;

if FLAG_OPTIMIZE_TIMES
    fprintf('[2/4] Ottimizzazione ripartizione temporale...\n');
    [times_current, c_current, cost_history] = optimize_segment_times(times, waypoints, config);
    
    figure('Name', 'Ottimizzazione Tempi');
    plot(1:config.planner.opt_times.max_iter, cost_history, '-co', 'LineWidth', 2, ...
        'MarkerSize', 6, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'w');
    xlabel('Iterazione'); ylabel('Costo f(T)');
    title('Convergenza Discesa del Gradiente'); grid on;
    
    plot_trajectory_evolution(waypoints, times, c_init, times_current, c_current, config);
end

%% --- 4. VERIFICA DI FATTIBILITÀ FISICA & DILATAZIONE TEMPORALE ---
fprintf('[3/4] Analisi vincoli motori e coppie attuatori...\n');
mission_possible = check_feasibility_advanced(times_current, c_current, config);

max_retries = 20;
retries = 0;
while ~mission_possible && retries < max_retries
    retries = retries + 1;
    fprintf(' [Fix missione] Tentativo %d: Dilato il tempo totale del 5%%...\n', retries);
    times_current = times_current * 1.05;
    [c_current, ~] = trajectoryGen(times_current, waypoints, config);
    mission_possible = check_feasibility_advanced(times_current, c_current, config);
end

%% --- 5. SIMULAZIONE AD ANELLO CHIUSO ---
if mission_possible
    fprintf('[4/4] Avvio simulazione ad anello chiuso...\n');
    simulate_flight(times_current, c_current, waypoints, config);
else
    warning('Simulazione non eseguibile: la manovra supera le capacità fisiche dei motori.');
end
