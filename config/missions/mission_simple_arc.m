function [waypoints, times, config_scenario] = mission_simple_arc()
%MISSION_SIMPLE_ARC traiettoria base di test

% OUTPUT: keyframes 3D [x, y, z, yaw]

% Ogni riga di waypoints: [x, y, z, yaw]
waypoints = [ 0,    0,   1,   0;        % Partenza (Hovering)
              4,    0,   2,   pi/6 ;   % Punto intermedio 1
              4,    3,   2, 3*pi/4;     % Punto intermedio 2
              0,    3,   1,  pi ];      % Ritorno

% times: Vettore colonna [M x 1] dei tempi di arrivo ai keyframes (t0, t1, ..., tm)
% Nota: t0 deve essere 0.
times = [0; 5.0; 10.0; 13.0];

%  -- Parametri per Safe Corridors (colonna [m x 1])
% Inf = nessun vincolo, numero = ampiezza massima deviazione (metri)
config_scenario.corridor_delta = [Inf; 0.2; Inf]; 

% Quanti punti controllare all'interno di un segmento attivo
config_scenario.corridor_samples = 7;
end