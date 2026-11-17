function [waypoints, times, mission_cfg] = load_mission(mission_name)
% LOAD_MISSION Factory dispatcher per il caricamento delle missioni di volo.
%
% INPUT:
%   mission_name - Stringa identificativa della missione:
%                  'figure8_3d'
%                  'figure8_smooth'
%                  'simple_arc'
%
% OUTPUT:
%   waypoints   - Matrice [M x 4] dei keyframes [x, y, z, yaw]
%   times       - Vettore colonna [M x 1] dei tempi di arrivo ai keyframes
%   mission_cfg - Struct contenente i parametri di missione (waypoints, times, corridor_delta, corridor_samples)

    if nargin < 1 || isempty(mission_name)
        mission_name = 'figure8_3d';
    end

    switch lower(mission_name)
        case {'figure8_3d', 'figure8'}
            [waypoints, times, mission_cfg] = mission_figure8_3d();

        case 'figure8_smooth'
            [waypoints, times, mission_cfg] = mission_figure8_smooth();

        case 'simple_arc'
            [waypoints, times, mission_cfg] = mission_simple_arc();

        otherwise
            error('load_mission:UnknownMission', ...
                'Missione "%s" non riconosciuta. Opzioni disponibili: "figure8_3d", "figure8_smooth", "simple_arc".', ...
                mission_name);
    end

    % Salviamo anche waypoints e times dentro la struct per renderla auto-contenuta
    mission_cfg.name = mission_name;
    mission_cfg.waypoints = waypoints;
    mission_cfg.times = times;
end

