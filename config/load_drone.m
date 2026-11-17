function drone = load_drone(drone_name)
% LOAD_DRONE Factory dispatcher per il caricamento dei modelli di quadricottero.
%
% INPUT:
%   drone_name - Stringa identificativa del velivolo:
%                'heavy_5kg' | 'heavy'
%                'hummingbird'
%
% OUTPUT:
%   drone - Struct contenente tutti i parametri fisici e limiti del modello.

    if nargin < 1 || isempty(drone_name)
        drone_name = 'heavy_5kg';
    end

    switch lower(drone_name)
        case {'heavy_5kg', 'heavy'}
            drone = drone_heavy_5kg();
            
        case {'hummingbird', 'mellinger'}
            if exist('drone_hummingbird', 'file') == 2
                drone = drone_hummingbird();
            else
                error('load_drone:NotFound', ...
                    'Il modello "drone_hummingbird.m" non è ancora presente in config/drones.');
            end

        otherwise
            error('load_drone:UnknownDrone', ...
                'Modello di drone "%s" non riconosciuto. Opzioni disponibili: "heavy_5kg", "hummingbird".', ...
                drone_name);
    end

    % Calcolo automatico del max_tilt_angle_deg fisicamente sostenibile in quota:
    % u1_max * cos(theta_max) >= m * g  =>  theta_max = acos((m * g) / u1_max)
    u1_max = 4 * drone.kF * drone.w_max^2;
    hover_thrust = drone.mass * drone.g;
    if u1_max <= hover_thrust
        warning('load_drone:InsufficientThrust', ...
            'Spinta massima (%.2f N) <= peso (%.2f N): TWR <= 1, impossibile sostenere il volo.', ...
            u1_max, hover_thrust);
        max_tilt_phys_deg = 0;
    else
        max_tilt_phys_deg = rad2deg(acos(hover_thrust / u1_max));
    end

    % Applica il minimo tra eventuale cap operativo del modello e limite fisico da TWR
    if isfield(drone, 'max_tilt_angle_deg') && ~isempty(drone.max_tilt_angle_deg)
        drone.max_tilt_angle_deg = min(drone.max_tilt_angle_deg, max_tilt_phys_deg);
    else
        drone.max_tilt_angle_deg = max_tilt_phys_deg;
    end
end

