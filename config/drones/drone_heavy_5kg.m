function drone = drone_heavy_5kg()
    % Proprietà di massa e inerzia
    drone.name = 'Heavy Quadrotor (5.4 kg)';
    drone.mass = 5.4;        % kg
    drone.g    = 9.81;       % m/s^2
    drone.J    = [0.509560, 0.000070, 0.000125;
                  0.000070, 0.528320, 0.000015;
                  0.000125, 0.000015, 0.933390]; % [kg * m^2]

    % Geometria ed aerodinamica
    drone.L      = 0.45;     % braccio (m) % TODO: chiedi al prof quanto vale
    drone.kF     = 2.4e-5;   % costante spinta (N/(rad/s)^2)
    drone.kM     = 1.22e-6;  % costante drag elica (Nm/(rad/s)^2)
    drone.k_drag = 0.05;     % coeff. resistenza aerodinamica traslazionale (N/(m/s)^2, da A~0.07 m^2)

    % Limiti fisici degli attuatori
    drone.w_min = 150;       % rad/s (~1430 RPM) Idle speed minima % TODO: chiedi al prof quanto vale
    drone.w_max = 1050;      % rad/s (~10025 RPM) Max RPM (TWR ~ 2.0) % TODO: chiedi al prof quanto vale
    drone.max_tilt_angle_deg = 45; % limite inclinazione roll/pitch (ricalcolato da TWR in load_drone)
    
end
