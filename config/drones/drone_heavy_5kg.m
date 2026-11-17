function drone = drone_heavy_5kg()
    % Proprietà di massa e inerzia
    drone.name = 'Heavy Quadrotor (5.4 kg)';
    drone.mass = 5.4;        % kg
    drone.g    = 9.81;       % m/s^2
    drone.J    = [0.509560, 0.000070, 0.000125;
                  0.000070, 0.528320, 0.000015;
                  0.000125, 0.000015, 0.933390]; % [kg * m^2]

    % Geometria ed aerodinamica
    drone.L    = 0.17;       % braccio (m)
    drone.kF   = 2.4e-5;     % costante spinta (N/(rad/s)^2)
    drone.kM   = 1.22e-6;    % costante drag elica (Nm/(rad/s)^2)

    % Limiti fisici degli attuatori
    drone.w_min = 150;       % rad/s (~1430 RPM)  Idle speed minima
    drone.w_max = 800;       % rad/s (~7640 RPM)  Max RPM   % TODO: check
    drone.max_tilt_angle_deg = 45; % limite inclinazione roll/pitch
    
end
