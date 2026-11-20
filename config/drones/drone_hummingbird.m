function drone = drone_hummingbird()
    % AscTec Hummingbird (modello di riferimento da Mellinger & Kumar, ICRA 2011)
    % Parametri convertiti in unità SI (rad/s per le velocità dei rotori)

    % Proprietà di massa e inerzia
    drone.name = 'AscTec Hummingbird (0.5 kg)';
    drone.mass = 0.50;       % kg
    drone.g    = 9.81;       % m/s^2
    drone.J    = [2.32e-3, 0,       0;
                  0,       2.32e-3, 0;
                  0,       0,       4.00e-3]; % [kg * m^2]

    % Geometria ed aerodinamica
    drone.L    = 0.175;      % braccio (m)
    % Nel paper: kF = 6.11e-8 N/RPM^2, kM = 1.5e-9 Nm/RPM^2
    % Conversione da RPM a rad/s: (30/pi)^2 ≈ 91.189
    drone.kF     = 5.57e-6;  % costante spinta (N/(rad/s)^2)
    drone.kM     = 1.37e-7;  % costante drag elica (Nm/(rad/s)^2)
    drone.k_drag = 0.015;    % coeff. resistenza aerodinamica traslazionale (N/(m/s)^2)

    % Limiti fisici degli attuatori
    drone.w_min = 125;       % rad/s (~1200 RPM) Idle speed minima
    drone.w_max = 900;       % rad/s (~8600 RPM) Max RPM (hover ~469 rad/s ≈ 4480 RPM)
    drone.max_tilt_angle_deg = 45; % limite operativo roll/pitch (limite fisico TWR ~74 deg)
end

