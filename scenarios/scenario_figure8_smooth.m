function [waypoints, times, config_scenario] = scenario_figure8_smooth()
    % SCENARIO_FIGURE8_SMOOTH Traiettoria acrobatica 3D con Yaw srotolato (Unwrapped)
    %
    % Risolve la discontinuità di Yaw a 180° garantendo rotazioni progressive 
    % e continue senza salti di fase, eliminando le richieste estreme di coppia sull'asse Z.

    % Keyframes: [x (m), y (m), z (m), yaw (rad)]
    waypoints = [
         0.0,   0.0,  1.0,   0.0;          % 1. Start Hovering (0°)
         2.5,   2.0,  2.8,   pi/4;         % 2. Salita virata destra (+45°)
         4.5,   0.0,  4.0,   pi/2;         % 3. Apice destro (+90°)
         2.0,  -2.0,  2.5,   3*pi/4;       % 4. Picchiata rientro (+135°)
         0.0,   0.0,  1.5,   pi;           % 5. Incrocio centrale (+180°)
        -2.0,   2.0,  2.5,   5*pi/4;       % 6. Salita sinistra (+225° continua, NON -135°!)
        -4.5,   0.0,  3.5,   3*pi/2;       % 7. Apice sinistro (+270°)
        -2.5,  -2.0,  2.0,   7*pi/4;       % 8. Rientro (+315°)
         0.0,   0.0,  1.0,   2*pi          % 9. Ritorno origine (360° = giro completo fluido)
    ];

    % Tempi proporzionati alle distanze (durata totale ~20s)
    times = [0.0, 2.5, 5.0, 7.5, 10.0, 12.5, 15.0, 17.5, 20.0];

    m = size(waypoints, 1) - 1;
    config_scenario.corridor_delta = Inf(1, m);
    config_scenario.corridor_samples = 5;
end
