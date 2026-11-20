function [waypoints, times, config_scenario] = mission_sharp_turn_pca()
    % MISSION_SHARP_TURN_PCA Manovra ad altissima curvatura per stressare attuatori e PCA
    %
    % Simula un avvicinamento veloce seguito da una virata a gomito (hairpin ~165 deg)
    % con rapida variazione di quota e rotazione simultanea di yaw di 180 deg.
    % Ideale per valutare:
    %   1. I limiti fisici degli attuatori (saturazione superiore/inferiore)
    %   2. L'efficacia della Prioritized Control Allocation (tau_x, tau_y > u1 > tau_z)
    %      rispetto al clamping naive quando tau_z e roll/pitch competono.

    % Keyframes: [x (m), y (m), z (m), yaw (rad)]
    waypoints = [
        0.0,  0.0,  1.0,  0.0;     % 1. Partenza in hovering
        5.0,  0.0,  1.5,  0.0;     % 2. Accelerazione rettilinea lungo +X
        6.5,  0.6,  2.5,  pi/2;    % 3. Ingresso curva stretta: salita + yaw +90 deg
        5.0,  1.5,  2.0,  pi;      % 4. Uscita hairpin: inversione verso -X + yaw 180 deg
        0.0,  1.5,  1.0,  pi       % 5. Decelerazione e arrivo
    ];

    % Tempi stretti nella zona di curva (t = 1.8s -> 2.5s -> 3.3s) per forzare
    % elevate accelerazioni centripete e angolari.
    times = [
        0.0;
        1.8;
        2.5;
        3.3;
        5.3
    ];

    m = size(waypoints, 1) - 1;
    config_scenario.corridor_delta = Inf(m, 1);
    config_scenario.corridor_samples = 5;
end

