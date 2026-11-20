function [waypoints, times, config_scenario] = mission_hoop_pass()
    % MISSION_HOOP_PASS Attraversamento di un anello in aria (hoop)
    %
    % Modella un anello sospeso/lanciato in aria mediante due waypoints molto
    % ravvicinati (pre-anello e post-anello, distanza 0.30 m) collegati da un
    % Safe Corridor stretto (+/- 0.08 m) per forzare il passaggio assiale.
    %
    % Nota: i waypoints 3 (pre-anello, t = 2.40s) e 4 (post-anello, t = 2.55s)
    % rappresentano la finestra temporale di attraversamento dell'anello.

    % Keyframes: [x (m), y (m), z (m), yaw (rad)]
    waypoints = [
        0.00, -1.0,  1.0,  0.0;      % 1. Partenza in hovering
        2.00, -0.3,  2.0,  0.15;     % 2. Avvicinamento e salita verso l'anello
        3.85,  0.0,  2.6,  0.0;      % 3. Ingresso anello (pre-hoop)
        4.15,  0.0,  2.6,  0.0;      % 4. Uscita anello (post-hoop, +0.30 m)
        6.50,  0.8,  2.0, -0.20;     % 5. Virata di uscita post-anello
        8.00,  1.0,  1.0,  0.0       % 6. Decelerazione e arrivo
    ];

    % Tempi di passaggio: attraversamento rapido dell'anello in 0.15s (2.40s -> 2.55s)
    times = [
        0.00;
        1.30;
        2.40;
        2.55;
        4.00;
        5.50
    ];

    % Safe Corridor stretto solo nel segmento attraverso l'anello (segmento 3: wp 3 -> wp 4)
    m = size(waypoints, 1) - 1;
    config_scenario.corridor_delta = Inf(m, 1);
    config_scenario.corridor_delta(3) = 0.08;  % Corridoio stretto anello (+/- 8 cm)
    config_scenario.corridor_samples = 7;
end

