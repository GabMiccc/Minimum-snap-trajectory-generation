function [waypoints, times, config_scenario] = scenario_figure8_3d()
    % SCENARIO_FIGURE8_3D Genera una traiettoria acrobatica a 8 tridimensionale (non complanare)
    %
    % Questa funzione restituisce keyframes 3D [x, y, z, yaw] che tracciano un otto 
    % nello spazio con ampie escursioni di quota (Z fino a 4 metri) e rotazioni continuous di Yaw,
    % ideale per testare la risposta dinamica del Geometric Controller e del PID.

    % Keyframes: [x (m), y (m), z (m), yaw (rad)]
    waypoints = [
         0.0,   0.0,  1.0,   0.0;       % 1. Partenza in Hovering (t = 0.0s)
         2.5,   2.0,  2.8,   pi/4;      % 2. Risalita virata alta destra
         4.5,   0.0,  4.0,   pi/2;      % 3. Apice anello destro (quota max 4m)
         2.0,  -2.0,  2.5,   3*pi/4;    % 4. Picchiata e rientro verso il centro
         0.0,   0.0,  1.5,   pi;        % 5. Incrocio al centro con Yaw a 180°
        -2.0,   2.0,  2.5,  -3*pi/4;    % 6. Risalita anello sinistro
        -4.5,   0.0,  3.5,  -pi/2;      % 7. Apice anello sinistro
        -2.5,  -2.0,  2.0,  -pi/4;      % 8. Virata bassa e riallineamento
         0.0,   0.0,  1.0,   0.0        % 9. Ritorno al punto di origine
    ];

    % Vettore dei tempi di arrivo ai keyframes (in secondi)
    times = [0.0, 2.0, 4.2, 6.5, 8.5, 10.8, 13.0, 15.2, 17.5];

    % Disattiviamo i corridoi di sicurezza per consentire l'evoluzione acrobatica libera
    m = size(waypoints, 1) - 1;
    config_scenario.corridor_delta = Inf(1, m);
    config_scenario.corridor_samples = 5;
end
