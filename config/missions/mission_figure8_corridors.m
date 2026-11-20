function [waypoints, times, config_scenario] = mission_figure8_corridors()
    % MISSION_FIGURE8_CORRIDORS Traiettoria a 8 3D con 11 keyframes e corridoi laterali
    %
    % Estende la figure-8 smooth sostituendo i singoli apici laterali (destro e sinistro)
    % con una coppia di waypoints ciascuno (totale 11 keyframes, 10 segmenti),
    % imponendo un Safe Corridor stretto lungo entrambi i lati esterni dell'8.
    % Lo yaw è srotolato (0 -> 2*pi) per evitare discontinuità.

    % Keyframes: [x (m), y (m), z (m), yaw (rad)]
    waypoints = [
         0.0,   0.0,  1.0,   0.0;        %  1. Partenza in hovering (origine)
         2.2,   2.2,  2.4,   pi/6;       %  2. Salita destra
         4.5,   1.2,  3.5,   pi/3;       %  3. Ingresso corridoio laterale destro
         4.5,  -1.2,  3.5,   2*pi/3;     %  4. Uscita corridoio laterale destro
         2.2,  -2.2,  2.4,   5*pi/6;     %  5. Rientro destro
         0.0,   0.0,  1.5,   pi;         %  6. Incrocio centrale (180 deg)
        -2.2,   2.2,  2.4,   7*pi/6;     %  7. Salita sinistra
        -4.5,   1.2,  3.5,   4*pi/3;     %  8. Ingresso corridoio laterale sinistro
        -4.5,  -1.2,  3.5,   5*pi/3;     %  9. Uscita corridoio laterale sinistro
        -2.2,  -2.2,  2.0,   11*pi/6;    % 10. Rientro sinistro
         0.0,   0.0,  1.0,   2*pi        % 11. Ritorno all'origine (360 deg)
    ];

    % Tempi iniziali proporzionati alle distanze (durata totale 21.0s)
    times = [
         0.0;
         2.5;
         4.5;
         6.5;
         8.5;
        10.5;
        12.5;
        14.5;
        16.5;
        18.5;
        21.0
    ];

    % Safe Corridors: attivi SOLO sui due segmenti laterali (3->4 e 8->9)
    m = size(waypoints, 1) - 1;
    config_scenario.corridor_delta = Inf(m, 1);
    config_scenario.corridor_delta(3) = 0.8;  % Corridoio laterale destro (+/- 0.25 m)
    config_scenario.corridor_delta(8) = 0.8;  % Corridoio laterale sinistro (+/- 0.25 m)
    config_scenario.corridor_samples = 7;
end

