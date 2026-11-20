function v_wind = wind_vel_from_stress(stress_ratio, dir_vec, drone)
    % WIND_VEL_FROM_STRESS Converte una frazione del peso del drone (stress_ratio)
    % nella velocità del vento equivalente in hover [m/s] lungo la direzione dir_vec.
    %
    % INPUT:
    %   stress_ratio - Rapporto F_target / (m * g) (es. 0.15 = 15% del peso)
    %   dir_vec      - Vettore direzione nel World Frame [3x1] (verrà normalizzato)
    %   drone        - Struct config.drone (mass, g, k_drag)
    %
    % OUTPUT:
    %   v_wind       - Vettore velocità del vento [3x1] (m/s)

    if stress_ratio <= 0 || norm(dir_vec) < 1e-9 || drone.k_drag <= 0
        v_wind = [0; 0; 0];
        return;
    end

    F_target = stress_ratio * drone.mass * drone.g;
    v_mag    = sqrt(F_target / drone.k_drag);
    v_wind   = v_mag * (dir_vec(:) / norm(dir_vec));
end

