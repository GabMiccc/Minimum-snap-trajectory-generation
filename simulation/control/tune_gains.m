function config = tune_gains(config, Ts_pos, zeta, separation_factor, z_rigidity_factor)
% TUNE_GAINS Calcola i guadagni ottimali (Outer & Inner Loop) per Pole Placement
% partendo dalle specifiche di prestazione temporale (Settling Time Ts).
%
% Dinamica 2° ordine desiderata: s^2 + 2*zeta*wn*s + wn^2 = 0
% Criterio del tempo di assestamento al 2%: Ts = 4 / (zeta * wn) -> wn = 4 / (zeta * Ts)
%
% INPUT:
%   config            - Struct di configurazione contenente config.drone (mass, J)
%   Ts_pos            - [s] Tempo di assestamento desiderato per la posizione (default: 1.33 s -> wn_pos = 3.0 rad/s)
%   zeta              - Fattore di smorzamento (default: 1.0, smorzamento critico senza overshoot)
%   separation_factor - Rapporto tra banda passante assetto e posizione wn_att / wn_pos (default: 5.0)
%   z_rigidity_factor - Irrigidimento inerzia asse z 
%
% OUTPUT:
%   config - Struct aggiornata con i campi:
%            * config.control.Kp
%            * config.control.Kv
%            * config.control.KR
%            * config.control.Komega
%            * config.control.tuned = true
%            * config.control.tuning_specs (metadati delle specifiche utilizzate)

    if nargin < 2 || isempty(Ts_pos),            Ts_pos = 1.33; end
    if nargin < 3 || isempty(zeta),              zeta = 1.0; end
    if nargin < 4 || isempty(separation_factor), separation_factor = 5.0; end
    if nargin < 5 || isempty(z_rigidity_factor), z_rigidity_factor = 1.5; end

    % Controllo presenza parametri fisici del drone
    if ~isfield(config, 'drone') || ~isfield(config.drone, 'mass') || ~isfield(config.drone, 'J')
        error('tune_gains:MissingDroneData', ...
              'La struct config deve contenere config.drone.mass e config.drone.J.');
    end

    m = config.drone.mass;
    J = config.drone.J;

    % 1. Calcolo pulsazioni naturali
    % Ts = 4 / (zeta * wn) -> wn = 4 / (zeta * Ts)
    wn_pos = 4.0 / (zeta * Ts_pos);
    wn_att = separation_factor * wn_pos;
    Ts_att = 4.0 / (zeta * wn_att);

    % 2. OUTER LOOP (Posizione) - Dipendente dalla Massa m
    % Guadagni base per asse
    kp_base = m * (wn_pos^2);
    kv_base = 2 * m * zeta * wn_pos;

    % Matrici diagonali 3x3 (asse Z irrigidito per contrastare la gravità)
    config.control.Kp = diag([kp_base, kp_base, kp_base * z_rigidity_factor]);
    config.control.Kv = diag([kv_base, kv_base, kv_base * z_rigidity_factor]);

    % 3. INNER LOOP (Assetto) - Scalato matricialmente sull'Inerzia J
    % tau = J * dw_des -> KR = J * wn_att^2, Komega = J * (2*zeta*wn_att)
    config.control.KR     = J * (wn_att^2);
    config.control.Komega = J * (2 * zeta * wn_att);

    % 4. Flag e Metadati
    config.control.tuned = true;
    config.control.tuning_specs.Ts_pos            = Ts_pos;
    config.control.tuning_specs.Ts_att            = Ts_att;
    config.control.tuning_specs.wn_pos            = wn_pos;
    config.control.tuning_specs.wn_att            = wn_att;
    config.control.tuning_specs.zeta              = zeta;
    config.control.tuning_specs.separation_factor = separation_factor;
    config.control.tuning_specs.z_rigidity_factor = z_rigidity_factor;


    fprintf(' [Tuning Guadagni] Eseguito con successo su specifica Ts_pos = %.2f s (wn_pos = %.2f rad/s, wn_att = %.2f rad/s)\n', ...
            Ts_pos, wn_pos, wn_att);
end
