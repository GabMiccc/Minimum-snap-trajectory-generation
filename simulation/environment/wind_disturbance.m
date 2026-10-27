function [F_ext, tau_ext] = wind_disturbance(state, t, config)
    % WIND_DISTURBANCE Calcola le forze e i momenti di disturbo ambientali.
    % Modellato per ora sul modello puntiforme del quadrirotore (World Frame).
    %
    % INPUT:
    %   state  - Struct dello stato attuale (pos, vel, Rbw, omega_BW)
    %   t      - Tempo attuale di simulazione (secondi)
    %   config - Struct di configurazione contenente il blocco config.wind
    %
    % OUTPUT:
    %   F_ext   - Vettore forza di disturbo esterna nel World Frame [3x1] (N)
    %   tau_ext - Vettore momento torcente di disturbo nel Body Frame [3x1] (Nm)

    F_ext = [0; 0; 0];
    tau_ext = [0; 0; 0];

    % Se i disturbi non sono abilitati o la struct non esiste, ritorna zeri
    if ~isfield(config, 'wind') || ~isfield(config.wind, 'enable') || ~config.wind.enable
        return;
    end

    w = config.wind;

    % --- GESTIONE PERSISTENTE DEL VENTO RANDOM COSTANTE PER IL VOLO ---
    persistent persistent_random_force last_t;
    
    % Se la simulazione inizia/riparte (t == 0 o t < last_t) o la forza non è ancora inizializzata
    if isempty(last_t) || t < last_t || isempty(persistent_random_force)
        last_t = t;
        persistent_random_force = [0; 0; 0];

        if isfield(w, 'random_enable') && w.random_enable
            max_f = 2.0;
            if isfield(w, 'random_max_force') && w.random_max_force > 0
                max_f = w.random_max_force;
            end

            % Gestione del Seed: se NaN, vuoto o non numerico -> usa 'shuffle' (orologio di sistema)
            if isfield(w, 'random_seed') && ~isnan(w.random_seed) && ~isempty(w.random_seed)
                used_seed = round(w.random_seed);
                rng(used_seed);
            else
                % Seed casuale da orologio di sistema
                s = rng('shuffle');
                used_seed = s.Seed;
            end

            % Generazione direzione 3D casuale e magnitudo
            dir_xy = randn(2, 1);
            if norm(dir_xy) > 1e-6, dir_xy = dir_xy / norm(dir_xy); end
            mag = rand() * max_f;
            f_z = (rand() - 0.5) * 0.2 * max_f;

            persistent_random_force = [dir_xy * mag; f_z];

            % Stampa a cruscotto del seed impiegato per garantire la replicabilità
            fprintf('\n [Vento Random Inizializzato] Seed impiegato: %u | Forza: [%.2f, %.2f, %.2f] N\n', ...
                used_seed, persistent_random_force(1), persistent_random_force(2), persistent_random_force(3));
        end
    end
    last_t = t;

    % 1. Vento Costante Manuale (World Frame)
    if isfield(w, 'constant_force') && ~isempty(w.constant_force)
        F_ext = F_ext + w.constant_force(:);
    end

    % 2. Vento Randomico Costante durante il Volo
    if isfield(w, 'random_enable') && w.random_enable
        F_ext = F_ext + persistent_random_force;
    end

    % 3. Resistenza Aerodinamica / Drag (Modello Puntiforme: F_drag = -k * ||v|| * v)
    if isfield(w, 'k_drag') && w.k_drag > 0
        v_norm = norm(state.vel);
        F_drag = -w.k_drag * v_norm * state.vel;
        F_ext = F_ext + F_drag;
    end

    % 4. Raffiche di Vento Temporizzate (Wind Gusts)
    if isfield(w, 'gust_enable') && w.gust_enable
        if isfield(w, 'gust_start') && isfield(w, 'gust_end') && isfield(w, 'gust_force')
            if t >= w.gust_start && t <= w.gust_end
                F_ext = F_ext + w.gust_force(:);
            end
        end
    end
end
