function [F_ext, tau_ext] = wind_disturbance(state, t, config)
    % WIND_DISTURBANCE Calcola le forze e i momenti di disturbo ambientali.
    % Modellato sul principio di sovrapposizione degli effetti nel World Frame:
    %   F_ext(t) = F_drag(v) + F_const + F_gust(t)
    %
    % INPUT:
    %   state  - Struct dello stato attuale (pos, vel, Rbw, omega_BW)
    %   t      - Tempo attuale di simulazione (secondi)
    %   config - Struct di configurazione contenente il blocco config.disturbances
    %
    % OUTPUT:
    %   F_ext   - Vettore forza di disturbo esterna nel World Frame [3x1] (N)
    %   tau_ext - Vettore momento torcente di disturbo nel Body Frame [3x1] (Nm)

    F_ext = [0; 0; 0];
    tau_ext = [0; 0; 0];

    % Master switch di attivazione disturbi
    if ~isfield(config, 'disturbances') || ~isfield(config.disturbances, 'enable') || ~config.disturbances.enable
        return;
    end

    d = config.disturbances;

    % 1. Resistenza Aerodinamica (Drag Puntiforme: F_drag = -k * ||v|| * v)
    if isfield(d, 'drag') && isfield(d.drag, 'enable') && d.drag.enable
        k = d.drag.k;
        v = state.vel;
        F_ext = F_ext - k * norm(v) * v;
    end

    % 2. Vento Costante (World Frame)
    if isfield(d, 'constant') && isfield(d.constant, 'enable') && d.constant.enable
        F_ext = F_ext + d.constant.force(:);
    end

    % 3. Raffiche di Vento ad Impulsi (Treno periodico con Periodo e Duty Cycle)
    if isfield(d, 'gust') && isfield(d.gust, 'enable') && d.gust.enable
        g = d.gust;
        if t >= g.t_start && t <= g.t_end
            if isinf(g.period) || g.period <= 0
                % Impulso singolo continuo tra t_start e t_end
                F_ext = F_ext + g.force(:);
            else
                % Treno periodico di impulsi (PWM-like)
                tau = mod(t - g.t_start, g.period);
                if tau <= (g.period * g.duty_cycle)
                    F_ext = F_ext + g.force(:);
                end
            end
        end
    end
end
