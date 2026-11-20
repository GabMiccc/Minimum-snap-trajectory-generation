function [F_ext, tau_ext] = wind_disturbance(state, t, config)
% WIND_DISTURBANCE Calcola le forze e i momenti di disturbo ambientali.
% Il vento (costante + raffiche) è definito come velocità dell'aria v_wind (m/s)
% nel World Frame, e genera forza aerodinamica tramite la velocità relativa:
%   v_rel = state.vel - v_wind(t)
%   F_ext(t) = -k_drag * ||v_rel|| * v_rel
%
% INPUT:
%   state  - Struct dello stato attuale (pos, vel, Rbw, omega_BW)
%   t      - Tempo attuale di simulazione (secondi)
%   config - Struct di configurazione (config.drone.k_drag, config.disturbances)
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

% 1. Costruzione del campo di velocità del vento v_wind (m/s, World Frame)
v_wind = [0; 0; 0];

% Vento Costante
if isfield(d, 'constant') && isfield(d.constant, 'enable') && d.constant.enable
    v_wind = v_wind + d.constant.vel(:);
end

% Raffiche di Vento ad Impulsi (Treno periodico con Periodo e Duty Cycle)
if isfield(d, 'gust') && isfield(d.gust, 'enable') && d.gust.enable
    g = d.gust;
    if t >= g.t_start && t <= g.t_end
        if isinf(g.period) || g.period <= 0
            % Impulso singolo continuo tra t_start e t_end
            v_wind = v_wind + g.vel(:);
        else
            % Treno periodico di impulsi (PWM-like)
            tau = mod(t - g.t_start, g.period);
            if tau <= (g.period * g.duty_cycle)
                v_wind = v_wind + g.vel(:);
            end
        end
    end
end

% 2. Resistenza Aerodinamica sulla velocità relativa all'aria (v_rel = v - v_wind)
if isfield(d, 'drag') && isfield(d.drag, 'enable') && d.drag.enable
    if isfield(config, 'drone') && isfield(config.drone, 'k_drag')
        k_drag = config.drone.k_drag;
    elseif isfield(d.drag, 'k')
        k_drag = d.drag.k; % Fallback retrocompatibilità con la versione legacy pre-Novembre
    else
        k_drag = 0;
    end
    v_rel = state.vel - v_wind;
    F_ext = F_ext - k_drag * norm(v_rel) * v_rel;
end
end

% ricorda: nella versione pre-Novembre si usava una forma leggermente diversa basata sulla sovrapposizione degli effetti: F_ext(t) = F_drag(v) + F_const + F_gust(t)
% con F_const e F_gust definiti come forze nel world frame. Ora invece si calcola la velocità del vento nel world frame che può essere costante o a impulsi, e poi si calcola la resistenza aerodinamica sulla velocità relativa all'aria