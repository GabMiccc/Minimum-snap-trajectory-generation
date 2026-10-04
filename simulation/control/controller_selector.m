function u = controller_selector(state, state_des, config, t)
    % CONTROLLER_SELECTOR Seleziona e invoca il controllore configurato
    %
    % INPUT:
    %   state     - Struct stato attuale reale (pos, vel, Rbw, omega_BW)
    %   state_des - Struct stato desiderato (pos, vel, acc, jerk, yaw, yaw_dot)
    %   config    - Struct di configurazione contenente 'controller_type'
    %   t         - Istante di tempo attuale
    %
    % OUTPUT:
    %   u         - Vettore 4x1 comandi ideali [u1; tau_x; tau_y; tau_z]

    % Controllo del tipo di controllore impostato in config (default: 'geometric')
    if isfield(config, 'controller_type')
        controller_type = config.controller_type;
    else
        controller_type = 'geometric';
    end

    switch lower(controller_type)
        case 'geometric'
            % Controllore Non Lineare su SO(3) (Mellinger & Kumar / Lee et al.)
            u = geometric_controller(state, state_des, config, t);
            
        case 'pid'
            % Controllore PID Classico (Linearizzato su angoli di Eulero)
            if exist('pid_controller', 'file') == 2
                u = pid_controller(state, state_des, config, t);
            else
                error('Controllore PID selezionato ma pid_controller.m non ancora implementato.');
            end
            
        case 'lqr'
            % Controllore LQR (Linear Quadratic Regulator attorno all''hovering)
            if exist('lqr_controller', 'file') == 2
                u = lqr_controller(state, state_des, config, t);
            else
                error('Controllore LQR selezionato ma lqr_controller.m non ancora implementato.');
            end
            
        otherwise
            error('Tipo di controllore sconosciuto: %s. Opzioni valide: ''geometric'', ''pid'', ''lqr''. \n scegli una di queste idiota', controller_type);
    end
end
