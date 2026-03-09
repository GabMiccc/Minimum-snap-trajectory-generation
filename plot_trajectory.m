function plot_trajectory(waypoints, times, c, config) % times_final, c_final,
    ts = linspace(times(1), times(end), 500); %time samples (discretization)
    pos_ref = zeros(length(ts), 4); % [x, y, z, yaw]
    
    for i = 1:length(ts) % discretized time steps
        t = ts(i);
        [pos_ref(i,1), ~, ~] = eval_traj(c.coeff_x, times, t, config.n_pos);
        [pos_ref(i,2), ~, ~] = eval_traj(c.coeff_y, times, t, config.n_pos);
        [pos_ref(i,3), ~, ~] = eval_traj(c.coeff_z, times, t, config.n_pos);
        [pos_ref(i,4), ~, ~] = eval_traj(c.coeff_psi, times, t, config.n_yaw);
    end
    
    % Grafico 2D (Vista dall'alto X-Y) - Simile a Fig. 3 e Fig. 6 del doc
    figure('Name', 'Traiettoria Ottimizzata');
    subplot(2,1,1);
    plot(pos_ref(:,1), pos_ref(:,2), 'b-', 'LineWidth', 2); hold on;
    plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on; xlabel('X (m)'); ylabel('Y (m)');
    title('Vista Piano X-Y (Minimum Snap)');
    legend('Traiettoria', 'Keyframes');
    
    % Grafico 3D
    subplot(2,1,2);
    plot3(pos_ref(:,1), pos_ref(:,2), pos_ref(:,3), 'b-', 'LineWidth', 2); hold on;
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Traiettoria 3D');
    view(45, 30);
    
    % Mantiene la proporzione fisica 1:1 tra X e Y, ma comprime Z
    daspect([1, 1, 2]); 
    
    % Riduce i limiti della "scatola" verticale attorno a Z=1
    zlim([0.5, 1.5]);   
    
    % % Opzionale: allarga la scatola del grafico per riempire meglio la finestra
    % pbaspect([2.5, 1.5, 1]);