% Esegui questo dopo aver risolto quadprog per vedere la fisica del drone
figure('Name', 'Analisi Derivate');
subplot(3,1,1); plot(t_query, pos_ref(:,1:3)); title('Posizione (m)'); grid on;
subplot(3,1,2); % Qui vedrai se la velocità è fluida o ha "salti"
vel_plot = diff(pos_ref(:,1:3)) ./ diff(t_query');
plot(t_query(1:end-1), vel_plot); title('Velocità (m/s) - Deve essere continua'); grid on;
subplot(3,1,3); % Qui vedrai l'accelerazione
acc_plot = diff(vel_plot) ./ diff(t_query(1:end-1)');
plot(t_query(1:end-2), acc_plot); title('Accelerazione (m/s^2) - Deve essere continua'); grid on;