% --- STEP DI CAMPIONAMENTO ---
dt = 0.01; % Risoluzione temporale (10ms)
t_query = ts(1):dt:ts(end); % Crea il vettore tempo dall'inizio alla fine
pos_ref = zeros(length(t_query), 3); % Matrice per memorizzare [x, y, z]

for i = 1:length(t_query)
    % Per ogni istante t, calcoliamo la posizione usando la funzione get_reference
    % (o eval_traj) che abbiamo discusso prima
    ref = get_reference(t_query(i), ts, coeff_x, coeff_y, coeff_z, coeff_psi, n_pos, n_yaw);
    pos_ref(i, :) = ref.pos'; 
end