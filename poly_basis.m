function basis = poly_basis(n, k, t_val, T_dur, use_temporal_scaling)
    % Restituisce il vettore dei termini del polinomio derivato k volte
    % Restituisce sempre n-k+1 termini
    % Esempio n=3, k=0: [1, t, t^2, t^3]

    % se use scaling è true. t_val rappresenta s (in [0,1]) e T_dur scala le derivate.
    basis = zeros(1, n+1);
    for i = k:n
        basis(i+1) = prod((i-k+1):i) * t_val^(i-k);

    end
    
    % Applica la regola della catena se usiamo l'adimensionalizzazione 
    % e stiamo effettivamente calcolando una derivata (k > 0)
    if use_temporal_scaling && k > 0
        basis = basis / (T_dur^k);
    end
end
