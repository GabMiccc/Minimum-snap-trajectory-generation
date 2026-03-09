function [pos, vel, acc] = eval_traj(coeff, times, t, n)
    % coeff: matrice (n+1) x m dei coefficienti calcolati
    % times: vettore dei tempi dei keyframes [t0, t1, ..., tm]
    % t: istante di tempo attuale
    % n: ordine del polinomio
    
    m = length(times) - 1;
    
    % 1. Trova in quale segmento i si trova il tempo t
    if t <= times(1)
        idx = 1;
        t_local = times(1);
    elseif t >= times(end)
        idx = m;
        t_local = times(end);
    else
        idx = find(times <= t, 1, 'last');
        if idx > m, idx = m; end
        t_local = t;
    end
    
    % 2. Estrai i coefficienti del segmento corrispondente
    c = coeff(:, idx);
    
    % 3. Valuta Posizione, Velocità e Accelerazione
    % Usiamo la funzione poly_basis definita prima
    pos = poly_basis(n, 0, t_local) * c;
    vel = poly_basis(n, 1, t_local) * c;
    acc = poly_basis(n, 2, t_local) * c;
end

function basis = poly_basis(n, k, t)
    basis = zeros(1, n+1);
    for i = k:n
        val = prod((i-k+1):i) * t^(i-k);
        basis(i+1) = val;
    end
end