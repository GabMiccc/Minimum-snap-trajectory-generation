function basis = poly_basis(n, k, t)
    % Restituisce il vettore dei termini del polinomio derivato k volte
    % Restituisce sempre n-k+1 termini
    % Esempio n=3, k=0: [1, t, t^2, t^3]
    basis = zeros(1, n+1);
    for i = k:n
        val = prod((i-k+1):i) * t^(i-k);
        basis(i+1) = val;
    end
end
