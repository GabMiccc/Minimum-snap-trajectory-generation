function H = compute_H_matrix(n, k, T_start, T_end)
    % Calcola la matrice Hessiana H per l'integrale della derivata k-esima al quadrato
    % n : ordine del polinomio
    % k : ordine della derivata applicata al polinomio
    % T_start, T_end : valori di tempo dell'integrale
    if k>n
        warning('tentativo di derivare con ordine maggiore rispetto al polinomio');end
    
    H = zeros(n+1, n+1);
    for i = k:n
        for j = k:n
            % Calcola l'integrale di (deriv_k(t^i) * deriv_k(t^j)) dt da T_start a T_end
            mult = prod((i-k+1):i) * prod((j-k+1):j);
            exp_val = i + j - 2*k + 1;
            H(i+1, j+1) = mult * (T_end^exp_val - T_start^exp_val) / exp_val;
        end
    end
end