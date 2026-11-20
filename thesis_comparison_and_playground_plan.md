# Analisi Dettagliata e Comparativa 1-a-1: MATLAB Codebase vs. Mellinger & Kumar (2011)

## 1. Sintesi dell'Analisi del File `contesto.md`
Il file `contesto.md` riassume accuratamente l'architettura attuale. L'esplorazione del codice conferma che:
- La pipeline **Offline Trajectory Planning → Safety Check → Closed-Loop Dynamic Simulation** è completa e ben strutturata.
- Le scelte ingegneristiche (Tilt Limit, Headroom per la coppia, Pole Placement analitico sull'Inerzia J, Best State nell'ottimizzazione dei tempi) sono effettivamente presenti nel codice e risolvono criticità pratiche di simulazione non dettagliate nel paper teorico.

---

## 2. Confronto 1-a-1: Codice MATLAB vs. Paper Mellinger & Kumar (2011)

| Componente Teorica (Paper 2011) | Implementazione nel Codice MATLAB | Stato & Note di Corrispondenza |
| :--- | :--- | :--- |
| **Uscite Piatte (Flat Outputs)**<br><b>σ</b> = [x, y, z, ψ]ᵀ | `trajectoryGen.m` risolve separatamente 4 QP per x, y, z, ψ. `eval_traj.m` valuta fino alla derivata 4ª per la posizione e 2ª per lo yaw. | **100% Fedele**. Disaccoppiamento completo confermato. |
| **Minimizzazione dello Snap**<br>min ∫ (r⁽⁴⁾)² dt per posizione (kᵣ = 4)<br>∫ (ψ⁽²⁾)² dt per yaw (k_ψ = 2) | config.k_pos = 4, config.k_yaw = 2, ordine polinomi n = 7 (posizione) e n = 3 (yaw). Matrice Hessiana calcolata analiticamente in `compute_H_matrix.m`. | **100% Fedele**. `compute_H_matrix` calcola l'integrale esatto ∫ (p⁽ᵏ⁾(t))² dt. |
| **Adimensionalizzazione (Scaling)**<br>Scaling spaziale (β₁, β₂) e temporale (αᵢ = T_end - T_start) | In `trajectoryGen.m`: β₁ = mean(w), β₂ = max(abs(w - β₁)), fattore di costo β₂² / αᵢ²ᵏ⁻¹. `poly_basis.m` applica la regola della catena 1 / αᵏ. | **100% Fedele**. Segue esattamente le formule (12)-(13) del paper per garantire il condizionamento numerico della QP. |
| **Safe Corridors (Vincoli di Corridoio)**<br>max(abs(dᵢ(t))) ≤ δᵢ | In `trajectoryGen.m`: campionamento di punti intermedi config.corridor_samples con disuguaglianze aperte in forma QP A · c ≤ b. Supporta Inf per segmenti liberi. | **100% Fedele**. Implementazione della Sezione V-B. |
| **Ottimizzazione Tempi Segmenti**<br>Gradient descent con vettori di direzione gᵢ e somma temporale costante | In `optimize_segment_times.m`: calcola il gradiente direzionale con gᵢ(i) = 1 e gᵢ(j ≠ i) = -1 / (m - 1). Usa backtracking line search. | **Migliorato**. Aggiunta la memoria **Best State** per evitare di salvare iterazioni degenerate da overshooting del gradiente. |
| **Controllo Geometrico su SO(3)**<br>F_des = -K_p e_p - K_v e_v + m g z_W + m r̈_T<br>u₁ = F_des · z_B<br>e_R = ½ (R_desᵀ R_B - R_Bᵀ R_des)ˇ | In `geometric_controller.m`: calcola F_des, proietta su z_B, estrae R_des = [x_B_des, y_B_des, z_B_des], mappa vee per e_R, calcola e_ω. | **100% Fedele + Migliorato**. Include la funzione di stabilità esponenziale Ψ = ½ tr(I - R_desᵀ R_BW) con warning di cooldown e Tilt-Limit. |
| **Feedforward delle Velocità Angolari**<br>h_ω = (m / u₁) (ȧ - (z_B · ȧ) z_B)<br>p_des = -h_ω · y_B, q_des = h_ω · x_B | In `geometric_controller.m` (righe 120-137): implementata sia la formula contratta col jerk che la verifica esplicita con h_ω. | **100% Fedele**. È presente un check di uguaglianza che lancia un warning se i due metodi divergono. |
| **Motor Mixing & Saturazione**<br>Conversione [u₁, τ] → ωᵢ², clamping a [ω_min, ω_max] | In `motor_mixing.m`: usa la matrice di mixing M a '+' per convertire in ωᵢ², limita in [ω_min², ω_max²] e ricalcola u_real. | **100% Fedele**. Rappresenta l'attuazione fisica reale. |
| **Simulatore Dinamico (Plant)**<br>Newton-Eulero con rotazione R_BW | In `quadrotor_dynamics.m` + `simulate_flight.m` con integrazione di Eulero (dt = 0.001) e ri-ortogonalizzazione SVD della matrice di rotazione R_BW. | **Eccellente**. L'uso della SVD ad ogni passo previene la deriva numerica della matrice di rotazione. |

---

## 3. Scostamenti Ingegneristici e Miglioramenti Introdotti

1. **Analytical Tuning tramite Pole Placement (`main.m`)**:
   Invece di guadagni scalari generici (K_p, K_v, K_R, K_ω), la codebase calcola le matrici di guadagno basandosi sulla massa m e sulla matrice di inerzia J, impostando frequenze naturali distinte per Outer Loop (ωₙ,ₚₒₛ = 3.0 rad/s) e Inner Loop (ωₙ,ₐₜₜ = 15.0 rad/s).
2. **Tilt-Limit Anti-Capovolgimento (`geometric_controller.m`)**:
   Se la forza orizzontale richiesta F_des(1:2) supera il limite di tilt (es. 45° o 90°), viene riscalata per preservare la componente verticale F_des(3), impedendo al drone di cadere mentre tenta virate troppo strette.
3. **Headroom del 10%-20% nel Feasibility Check (`check_feasibility.m`)**:
   Riserva un margine sulla spinta massima u₁,ₘₐₓ = 0.90 · (4 · k_F · ω_max²) affinché i motori abbiano l'autorità di controllo necessaria per generare i momenti di assetto (τ) senza saturare.
4. **Auto Temporal Scaling Loop (`main.m`)**:
   Se la traiettoria pianificata supera i limiti fisici, il sistema applica autonomamente una dilatazione temporale del 5% ricondizionando i coefficienti al volo senza dover ri-eseguire l'ottimizzazione dei tempi da zero.

---

## 4. Architettura Proposta per il "Thesis Playground" (Esperimenti per la Tesi)

Per trasformare questa codebase in un framework di simulazione e ricerca ideale per una Tesi Magistrale, si raccomanda di strutturare il progetto come un **Playground Modulare**.

### Moduli Sperimentali Raccomandati per la Tesi:

```
playground/
├── core/
│   ├── planning/          (trajectoryGen, optimize_segment_times, eval_traj)
│   ├── control/           (geometric_controller, pid_controller, lqr_controller)
│   ├── dynamics/          (quadrotor_dynamics, motor_mixing, motor_first_order_delay)
│   └── environment/       (wind_disturbance, drag_effects, payload_mass_change)
├── experiments/
│   ├── exp1_controller_comparison.m   (Geometric vs PID vs LQR under high tilt)
│   ├── exp2_robustness_wind.m          (Tracciamento traiettoria con raffiche di vento)
│   ├── exp3_parametric_uncertainty.m  (Incertezza sulla massa +20% e sull'inerzia J)
│   ├── exp4_actuator_fault.m           (Perdita di efficienza di 1 motore 15%)
│   └── exp5_dynamic_hoop_interception.m (Replica esplicita dell'esperimento del cerchio lanciato)
└── benchmarks/
    └── compute_metrics.m  (Calcolo automatico di RMSE posizione, consumo energetico, max tilt)
```

### Prossimi Passi Consigliati per lo Sviluppo della Tesi:
1. **Riconfigurazione a Classi / Struct di Configurazione Pulita**: Separare nettamente la configurazione degli esperimenti dai moduli principali.
2. **Implementazione di Controllori di Confronto**: Aggiungere un controllore PID/Euler-angle o LQR per dimostrare quantitativamente nella tesi perché il controllore Geometrico su SO(3) supera i metodi classici durante manovre ad alte accelerazioni.
3. **Aggiunta di Disturbi Fisici Reali**: Simulare il ritardo del motore (dinamica di 1° ordine delle eliche ω̇ = (ω_des - ω) / τ_m), il drag aerodinamico lineare/quadratico e raffiche di vento costanti/gusts.
