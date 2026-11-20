# Contesto Recente — Update Ultime Due Settimane (Novembre 2026)

Questo documento riassume l'evoluzione recente del progetto, partendo dai dubbi emersi durante i test e arrivando alle soluzioni implementate fino al **17/11/2026**.

---

## 1. Fattibilità Traiettoria, Headroom Motori e Prioritized Control Allocation (PCA)

### Dubbio di partenza
- Il vecchio `check_feasibility.m` stimava le coppie di roll/pitch in modo euristico (senza usare l'inerzia $J$ né lo yaw), non controllava lo spegnimento del singolo motore (`w_min`) e usava un headroom rudimentale sulla sola spinta totale (`max_total_thrust * 0.90`).
- Passando a `check_feasibility_advanced.m` (basato su *differential flatness* completa), l'headroom era sparito: una traiettoria accettata al 99.9% di `w_max` in anello aperto saturava appena partiva il controllore ad anello chiuso con disturbi.
- Inoltre, in `motor_mixing.m` il clamping indipendente dei 4 motori rischiava di sacrificare roll/pitch a causa di richieste elevate di yaw ($\tau_z$).

### Cosa è stato implementato
- **`planning/check_feasibility_advanced.m`**:
  - Campionamento fine (`50 Hz`), calcolo esatto di $u_1$, $\omega$, $\dot{\omega}$ e coppia di Newton-Eulero $\tau = J\dot{\omega} + \omega \times (J\omega)$, con verifica di saturazione superiore (`w_max`) e inferiore (`w_min`) per **ogni singolo rotore**.
  - Aggiunto il parametro configurabile `config.planner.motor_headroom` (applicato in $\omega^2$, cioè sulla spinta disponibile: `w_max_sq_eff = (1 - headroom) * w_max^2` e `w_min_sq_eff = (1 + headroom) * w_min^2`).
  - Impostato `config.planner.motor_headroom = 0.15` in `main.m` e `run_playground.m` (nota: da valutare se portare a `0.0` il fallback di default dentro `check_feasibility_advanced.m` quando non specificato dalla missione).
- **`simulation/control/motor_mixing.m`**:
  - Sostituito il clamping cieco con **Prioritized Control Allocation (PCA) analitica** in forma chiusa (sfruttando la decomposizione ortogonale di $M^{-1}$ nei contributi di $u_1$, $(\tau_x, \tau_y)$ e $\tau_z$).
  - Fast path a costo zero quando nessun rotore satura; in saturazione applica la gerarchia **Roll/Pitch ($\tau_x, \tau_y$) $\succ$ Spinta ($u_1$) $\succ$ Yaw ($\tau_z$)**.

---

## 2. Limite di inclinazione (`max_tilt_angle_deg`)

### Dubbio di partenza
- Nel controllore geometrico (`simulation/control/geometric_controller.m`) era presente `max_tilt_angle_deg = 45`, ma non era chiaro se fosse utile e se per un drone pesante dovesse essere più basso.
- Con un TWR basso, inclinarsi a `45°` riduce la componente verticale della spinta ($u_{1,\max} \cos 45^\circ < mg$), facendo perdere quota inevitabilmente.

### Cosa è stato implementato
- **`config/load_drone.m`**:
  - Calcolo automatico del limite fisico sostenibile in quota in base al TWR del drone caricato:
    $$\theta_{\max,\text{phys}} = \arccos\!\left(\frac{mg}{4 k_F w_{\max}^2}\right)$$
  - Applicato `min(drone.max_tilt_angle_deg, max_tilt_phys_deg)`: mantiene un cap operativo (es. `45°`) sui droni agili ad alto TWR, ma lo riduce automaticamente sui droni pesanti se il TWR non lo consente.

---

## 3. Parametri dei Droni (`drone_heavy_5kg` vs `drone_hummingbird`)

### Dubbio di partenza
- Con `w_max = 800 rad/s` in `drone_heavy_5kg.m`, il drone da `5.4 kg` risultava avere $\text{TWR} = 1.16$, con rotori in hover già al **92.9% di `w_max`** (~743 rad/s) e angolo limite fisico di appena $30.4^\circ$.
- Confrontando con la Tabella 7.4 della tesi di Giovanni Di Monaco (Sapienza, trasporto carichi appesi), si è visto che la tabella fornisce $m$, $I$, $k_T$, $k_d$, ma **non** riporta `L` e `w_max` (che erano rimasti come placeholder copiati da droni piccoli).

### Cosa è stato implementato
- **`config/drones/drone_heavy_5kg.m`**:
  - Mantenuti intatti i dati della Tabella 7.4 (`mass = 5.4`, `J`, `kF = 2.4e-5`, `kM = 1.22e-6`).
  - Aggiornati i parametri mancanti con stime fisicamente realistiche per un heavy-lift da ~5 kg ($\text{TWR} \approx 2.0$), taggati con `% TODO: chiedi al prof quanto vale`: `L = 0.45 m`, `w_min = 150 rad/s`, `w_max = 1050 rad/s`.
- **`config/drones/drone_hummingbird.m`** *(nuovo)*:
  - Creato il modello dell'**AscTec Hummingbird** (`0.50 kg`, parametri del paper di Mellinger & Kumar convertiti in SI `rad/s`, `w_max = 900 rad/s`, $\text{TWR} \approx 3.68$), richiamabile tramite `load_drone('hummingbird')`.

---

## 4. Tuning analitico dei guadagni (`tune_gains.m`)

### Dubbio di partenza
- Cambiando modello di drone (da `5.4 kg` a `0.5 kg`) o modificando massa/inerzia, i guadagni fissi $K_p, K_v, K_R, K_\omega$ non scalavano e rendevano impossibile confrontare in modo equo controllori e piattaforme diverse.

### Cosa è stato implementato
- **`simulation/control/tune_gains.m`**:
  - Calcolo analitico per **Pole Placement** (modello del 2° ordine con criterio del 2%: $\omega_n = \frac{4}{\zeta T_s}$) a partire da massa `m`, inerzia `J` e tempo di assestamento desiderato `Ts_pos` (con `zeta = 1.0`, `separation_factor = 5.0` tra banda interna ed esterna e `z_rigidity_factor = 1.5` sull'asse verticale).

---

## 5. Altri Highlights recenti nel Repository

- **Nuovo cruscotto modulare (`run_playground.m`)**: setup rapido di drone (`load_drone`), missione (`load_mission`), controllore (`geometric` / `pid`), disturbi e ottimizzazione tempi.
- **Controllore PID e Benchmark (`simulation/control/pid_controller.m`, `benchmarks/compute_metrics.m`)**: aggiunto il controllore PID per confronto con il geometrico e calcolo sistematico degli errori (RMSE, ecc.).
- **Disturbi ambientali e fix Yaw (`e1dec4f`)**: modello di drag, vento costante e raffiche ad impulsi, più fix al bug di crash dovuto al mancato *unwrap* dello yaw.

---

## Prossimi TODO aperti
1. **Conferma parametri drone pesante**: chiedere al relatore i valori esatti di `L`, `w_min` e `w_max` del modello della tesi Di Monaco.
2. **Default `motor_headroom`**: decidere se portare a `0.0` il fallback in `check_feasibility_advanced.m` (lasciando il margine solo dove richiesto dalla missione).
3. **Controllore LQR**: implementare la variante LQR per completare il benchmark `Geometric vs PID vs LQR`.
4. **Test comparativi**: eseguire `run_playground.m` sia su `heavy_5kg` sia su `hummingbird` per validare PCA + `tune_gains` + headroom in entrambi gli scenari.

