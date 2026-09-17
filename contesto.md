# Documentazione di Progetto: Simulatore Quadrirotore Closed-Loop 
**Riferimento Teorico:** Mellinger & Kumar (Minimum Snap Trajectory Generation and Control)
**Stato Attuale:** Pipeline completa implementata e funzionante (Pianificazione -> Controllo -> Motore Fisico).

## 1. Architettura di Sistema e Flusso Dati
Il progetto modella un quadrirotore sfruttando la proprietà di **Piattezza Differenziale (Differential Flatness)**. Lo stato del drone (12 variabili) e gli ingressi di controllo (4 rotori) sono mappati algebricamente su 4 uscite piatte: $[x, y, z, \psi]$ e le relative derivate (fino allo Snap per la posizione, fino all'accelerazione per lo Yaw). Il sistema non richiede l'integrazione di equazioni differenziali complesse per la generazione dei comandi di base.

## 2. Modulo di Pianificazione (Trajectory Generation)
Generazione offline di traiettorie polinomiali multi-segmento tramite Programmazione Quadratica (QP).

*   **Formulazione QP (`quadprog`):** Minimizzazione dell'integrale della derivata k-esima al quadrato (Snap per X, Y, Z con $k=4$; Accelerazione per Yaw con $k=2$). Il problema è risolto in modo disaccoppiato per le 4 dimensioni.
*   **Adimensionalizzazione (Scaling):** Implementazione di fattori di scala spaziali ($\beta_1$ per lo shift, $\beta_2$ per l'ampiezza) e temporali per garantire il condizionamento numerico ottimale delle matrici $H$ e $A_{eq}$.
*   **Vincoli di Diseguaglianza (Safe Corridors):** Implementati tramite la variabile `config.corridor_delta`. Permette di forzare la traiettoria all'interno di tolleranze spaziali specifiche campionando punti intermedi nei segmenti, gestibile dinamicamente tramite valori `Inf` per i segmenti liberi.
*   **Ottimizzazione Temporale Adattiva:** Discesa del gradiente numerico per l'ottimizzazione della durata dei segmenti ($T_i$). Include un approccio *Backtracking Line Search*. 
*   **Logica "Best State":** Aggiunta per prevenire l'instabilità numerica (Overshooting) nelle ultime iterazioni del gradiente. L'algoritmo salva costantemente la combinazione di tempi/costo migliore e la ripristina se le iterazioni successive divergono irrigidendo la traiettoria.

## 3. Modulo di Controllo (Geometric Controller su $SO(3)$)
Controllo non lineare dell'assetto svincolato dagli angoli di Eulero per prevenire il *Gimbal Lock*.

*   **Tuning Analitico (Pole Placement):** Sostituzione dei guadagni scalari base con matrici diagonali ($\mathbf{K}_p, \mathbf{K}_v, \mathbf{K}_R, \mathbf{K}_\omega$). I guadagni sono calcolati analiticamente in fase di setup basandosi su massa ($m$), matrice di inerzia ($\mathbf{J}$), pulsazione naturale ($\omega_n$) e smorzamento ($\zeta$). L'Inner Loop (assetto) è tarato per essere significativamente più veloce (es. $\omega_{n,att} = 15.0$ rad/s) dell'Outer Loop (posizione, $\omega_{n,pos} = 3.0$ rad/s).
*   **Tilt-Limit Integrato:** Inserito un limite rigido all'inclinazione massima (es. 45°). Se la forza orizzontale richiesta $F_{des}(1:2)$ supera $F_{des}(3) \cdot \tan(45^\circ)$, il vettore orizzontale viene scalato. Questo garantisce che la componente verticale $u_1$ sia sempre sufficiente a contrastare la gravità, prevenendo cadute repentine durante manovre con ritardo di tracking.
*   **Feed-Forward dello Snap:** Implementazione completa dell'equazione dei momenti torcenti ($\tau$) del paper originale. Oltre al feedback PD e alla compensazione dell'effetto giroscopico ($\boldsymbol{\omega} \times \mathbf{J}\boldsymbol{\omega}$), è stato aggiunto il termine $+\mathbf{J}\dot{\boldsymbol{\omega}}_{des}$. L'accelerazione angolare desiderata è calcolata algebricamente proiettando il vettore Snap e la derivata della spinta ($\dot{u}_1$, dal Jerk), garantendo reattività anticipatoria nelle curve strette.
*   **Monitoraggio Stabilità Esponenziale:** Funzione di alert integrata basata sulla funzione di errore $\Psi = \frac{1}{2} \text{tr}(\mathbf{I} - \mathbf{R}_{des}^T \mathbf{R})$. Avvisa con cooldown se l'errore di assetto supera i 90° ($\Psi \ge 1.0$) o i 180° ($\Psi \ge 2.0$).

## 4. Modulo Fisico e Gestione della Sicurezza (Plant & Feasibility)
Ponte tra astrazione matematica e limiti hardware.

*   **Matrice di Mixing e Saturazione:** Conversione dei comandi ideali $(u_1, u_2, u_3, u_4)$ in regimi di rotazione ($\omega_1, \ldots, \omega_4$) per i singoli rotori, con applicazione di limiti fisici di taglio (RPM max/min). Simula sbandamenti asimmetrici realistici se i limiti vengono superati.
*   **Check di Fattibilità con "Control Authority" (Headroom):** Controllo preventivo della traiettoria *prima* della simulazione dinamica. Non si limita a verificare se $u_1 \le u_{max}$. Riserva obbligatoriamente un margine (es. 20%) della spinta massima totale esclusivamente per la generazione di coppia (Torque). Se la traiettoria richiede $> 80\%$ dei motori solo per traslare, viene marcata come infattibile per prevenire la saturazione asimmetrica durante il rollio/beccheggio.
*   **Auto-Relax Temporale (Scalo Temporale):** Se il *Feasibility Check* fallisce, il sistema applica autonomamente una dilatazione progressiva (es. $+5\%$) al vettore dei tempi assoluti. Sfruttando le leggi di scala ($a \propto 1/c^2$, $cost \propto 1/c^7$), la traiettoria viene rilassata istantaneamente senza dover rieseguire l'intera ottimizzazione QP, finché non rientra nei limiti fisici dei motori.

## 5. Deviazioni e Miglioramenti rispetto al Paper Originale
Dettagli architetturali introdotti per garantire stabilità nel simulatore che non sono esplicitati in Mellinger & Kumar:
1.  **Headroom del 20%:** Aggiunto perché il limite teorico di spinta pura porta infallibilmente allo schianto nella simulazione dinamica per mancanza di margine di correzione.
2.  **Tilt-Limit sul calcolo di $F_{des}$:** Modifica architetturale fondamentale inserita a monte del calcolo dell'asse $z_{B,des}$.
3.  **Memoria del gradiente (Best State):** Soluzione introdotta per la gestione dell'overshooting nell'ottimizzazione dei segmenti temporali.
4.  **Matrici vs Scalari:** Abbandono sistematico dei guadagni scalari suggeriti in forma base, in favore di un posizionamento dei poli rigoroso agganciato al tensore di inerzia reale del modello.