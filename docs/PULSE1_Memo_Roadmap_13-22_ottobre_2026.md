# PULSE-1 — Checkpoint progettuale e roadmap delle due demo

**Decisione:** 13 ottobre 2026  
**Evento:** LinuxDay 2026, sabato 24 ottobre  
**Sviluppo:** 14–22 ottobre; **23 ottobre:** freeze e prova generale  
**Repository:** `git@github.com:zoro10/linuxday2026-robot-diagnostics.git`  
**Ultimo commit verificato nella sessione:** `02cdfde` — *Document PULSE-1 navigation missions and roadmap to Oct 22*, su `main` sincronizzato.  
**Stato di questo documento:** memo aggiornato pronto da aggiungere a Git; **non ancora pubblicato nel repository**.

## 1. Decisione principale: due demo pubbliche, una sola piattaforma

**PULSE-1 è il protagonista.** Le due dimostrazioni sono complementari e devono entrambe essere mostrate al pubblico. La seconda **non sostituisce** la prima.

### Demo 1 — «Il robot sta bene?» (baseline già funzionante)

**Tema:** osservabilità, diagnostica, sicurezza e recupero controllato.

Sequenza pubblica prevista:

1. Avvio di Gazebo, ROS 2, PULSE-1 e delle due dashboard; stato iniziale dei sensori e punteggi Robot Health / Platform Health.
2. Il rover si muove e produce telemetria; il pubblico osserva sensori, metriche e stato generale.
3. Introduciamo un guasto **controllato**, preferibilmente interrompendo il bridge LiDAR dedicato.
4. Il collector e l'API segnalano il problema; il controller entra in `NO_LIDAR` e comanda velocità zero.
5. Ripristiniamo **manualmente** il componente e osserviamo il ritorno dei dati e il comportamento previsto.
6. Mostriamo `FAULT`, `RECOVERY` e durata nel Diagnostic Event Journal.

**Già verificato:** architettura in esecuzione, API e due dashboard raggiungibili, Health Score separati, test su collector/API/LiDAR/controller, storico persistente con 34 eventi all'ultimo controllo, prova di stabilità Gazebo di 60 minuti e ripristino dopo riavvio. **Da completare:** prova visiva delle dashboard durante l'errore e sceneggiatura della dimostrazione. Non affermare che il sistema ripari automaticamente tutti i guasti: oggi rileva e consente ripristini guidati/manuali.

**Vincolo:** non compromettere questa baseline nello sviluppo della Demo 2. Effettuare regressioni giornaliere brevi su `/api/status`, `/api/events`, dashboard, sensori, sicurezza e persistenza dello storico.

### Demo 2 — «Che cosa vede e decide PULSE-1?» (da sviluppare)

**Tema:** sensori, percezione, ragionamento locale e navigazione autonoma controllata.

L'operatore sceglie **scenario** e **modalità** prima dell'avvio, poi assegna una destinazione. PULSE-1 riceve il goal, i dati dei sensori e l'odometria; **non riceve la geometria reale di Gazebo né una direzione di aggiramento preimpostata**.

- **STOP:** raggiungi il goal se libero; al primo ostacolo che blocca il percorso, fermati e dichiara `STOP_PER_OBSTACLE`.
- **AVOID:** raggiungi il goal cercando un varco dai settori LiDAR, evitando gli ostacoli e riallineandoti alla destinazione; dichiara `GOAL_REACHED` soltanto quando il goal è realmente raggiunto.

**Goal:** punto posto 10 m davanti alla posa iniziale (non 10 m di strada percorsa). Con yaw iniziale `θ`: `goal = (x0 + 10 cos θ, y0 + 10 sin θ)`. Tolleranza finale proposta: **0,5 m**. `NO_LIDAR`, `NO_PATH` e `TIMEOUT` devono dare luogo a un arresto sicuro, non a movimenti ciechi.

## 2. Scenari selezionabili (stessa logica di navigazione)

| Scenario | Geometria introdotta **soltanto in Gazebo** | Scopo |
|---|---|---|
| **S1 — Ostacolo frontale** | Un ostacolo sul tragitto | Dimostrare STOP e primo aggiramento AVOID |
| **S2 — Fronte + sinistra** | Ostacolo davanti, secondo a sinistra circa +45° | La scelta di passare a destra deve emergere dal LiDAR |
| **S3 — Fronte + destra** | Scenario speculare a S2 | Verificare che non esista una svolta a destra hardcoded |
| **S4 — Ostacoli multipli** | Ostacoli sfalsati lungo il percorso | Estensione, solo se i primi tre scenari sono stabili |

**Otto combinazioni teoriche:** S1–S4 × STOP/AVOID; **non promettere otto dimostrazioni robuste entro il 22**. Obiettivo obbligatorio: S1 e S2 in entrambe le modalità, con S3 come verifica di generalità; S4 è un bonus.

**Nota geometrica:** con un ostacolo iniziale a **1 m** e soglia di arresto frontale a **1,2 m**, la modalità STOP si fermerà immediatamente. Per mostrare un avvicinamento prima dello STOP, preparare una variante con ostacolo inizialmente più distante, ad esempio 2,5–3 m; mantenere il confronto STOP/AVOID nello stesso scenario quando presentato come confronto diretto.

## 3. Dashboard: ruoli distinti, nessuna informazione inventata

**Dashboard A — PULSE-1 Diagnostics (`/dashboard/`):** rimane sostanzialmente invariata; Robot Health, Platform Health, CPU/RAM/disco/load Linux, servizi, collector/API/bridge, sensori ed Event Journal. Il pannello **Linux Simulation Host** resta qui.

**Dashboard B — PULSE-1 Rover Cockpit (`/dashboard/rover/`):** evolve in quattro viste simultanee:

| Vista | Domanda | Dato mostrato e provenienza |
|---|---|---|
| **Sensor Health** | «Quanto funziona ogni sensore?» | LiDAR, IMU e odometria: 0–100%, OK/DEGRADED/FAIL/UNKNOWN, Hz, age, causa del degrado; calcolo esplicito dal collector |
| **Sensor Readings** | «Che cosa misurano?» | Distanze per settore LiDAR, posizione, velocità, orientamento e, se esposte, misure IMU da messaggi ROS |
| **Perception** | «Che ambiente percepisce?» | Settori occupati/liberi, ostacoli e rappresentazione 2D basata esclusivamente su `LaserScan` |
| **Robot Decision** | «Che cosa ha capito e cosa decide?» | Modalità, goal, stato del controller, direzione scelta, motivo, `cmd_vel` e distanza residua, **pubblicati dal controller** |

**Regola per Sensor Health:** 100% significa che il sensore soddisfa i controlli definiti, non che sia fisicamente perfetto; evitare di trasformare direttamente una frequenza nominale in percentuale di affidabilità. Distinguere assenza completa, riduzione di frequenza, dato obsoleto e impossibilità di valutazione (`UNKNOWN`). Un ostacolo rilevato **non è un guasto**.

## 4. Architettura e vincoli da preservare

- Gazebo Harmonic simula la scena; ROS 2 Jazzy, bridge principale e bridge LiDAR dedicato trasportano misure e comandi. Niente ground truth dell'ambiente nel controller.
- Controller PULSE-1: macchina a stati STOP/AVOID, gestione goal, scelta locale basata su settori LiDAR, comandi `linear.x` e `angular.z`, stop sicuro.
- Collector: acquisizione dei sensori, Sensor Health e stato decisionale; scrittura JSON atomica; Event Journal persistente.
- API diagnostica: espone dati e diagnosi, resta indipendente da ROS 2. Dashboard visualizzano soltanto ciò che proviene da processi reali.
- Carico VM limitato (**2 vCPU**): preferire controllo locale leggero; niente introduzione frettolosa di Nav2/SLAM. Mantenere Gazebo, collector e servizi esistenti stabili.
- Durante il lavoro adottare **massimo tre comandi e un controllo** per blocco operativo, con commit piccoli e verifiche di regressione.

## 5. Roadmap riorganizzata: 14–22 ottobre

| Giorno | Priorità | Uscita verificabile |
|---|---|---|
| **Mer 14/10 — Fondazioni** | Backup baseline Demo 1; controllare fisica e sterzata `angular.z`, pose/yaw, FOV e settori LiDAR; scenari S1/S2 ripetibili | Rover sterza realmente, distingue fronte/sinistra/destra, reset mondo affidabile; primo commit tecnico |
| **Gio 15/10 — STOP e missione** | Contratto missione (`goal`, modalità), comando di selezione pre-run, stato esplicito e telemetria della decisione; STOP verificato in S1 e S2 | `STOP_PER_OBSTACLE`, velocità zero e nessuna collisione; baseline Demo 1 ancora funzionante |
| **Ven 16/10 — AVOID MVP** | Primo controllo locale con scansioni per settore, selezione varco, sterzata, avanzamento e riallineamento alla meta | Un aggiramento reale in S1, senza svolta hardcoded e senza intervento umano |
| **Sab 17/10 — Due ostacoli** | Stabilizzare AVOID in S2 e verificare S3 speculare; goal raggiunto da odometria | Almeno un run S2 `GOAL_REACHED` entro 0,5 m, nessuna collisione; S3 come test contro hardcoding |
| **Dom 18/10 — Gate navigazione** | Tre run consecutivi del percorso target, correzione oscillazioni, timeout, `NO_LIDAR` e `NO_PATH`; reset e logging | **GO/NO-GO AVOID**: se instabile, ridurre a S1 e/o S2 affidabili. S4 non prioritario |
| **Lun 19/10 — Dati e qualità sensori** | Sensor Health 0–100 con guasti parziali/totali; esportare letture LiDAR/IMU/odometria e intenzioni del controller | Test 100/degradato/0/UNKNOWN, API coerente, non regressione di entrambi gli Health Score e dello storico |
| **Mar 20/10 — Rover Cockpit** | Quattro viste, visualizzazione selettore/modalità e decisioni reali; mantenere Diagnostics invariata | STOP e AVOID osservabili da browser, gestione `STALE` e assenza API. **GO/NO-GO UI** |
| **Mer 21/10 — Due demo integrate** | Sequenza **Demo 1 completa → reset → Demo 2**; prove laptop/VM, accesso remoto, rete/VPN ove necessario, video e fallback offline | Entrambe le demo eseguibili senza interventi non previsti; prove di guasto e recupero ancora superate |
| **Gio 22/10 — Collaudo pubblico** | Tre sequenze complete ripetibili, tempi della presentazione, checkpoint Git, backup e scelta scenari definitivi | **Approvazione tecnica delle due demo** e documentazione della versione da portare all'evento |

**Ven 23/10 — Freeze:** solo prove generali, backup e fix bloccanti; nessuna nuova funzionalità.  
**Sab 24/10 — LinuxDay:** eseguire Demo 1 e Demo 2, con S1/S2 come percorsi principali e S3/S4 soltanto se certificati.

## 6. Criteri di accettazione e fallback

**Demo 1 approvata se:** guasto LiDAR visibile, robot in `NO_LIDAR` e `cmd_vel = 0`, recovery manuale riuscito, Event Journal integra coppia FAULT/RECOVERY, dashboard e servizi principali funzionanti.

**Demo 2 approvata se:** modalità e scenario selezionabili; il controller decide soltanto dai sensori; STOP non collidere; AVOID supera gli ostacoli e arriva al goal entro 0,5 m; il cockpit mostra letture, qualità sensori, percezione e decisione; perdita LiDAR o mancanza di strada termina in sicurezza.

**Ordine dei tagli se manca tempo:** eliminare S4, poi S3 dalla demo pubblica, poi ridurre AVOID al solo S1 affidabile; non eliminare Demo 1 e non simulare visivamente un'intelligenza che il controller non possiede. Non sacrificare i controlli sicurezza per mostrare la navigazione.

**Definizione di successo narrativo:** prima facciamo vedere che *il robot è osservabile, rileviamo un guasto e possiamo recuperare*; poi che *il robot vede, valuta e decide come muoversi*. La Demo 2 amplia la storia, non nasconde la Demo 1.

## 7. Git e checkpoint

Questo memo **sostituisce come pianificazione operativa** il contenuto del documento precedente `docs/PULSE1_Memo_Roadmap_13-22_ottobre_2026.md`, senza cancellare la storia Git. La versione più recente verificata del progetto è il commit `02cdfde`. La nuova pianificazione non è ancora stata applicata sulla VM o registrata con un commit.

**Checkpoint documentale suggerito:** aggiornare quel medesimo file di `docs/`, poi fare `git diff --check`, `git diff`, commit `Update PULSE-1 roadmap for two public demos and scenarios` e push. Una volta eseguito il push, il nuovo hash diventa il punto di ripartenza ufficiale.
