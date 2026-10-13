# PULSE-1 — Memo di progetto e roadmap riorganizzata

**Data della decisione:** 13 ottobre 2026  
**Evento:** LinuxDay, 24 ottobre 2026  
**Orizzonte di sviluppo:** 14–22 ottobre 2026  
**23 ottobre:** freeze, backup e prova generale, senza nuove funzionalità  
**Repository:** `git@github.com:zoro10/linuxday2026-robot-diagnostics.git`  
**Baseline Git verificata:** `27fe28c` — `Separate robot and platform health scoring` (`main` sincronizzato con `origin/main` al 13/10).  
**Stato del presente documento:** proposta di checkpoint documentale; **non ancora commit né push su GitHub**.

## 1. Decisione da preservare

**PULSE-1 è il protagonista.** La diagnostica serve a rendere comprensibile la sua esperienza, non a sostituirsi al rover. Il robot deve interpretare l'ambiente **esclusivamente dalle misure dei propri sensori e dall'odometria**, senza ricevere dal simulatore la mappa degli ostacoli, la loro posizione o una svolta precalcolata. Gazebo simula la fisica e genera i dati sensoriali; non suggerisce la soluzione.

Si mantengono le due dashboard:

- **PULSE-1 Diagnostics (`/dashboard/`)**: quadro generale della piattaforma, Robot Health, Platform Health, Linux host, servizi, nodi, sensori, cronologia FAULT/RECOVERY. Mantenerla sostanzialmente invariata.
- **PULSE-1 Rover Cockpit (`/dashboard/rover/`)**: evolvere la pagina del rover come vista principale della demo, articolata in **stato dei sensori**, **risultati dei sensori**, **percezione spaziale da scansioni LiDAR**, **interpretazione / decisione dichiarata dal controller**.

Le percentuali **Sensor Health 0–100** descrivono una **valutazione operativa convenzionale**, non la probabilità di funzionamento o l'accuratezza metrologica. Definire soglie, tempi, cause e comportamento `UNKNOWN` prima di implementarle. Non ricavare un 88% semplicemente da una frequenza di 4,4 Hz su 5 Hz.

## 2. Due missioni, stesso ambiente

**Scenario comune di partenza:** PULSE-1 con ostacolo a circa **1 m davanti** e secondo ostacolo a circa **1 m verso sinistra, a +45°** (posizioni del mondo fissate **solo nello scenario Gazebo**). Spazio libero verso destra e per un percorso alternativo, da verificare considerando ingombro del rover e margini di sicurezza.

**Obiettivo della missione:** arrivare a un punto **10 m in avanti rispetto alla posa iniziale**, non percorrere soltanto 10 m di traiettoria. Alla partenza si fissa un *goal* nel frame del mondo: `goal_x = x0 + 10*cos(yaw0)`, `goal_y = y0 + 10*sin(yaw0)`. L'odometria serve a misurare il progresso e verificare il raggiungimento, con tolleranza proposta `<= 0,5 m`.

| Modalità | Ordine impartito all'inizio | Comportamento atteso | Esito corretto |
|---|---|---|---|
| **STOP** | «Vai 10 m avanti; se trovi un ostacolo fermati» | Avanza; al rilevamento di ostacolo frontale entro la soglia di sicurezza si arresta | **STOP_PER_OBSTACLE** documentato; non è un guasto del robot e non raggiunge necessariamente la meta |
| **AVOID** | «Vai 10 m avanti; se trovi ostacoli cerca autonomamente una via libera» | Valuta i settori del LiDAR, sceglie la direzione libera, aggira gli ostacoli e ripunta al goal | **GOAL_REACHED** quando la distanza dalla meta è `<= 0,5 m`, senza collisioni |

**Non programmare la svolta a destra.** Nello scenario la destra dovrebbe emergere come scelta conveniente perché davanti e a sinistra sono occupati; con geometria diversa la decisione potrebbe cambiare. Non confondere navigazione locale reattiva con SLAM, mappatura globale o riconoscimento semantico degli oggetti.

## 3. Informazioni visibili nel Rover Cockpit

| Vista | Contenuto richiesto | Provenienza verificabile |
|---|---|---|
| **Sensor Health** | LiDAR / IMU / odometria: punteggio operativo 0–100, stato OK/DEGRADED/FAIL/UNKNOWN, Hz effettivi, freschezza e motivo del degrado | Collector e valutatore; stessa metodologia per ciascun sensore |
| **Sensor Readings** | Distanze LiDAR per settori, posizione X/Y, orientamento, velocità, accelerazioni e velocità angolari IMU se esposte | Messaggi ROS reali (`LaserScan`, `Odometry`, `Imu`) |
| **Perception** | Rover 2D e scansioni LiDAR, occupazione frontale/sinistra/destra e passaggi liberi con grado di incertezza | Scansioni LiDAR ricevute, non geometria ground-truth Gazebo |
| **Robot Decision** | Missione STOP/AVOID, goal, stato macchina, settore scelto, comando velocità/rotazione, motivo e distanza residua | Controller ROS 2, pubblicato dal controller e letto dal collector |
| **Platform Overview** | Linux CPU/RAM/disco/load, servizi, API, collector, bridge, Event Journal, punteggi Robot/Platform Health | Dashboard diagnostica attuale, da preservare |

Una missione terminata correttamente in **STOP_PER_OBSTACLE** non deve penalizzare Robot Health. `NO_LIDAR` deve imporre immediatamente velocità nulla, indipendentemente dalla modalità. Valori obsoleti devono diventare `UNKNOWN`, mai restare apparentemente live.

## 4. Architettura funzionale minima

- **Gazebo Harmonic + modello PULSE-1:** scenario, ostacoli e simulazione; nessun dato privilegiato al controller.
- **Bridge ROS 2 Jazzy:** `LaserScan`, `Odometry`, `Imu`, `cmd_vel`. Il bridge LiDAR rimane separato per consentire guasti controllati.
- **Controller PULSE-1:** missione `STOP` o `AVOID`, target fissato all'avvio, macchina a stati, controllo geometrico e comando `cmd_vel`. Stati candidati: `READY`, `GO_TO_GOAL`, `OBSTACLE_DETECTED`, `AVOIDING`, `REJOINING_GOAL`, `GOAL_REACHED`, `STOP_PER_OBSTACLE`, `NO_LIDAR`, `NO_PATH`, `TIMEOUT`.
- **Collector:** aggiungere misure sensoriali utili, stato del controller/missione e Sensor Health 0–100 senza interrompere la raccolta attuale.
- **Diagnostic API:** aggiungere dati in sola lettura al payload già esistente; conservare funzionalità di status/eventi e isolamento dei servizi.
- **Dashboard rover:** aggiungere i quattro riquadri; nessuna logica che 'indovina' una decisione invece di leggere quella pubblicata dal controller.
- **Selettore missione:** preferenza per una scelta esplicita prima del run e indicazione chiara nella dashboard. **MVP affidabile:** CLI/script sul terminale che imposta la modalità e avvia la missione; **opzionale:** pulsante web soltanto se la catena di comando è sicura e collaudata. Non esporre comandi arbitrari tramite l'API pubblica.

## 5. Scaletta dal 14 al 22 ottobre — obiettivi giornalieri verificabili

| Data | Lavoro principale | Deliverable / test di accettazione |
|---|---|---|
| **Mer 14/10** | Validazione capacità di sterzata (`angular.z`), orientamento da odometria, ampiezza/FOV e settori LaserScan; predisposizione mondo con ostacolo frontale e altro a +45° sinistra; backup scenario attuale. | Rover ruota realmente; settori LiDAR distinguibili; scena a due ostacoli ripetibile; nessuna collisione di spawn. Se sterzata impossibile, priorità assoluta a correggere modello/controllo. |
| **Gio 15/10** | Definire missione (goal a +10 m nel frame iniziale), comando STOP/AVOID, stato e logging espliciti; implementare **STOP** con arresto sicuro. | Selezione STOP, partenza, ostacolo identificato e arresto prima della collisione; stato `STOP_PER_OBSTACLE` esportabile; ripartenza della demo ripetibile. |
| **Ven 16/10** | Prima versione **AVOID**: scelta da settori LiDAR, controllo `linear.x`/`angular.z`, margine di collisione, macchina a stati per aggirare e riallinearsi al goal. | Evitamento di un ostacolo, senza direzione hardcoded; distanze e decisioni registrate. Eventuali funzioni originali della dashboard già realizzate restano attive. |
| **Sab 17/10** | Prova **AVOID con due ostacoli**: fronte a ~1 m, sinistra +45° a ~1 m; il rover deve individuare il varco a destra e poi tornare verso il goal. | Almeno un run che supera entrambi gli ostacoli e raggiunge il goal entro 0,5 m senza collisioni; se fallisce, raccogliere log/pose e correggere oscillazione o clearance. |
| **Dom 18/10** | Stabilizzare navigazione: timeout, blocco/oscillazione, assenza LiDAR, nessun settore sicuro, reset dello scenario, tuning delle velocità. | Tre run consecutivi AVOID sullo stesso scenario senza collisioni e senza interventi manuali; in caso di situazione insolubile arresto sicuro `NO_PATH`/`TIMEOUT`. **Primo checkpoint GO/NO-GO.** |
| **Lun 19/10** | Sensor Health individuale 0–100: regole esplicite di age/rate/validità, guasto totale/parziale, `UNKNOWN`; esportare letture IMU/LiDAR/odometria necessarie e stato missione/decisione. | Test sintetici 100 / degradato / 0 / UNKNOWN; API JSON coerente, nessuna regressione Robot/Platform Health e FAULT/RECOVERY. |
| **Mar 20/10** | UI del **Rover Cockpit**: quattro viste Sensor Health, Sensor Readings, Perception LiDAR e Robot Decision; modalità visibile; continuità della dashboard di piattaforma. | Run STOP/AVOID osservabile sul browser, decisioni provenienti dal controller, dati stale gestiti; demo fruibile su schermo laptop. **Secondo checkpoint GO/NO-GO.** |
| **Mer 21/10** | Prove end-to-end e operatività evento: avvio/ripristino, networking laptop↔VM e accesso remoto/VPN ove previsto, fallback offline, reset tra missioni, registrazione evidenze. | Sequenza STOP→reset→AVOID ripetuta; log e screenshot; procedure di recovery testate; nessun nuovo componente introdotto in caso di instabilità. |
| **Gio 22/10** | **Prova generale e certificazione demo**: cronometria dei due scenari, verifica raggiungimento goal, sicurezza, monitoraggio, fallimenti previsti; preparare storyboard per il pubblico. | Almeno tre dimostrazioni complete e ripetibili (STOP + AVOID), console e browser puliti, backup e Git sincronizzati; decisione di congelamento delle funzionalità. |

**Ven 23/10:** freeze, backup, verifica finale e prova parlata; solo bugfix bloccanti.  
**Sab 24/10:** LinuxDay — demo pubblica.

I blocchi collector/API, dashboard diagnostica, timeline eventi, stabilità 60 min e prove di reboot già completati il 13/10 **non vanno rifatti come sviluppo**: rimangono controlli di regressione e riserva di tempo. La scaletta pregressa del 20–23 (accesso laptop, fallback, prove e freeze) è conservata, concentrando la nuova funzionalità entro il 20.

## 6. Criteri di successo e sicurezza

1. **Niente conoscenza privilegiata dell'ambiente:** nessuna coordinata ostacolo fornita al controller; solo goal + telemetria ROS.
2. **STOP:** al fronte bloccato il comando finale è 0 m/s e lo stato è `STOP_PER_OBSTACLE`, non Robot Health FAIL.
3. **AVOID:** il percorso è scelto da sensori e odometria; si considera riuscito solo con `GOAL_REACHED`, errore di posizione finale `<= 0,5 m`, nessuna collisione e nessun comando manuale durante la corsa.
4. **Fail-safe:** LiDAR mancante/non fresco -> `NO_LIDAR` + `cmd_vel` zero; nessun varco/timeout -> stop con stato esplicito. Con 0,18 m/s nominali, prevedere timeout iniziale di almeno 180 s per un goal a 10 m con deviazioni, da verificare sul campo.
5. **Test sensori:** qualità 0–100 visibile e giustificata; guasto totale/parziale distinguibili; non trattare un ostacolo come guasto del LiDAR.
6. **Non regressione:** `/api/status`, `/api/events`, entrambe le dashboard, Robot Health, Platform Health, 34 eventi storici, processi tmux, Docker e sicurezza d'isolamento restano integri.
7. **Reset ripetibile:** stesso punto iniziale, stesso mondo, stesso goal; prima di ogni confronto reset dello scenario senza cancellare lo storico diagnostico.

## 7. Rischi e regola di riduzione dello scope

**Vincolo dominante: 2 vCPU su Proxmox, Gazebo + ROS 2 + collector già con CPU elevata.** Evitare stack di navigazione pesanti, SLAM e Nav2 in questa finestra, salvo evidenza chiara che il carico lo consenta. Preferire un planner locale semplice, testabile, con macchina a stati e logging.

- **14/10:** se il robot non sterza realmente o il LiDAR non fornisce direzioni laterali, questo è il blocco critico da risolvere prima di qualsiasi dashboard.
- **18/10 sera:** se l'AVOID a due ostacoli non è ripetibile, mettere in sicurezza prima l'aggiramento di un solo ostacolo, dichiarandolo esplicitamente come livello dimostrativo ridotto.
- **20/10 sera:** se l'autonomia resta instabile, mantenere STOP + percezione/decisione come dimostrazione affidabile e mostrare AVOID soltanto con un caso collaudato; mai presentare una scelta forzata come navigazione autonoma.
- **Dal 22/10:** niente nuove feature; privilegiare ripetibilità, osservabilità e narrativa.

## 8. Checkpoint e tracciabilità Git

**Checkpoint sorgente già esistente (verificato):** `27fe28c` — separazione Robot Health e Platform Health. **Non rappresenta la nuova roadmap**.

**Checkpoint progettuale proposto da creare nel repo** (soltanto quando questo file viene copiato nella VM):

```bash
cd ~/vmdiag-app
mkdir -p docs
# Copia il file scaricato in docs/PULSE1_Memo_Roadmap_13-22_ottobre_2026.md
git add docs/PULSE1_Memo_Roadmap_13-22_ottobre_2026.md
git commit -m "Document PULSE-1 mission modes and roadmap to Oct 22"
git push origin main
git status -sb
```

**Regola dei checkpoint di implementazione:** commit piccoli dopo scenario/sensori, STOP, AVOID MVP, hardening, Sensor Health, Cockpit e collaudo; aggiungere tag/demo baseline a freeze se utile. Non dichiarare completata una funzionalità non testata.

---

**Decisione sintetica:** dal 14 al 22 ottobre la priorità è dimostrare due missioni a 10 m nello stesso ambiente, con comportamento STOP o aggiramento realmente autonomo tramite LiDAR; rendere visibili sensori, affidabilità, interpretazione e decisione del rover; conservare la dashboard della piattaforma e chiudere con prove ripetibili e freeze il 23 ottobre.
