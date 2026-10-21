# PULSE-1 — Checkpoint 22 ottobre 2026

## Evento
Linux Day 2026: 24 ottobre.
Progetto: PULSE-1, ROS 2 Jazzy + Gazebo Harmonic.
Scenario: S1 AVOID, navigazione intorno all'ostacolo.

## Navigazione validata
- 28 avanzamenti reali riusciti.
- Riarmi Guard/Geofence e arresti indipendenti verificati.
- Primo waypoint raggiunto.
- Ultima posa Gazebo verificata: (2.36122, 0, 0).
- Primo waypoint: (2.368566677973659, 0).
- Il planner propone TURN LEFT, waypoint 1.
- Zero rotazioni reali riuscite.

## Ultimo tentativo TURN
- Segmento: v6-route-f2bfd1512bae91e646dc.
- Riarmo completato.
- Abort: START_SENSORS_OR_CLEARANCE.
- Nessuna DUAL AUTH e nessuna rotazione confermata.
- Geofence PAUSE con pause_ack=True.
- Guard BLOCKED, emergency_latched=False.
- Non riarmare automaticamente.

## Analisi sensori
- Observer in WAIT_SENSORS durante diagnosi a Gazebo PAUSED.
- I dati mancanti dopo l'abort non provano che fossero
  assenti durante il warm-up.
- turn_clear() richiede sei settori noti e distanza
  minima di 3.20 m.
- Serve verificare dati LiDAR freschi durante il warm-up.

## Geometria
- Modelli trovati: model.sdf e model_demo2.sdf.
- Chassis: 2.01142 x 1.0 m.
- Raggio conservativo collisioni: 1.27457 m.
- Ostacolo fisico: inizio X=4.5 m.
- Rover: X=2.36122 m.
- Separazione fisica conservativa: 0.86421 m.
- Keep-out inflazionato: X minimo 2.8186 m.
- Confermare il MODEL_FILE caricato e la validità
  del percorso rispetto all'ingombro.

## Modifiche tecniche della sessione
- Ripristinato bridge principale ROS/Gazebo.
- Recuperati quattro publisher sensori.
- Mitigazione limitata del physics overrun nel warm-up.
- Retry limitati per lettura gazebo_pose().
- Modalità opzionale PRECISION_WP0 testata offline.
- Primo waypoint raggiunto con precisione WP0.

## Priorità alla ripresa
1. Verificare MODEL_FILE e ingombro effettivo.
2. Definire/testare una clearance TURN geometrica sicura.
3. Verificare Observer con dati freschi.
4. Gestire il recupero controllato del Geofence PAUSE.
5. Validare il primo TURN reale, massimo 12 gradi.
6. Completare la prima curva di 90 gradi.
7. Proseguire verso l'aggiramento dell'ostacolo.
8. Collegare l'avvio missione alla dashboard.

## Sicurezza
Non resettare latch, riarmare, riavviare nodi o
muovere il rover senza un nuovo preflight.
Non trattare il calcolo geometrico offline come
prova definitiva di clearance.
