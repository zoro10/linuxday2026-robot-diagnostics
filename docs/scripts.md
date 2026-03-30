# Uso degli script

## render_nginx.sh
Genera il file proxy/nginx.conf in base al numero di backend web desiderati.

### Esempi
./scripts/render_nginx.sh 1
./scripts/render_nginx.sh 2
./scripts/render_nginx.sh 3

### Funzione
Aggiorna il blocco upstream app_backend di Nginx con:
- vmdiag-app-web-1
- vmdiag-app-web-2
- vmdiag-app-web-3
- ecc.

## scale_web.sh
Script operativo principale.

### Funzione
1. genera nginx.conf
2. scala il servizio web
3. controlla il numero di istanze attive
4. aspetta che il proxy sia running
5. verifica la configurazione Nginx
6. ricarica il proxy
7. stampa lo stato finale

### Esempi
./scripts/scale_web.sh 1
./scripts/scale_web.sh 2
./scripts/scale_web.sh 3

## Sequenza di test consigliata
./scripts/scale_web.sh 1
./scripts/scale_web.sh 3
./scripts/scale_web.sh 2

## Output atteso
- numero di istanze coerente
- health OK
- round-robin coerente con il numero di backend attivi
