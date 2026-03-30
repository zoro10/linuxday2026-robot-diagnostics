# VM Diagnostics Demo

Progetto laboratorio per LinuxDay 2026.

## Obiettivo
Passare da monitoraggio a diagnostica su una VM Linux, usando:
- applicazione PHP custom
- MariaDB
- reverse proxy Nginx
- scaling dinamico dei frontend web
- futura integrazione con Netdata e score engine

## Topologia attuale
proxy -> N web -> db

## Componenti
- compose.yaml -> stack runtime
- web/ -> applicazione PHP
- db/init/ -> inizializzazione database
- proxy/nginx.conf -> configurazione proxy
- scripts/render_nginx.sh -> genera upstream Nginx
- scripts/scale_web.sh -> scala i container web e aggiorna il proxy

## Endpoint applicativi
- / -> pagina HTML leggibile
- /health.php -> check tecnico JSON
- /dbcheck.php -> check database
- /info.php -> info applicative

## Avvio base
docker compose up -d

## Scaling
Esempi:
./scripts/scale_web.sh 1
./scripts/scale_web.sh 2
./scripts/scale_web.sh 3

## Stato attuale
- runtime validato su VM target
- scaling validato
- round-robin validato
- progetto in preparazione per pubblicazione GitHub

## Prossimi passi
- ulteriore pulizia repo
- pubblicazione GitHub
- HTTPS/SSL
- Netdata
- score engine
