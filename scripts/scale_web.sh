#!/usr/bin/env bash
set -euo pipefail

COUNT="${1:-1}"
PROJECT_DIR="${HOME}/vmdiag-app"
COMPOSE_FILE="${PROJECT_DIR}/compose.yaml"
PROXY_CONTAINER="vmdiag-proxy"
HEALTH_URL="http://localhost:8080/health.php"

if ! [[ "$COUNT" =~ ^[0-9]+$ ]] || [ "$COUNT" -lt 1 ]; then
  echo "[ERRORE] Uso: $0 <numero_istanze_web>  (minimo 1)"
  exit 1
fi

if [ ! -d "${PROJECT_DIR}" ]; then
  echo "[ERRORE] Directory progetto non trovata: ${PROJECT_DIR}"
  exit 1
fi

if [ ! -f "${COMPOSE_FILE}" ]; then
  echo "[ERRORE] File compose non trovato: ${COMPOSE_FILE}"
  exit 1
fi

cd "${PROJECT_DIR}"

echo "[1/7] Genero nginx.conf per ${COUNT} istanze..."
./scripts/render_nginx.sh "${COUNT}"

echo "[2/7] Scalo il servizio web..."
docker compose up -d --scale web="${COUNT}"

echo "[3/7] Verifico numero istanze attive..."
ACTUAL="$(docker ps --format '{{.Names}}' | grep -Ec '^vmdiag-app-web-[0-9]+$' || true)"
echo "Attese: ${COUNT} | Attive: ${ACTUAL}"
if [ "${ACTUAL}" -ne "${COUNT}" ]; then
  echo "[ERRORE] Numero istanze attive non coerente"
  exit 1
fi

echo "[4/7] Attendo che il proxy sia running..."
for i in $(seq 1 20); do
  STATUS="$(docker inspect -f '{{.State.Status}}' "${PROXY_CONTAINER}" 2>/dev/null || true)"
  if [ "${STATUS}" = "running" ]; then
    echo "Proxy running"
    break
  fi
  sleep 1
done

STATUS="$(docker inspect -f '{{.State.Status}}' "${PROXY_CONTAINER}" 2>/dev/null || true)"
if [ "${STATUS}" != "running" ]; then
  echo "[ERRORE] Proxy non in stato running"
  exit 1
fi

echo "[5/7] Verifico configurazione nginx..."
docker exec "${PROXY_CONTAINER}" nginx -t

echo "[6/7] Ricarico nginx..."
docker exec "${PROXY_CONTAINER}" nginx -s reload

echo "[7/7] Verifica finale..."
docker compose ps
echo "----"
curl -s "${HEALTH_URL}" || true
echo
echo "----"
for i in $(seq 1 6); do
  curl -s http://localhost:8080/info.php | grep -E '"app_instance"|"hostname"' || true
  echo "----"
done

echo "[OK] Scaling completato a ${COUNT} istanze web"
