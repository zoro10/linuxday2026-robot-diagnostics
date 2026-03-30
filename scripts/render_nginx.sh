#!/usr/bin/env bash
set -euo pipefail

COUNT="${1:-1}"
PROJECT_DIR="${HOME}/vmdiag-app"
PROXY_DIR="${PROJECT_DIR}/proxy"
NGINX_CONF="${PROXY_DIR}/nginx.conf"

if ! [[ "$COUNT" =~ ^[0-9]+$ ]] || [ "$COUNT" -lt 1 ]; then
  echo "[ERRORE] Uso: $0 <numero_istanze_web>  (minimo 1)"
  exit 1
fi

mkdir -p "${PROXY_DIR}"

{
  echo "events {}"
  echo
  echo "http {"
  echo "  upstream app_backend {"
  for i in $(seq 1 "$COUNT"); do
    echo "    server vmdiag-app-web-${i}:80;"
  done
  echo "  }"
  echo
  echo "  server {"
  echo "    listen 80;"
  echo "    server_name _;"
  echo
  echo "    location / {"
  echo "      proxy_pass http://app_backend;"
  echo "      proxy_set_header Host \$host;"
  echo "      proxy_set_header X-Real-IP \$remote_addr;"
  echo "      proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;"
  echo "      proxy_set_header X-Forwarded-Proto \$scheme;"
  echo "    }"
  echo "  }"
  echo "}"
} > "${NGINX_CONF}"

echo "[OK] Creato ${NGINX_CONF} con ${COUNT} backend web"
