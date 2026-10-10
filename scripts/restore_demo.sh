#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "[1/3] Ripristino servizi Docker..."
docker compose \
  -f compose.yaml \
  -f compose.api.yaml \
  up -d

echo "[2/3] Verifica robot..."
if tmux has-session -t '=pulse1' 2>/dev/null; then
    echo "Sessione robot gia' presente."
else
    ./scripts/start_demo.sh
fi

echo "[3/3] Verifica collector..."
./scripts/start_collector.sh

echo "Procedura di ripristino completata."
