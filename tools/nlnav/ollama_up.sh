#!/usr/bin/env bash
# Start (or reuse) the local LLM for natural-language navigation: Ollama in the m20_ollama container on the
# laptop's GPU, model NLNAV_LLM_MODEL (default qwen2.5:7b-instruct), weights kept in ~/m20_sim/ollama so a pull
# happens once. The container stays up between launches so the model stays loaded.
#   bash tools/nlnav/ollama_up.sh
set -euo pipefail
MODEL="${NLNAV_LLM_MODEL:-qwen2.5:7b-instruct}"
if ! docker ps --format '{{.Names}}' | grep -q '^m20_ollama$'; then
  docker rm -f m20_ollama >/dev/null 2>&1 || true
  mkdir -p "$HOME/m20_sim/ollama"
  docker run -d --name m20_ollama --gpus all --network host -e OLLAMA_HOST=127.0.0.1:11434 \
    -e OLLAMA_KEEP_ALIVE=30m -v "$HOME/m20_sim/ollama":/root/.ollama ollama/ollama:latest >/dev/null
  for _ in $(seq 1 30); do docker exec m20_ollama ollama list >/dev/null 2>&1 && break; sleep 1; done
  docker exec m20_ollama ollama list >/dev/null 2>&1 || { echo "ERROR: Ollama did not come up: docker logs m20_ollama"; exit 1; }
fi
docker exec m20_ollama ollama list | grep -q "^${MODEL}" || {
  echo "[ollama_up] pulling $MODEL (once, a few GB)..."; docker exec m20_ollama ollama pull "$MODEL"; }
echo "[ollama_up] m20_ollama up with $MODEL on 127.0.0.1:11434"
