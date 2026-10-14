#!/bin/zsh
# Start the cell twin web app (worker + server) and open it in the browser.
cd "$(dirname "$0")/.."
pkill -f "app/worker.py" 2>/dev/null; pkill -f "app/server.py" 2>/dev/null
mkdir -p out/live
nohup .venv/bin/python app/server.py > out/live/server.log 2>&1 &
sleep 2
open http://127.0.0.1:8771
echo "cell twin app: http://127.0.0.1:8771 (logs in out/live/)"
