if [ -f local.env ]; then
    set -a
    source local.env
    set +a
fi

# Default port
PORT="${GDB_PORT:-3333}"

# Ensure GDB server is running on $PORT
if ! ss -tunlp | grep -q ":$PORT"; then
    echo "Starting OpenOCD GDB server on port $PORT..."
    python3 -c "import sys; sys.path.append('scripts'); from debug_tools import start_gdb_server; start_gdb_server($PORT)"
    # Give it a few seconds to start
    sleep 3
else
    echo "GDB server already running on port $PORT."
fi

# Run the requested agent command
# Usage: ./debug_me.sh <command> [args]
# Commands: reset, backtrace, read <var>
python3 scripts/gdb_agent.py "$@"
