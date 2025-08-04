for port in {19850..19852}; do
	echo "Checking port $port..."
	pids=$(lsof -t -iTCP:$port -iUDP:$port)
	if [ -n "$pids" ]; then
		echo "Killing processes on port $port: $pids"
		for pid in $pids; do
			kill -9 "$pid" && echo "Killed PID $pid"
		done
	else
		echo "No process found on port $port."
	fi
done