sudo tee /usr/local/bin/run_bin2json.sh > /dev/null << 'EOF'
#!/usr/bin/env bash

sleep 3

exec /usr/local/bin/bin2json_mqtt
EOF

sudo chmod +x /usr/local/bin/run_bin2json.sh
