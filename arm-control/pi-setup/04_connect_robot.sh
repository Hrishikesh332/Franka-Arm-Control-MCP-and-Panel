#!/bin/bash
# Phase 3: configure eth0 for the robot network and check the FCI connection.
# Usage: sudo bash 04_connect_robot.sh <robot_ip> [pi_ip/prefix]
#   e.g. sudo bash 04_connect_robot.sh 172.16.0.2            -> Pi gets 172.16.0.1/24
#        sudo bash 04_connect_robot.sh 192.168.1.20 192.168.1.50/24
set -eo pipefail
ROBOT_IP=${1:?robot ip required}
PI_ADDR=${2:-$(echo "$ROBOT_IP" | awk -F. '{print $1"."$2"."$3"."($4==1?2:1)"/24"}')}

python3 - "$PI_ADDR" <<'PY'
import sys, yaml
p = '/etc/netplan/50-cloud-init.yaml'
c = yaml.safe_load(open(p))
eth = c['network']['ethernets']['eth0']
addrs = [a for a in eth.get('addresses', []) if a.startswith('192.168.5.')]  # keep laptop link
eth['addresses'] = addrs + [sys.argv[1]]
eth['dhcp4'] = False
yaml.safe_dump(c, open(p, 'w'), default_flow_style=False)
PY
chmod 600 /etc/netplan/50-cloud-init.yaml
netplan apply
sleep 3
ip -br -4 a show eth0
ethtool eth0 2>/dev/null | grep -E 'Speed|Duplex|Link detected' || true

echo "--- ping robot $ROBOT_IP"
ping -c 5 -i 0.2 "$ROBOT_IP" | tail -2
echo "--- Desk (https) reachable?"
curl -sk -m 5 -o /dev/null -w '%{http_code}\n' "https://$ROBOT_IP/" || true
