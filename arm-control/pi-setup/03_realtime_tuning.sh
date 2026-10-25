#!/bin/bash
# Realtime tuning for the 1 kHz robot link (run with sudo, then reboot):
#  - core 3 reserved for the control loop (isolcpus etc. on the kernel command line)
#  - eth0 interrupts on core 3 at SCHED_FIFO 98 (franka-rt-irq.service)
# Prerequisite: the Ubuntu Pro realtime kernel:
#   sudo pro attach <token> && sudo pro enable realtime-kernel --variant=raspi
set -exo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
CMDLINE=/boot/firmware/cmdline.txt
grep -q isolcpus $CMDLINE || {
  cp $CMDLINE $CMDLINE.before-isolcpus
  sed -i '1 s/$/ isolcpus=3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2/' $CMDLINE
}
install -m 755 $HERE/rt/franka-rt-irq.sh /usr/local/sbin/
install -m 644 $HERE/rt/franka-rt-irq.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable franka-rt-irq.service
echo "reboot to activate the isolated core"
