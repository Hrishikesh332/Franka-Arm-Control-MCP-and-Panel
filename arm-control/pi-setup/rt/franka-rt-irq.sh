#!/bin/sh
# Ethernet interrupts for the robot link: on the isolated core 3 (isolcpus=3 in cmdline.txt,
# shared with the ros2_control loop) and above the loop's priority (FIFO 97), so the robot's
# UDP packets are handled as soon as they arrive.
for irq in $(awk -F: '/eth0/ {gsub(/ /, "", $1); print $1}' /proc/interrupts); do
  echo 3 > "/proc/irq/$irq/smp_affinity_list"
done
for pid in $(ps -eo pid,comm | awk '$2 ~ /^irq\/[0-9]+-eth0$/ {print $1}'); do
  taskset -p -c 3 "$pid" > /dev/null
  chrt -f -p 98 "$pid"
done
