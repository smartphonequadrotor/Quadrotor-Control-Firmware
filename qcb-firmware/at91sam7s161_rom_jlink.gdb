target remote localhost:2331
monitor flash device = AT91SAM7S161
monitor speed adaptive
monitor flash download 1
monitor halt
load qcb-firmware.elf
monitor reset
monitor go
