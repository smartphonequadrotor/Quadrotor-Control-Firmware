target remote localhost:2331
monitor flash device = AT91SAM7S161
monitor endian little
monitor speed 30
monitor reset 8
monitor writeu32 0xFFFFFD44 = 0x00008000
monitor sleep 100
monitor speed 12000