#!/bin/bash
echo "compiling BD_sensor.c into the klipper firmware"
sed -i '/BD_sensor/d' src/Makefile;echo "src-y += BD_sensor.c" >> src/Makefile
make
sed -i '/BD_sensor/d' src/Makefile
git checkout src/Makefile
