nRF52832 and CAP1106 test
=========================

## Introduction
Testing nRF52 + CAP1106 using Zephyr

## Build
Setup zephyr
Setup nrfjprog

```bash
mkdir build
cd build
cmake -DBOARD=nrf52_vbluno52 ..
make -j4
```

## Flash
```bash
cd build
../flash.sh
```
