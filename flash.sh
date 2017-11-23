#!/bin/bash

nrfjprog --family NRF52 --program zephyr/zephyr.hex --chiperase --verify --log; nrfjprog --family NRF52 --reset
