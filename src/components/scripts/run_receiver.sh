#!/bin/bash

set -e 
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <output prefix>"
    exit 1
fi

if [[ -z $MAHIMAHI_BASE ]]; then
    MAHIMAHI_BASE=127.0.0.1
fi

PORT=54123
echo "Target PORT = $PORT"
echo "Dumping to ${1}receiver-*.csv"

BIN=./test-receiver

echo "Starting receiver: 0"
$BIN $PORT ${1}receiver-0.csv 
sleep 1

echo "Starting receiver: 1"
$BIN $PORT ${1}receiver-1.csv 
sleep 1

echo "Starting receiver: 2"
$BIN $PORT ${1}receiver-2.csv 
sleep 1



