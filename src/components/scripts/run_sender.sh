#!/bin/bash

set -e 
#if [[ $# -ne 2 ]]; then
#    echo "Usage: $0 <fec protection overhead> <output prefix>"
#    echo "  <fec protection overhead> is representing: fec_protection_rate / loss_rate"
#    exit 1
#fi

if [[ -z $MAHIMAHI_BASE ]]; then
    MAHIMAHI_BASE=127.0.0.1
fi

PORT=54123
echo "Target Address = $MAHIMAHI_BASE:$PORT"
#echo "Dumping to ${2}sender-*.csv"

BIN=./test-sender
CFG_DIR=/home/aposta/projects/alfalfa/test/config/

for file in $CFG_DIR/*-3-1.json; do
    echo "Starting sender with the configure file $file"
    $BIN $MAHIMAHI_BASE $PORT $file 
    sleep 5s
done
#echo "Starting sender: 0"
#$BIN $MAHIMAHI_BASE $PORT $1 ${2}sender-0.csv 2>&1 | tee out.txt
#sleep 5s
#
#echo "Starting sender: 1"
#$BIN $MAHIMAHI_BASE $PORT $1 ${2}sender-1.csv 2>&1 | tee out.txt
#sleep 5s
#
#echo "Starting sender: 2"
#$BIN $MAHIMAHI_BASE $PORT $1 ${2}sender-2.csv 2>&1 | tee out.txt
