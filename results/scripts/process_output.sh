#!/bin/bash
# process the output of "process_all.sh"
# format: ../<setup> <trace id>-<expr id> Tail latency: <tail lat>, AVG PSNR = <avg PSNR>
cat output.txt | cut -c 4- | sed 's/Tail latency: //g' | sed 's/, AVG PSNR =//g' | sed -E 's/([[:digit:]])-([[:digit:]])/\1 \2/g' | sed 's/\t//g' | sed 's/ /,/g' > output.csv
python3 process_output.py
