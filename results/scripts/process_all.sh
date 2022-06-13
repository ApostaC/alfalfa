#!/bin/bash
echo -n "" > util.txt
echo -n "" > psnr.txt

process_util() {
    outfile=util.txt
    folder=$1
    postfix=$2
    if [[ -f $folder/encoder-$postfix.csv ]]; then
        echo "processing util for $folder $postfix"
        echo -n "$folder $postfix " >> $outfile
        python3 ./process_result.py $folder/encoder-$postfix.csv $folder/decoder-$postfix.csv util >> $outfile
    fi
}

process_psnr_lat() {
    outfile=psnr.txt
    folder=$1
    postfix=$2
    if [[ -f $folder/encoder-$postfix.csv ]]; then
        echo "processing psnr and lat for $folder $postfix"
        echo -n "$folder $postfix " >> $outfile
        python3 ./process_result.py $folder/encoder-$postfix.csv $folder/decoder-$postfix.csv psnr >> $outfile
    fi
}

for folder in ../*; do
    #process_one $folder 1-1
    #process_one $folder 2-1
    process_util $folder 3-1
    process_psnr_lat $folder 3-1
    #process_one $folder 4-1
    #process_one $folder 5-1
done
