
process_one() {
    folder=$1
    postfix=$2
    if [[ -f $folder/encoder-$postfix.csv ]]; then
        echo -n "$folder $postfix "
        python3 ./process_result.py $folder/encoder-$postfix.csv $folder/decoder-$postfix.csv
    fi
}

for folder in ../*; do
    #process_one $folder 1-1
    #process_one $folder 2-1
    process_one $folder 3-1
    #process_one $folder 4-1
    #process_one $folder 5-1
done
