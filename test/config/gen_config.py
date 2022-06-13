#!/usr/bin/python3
import os, sys
import json

LENGTH = 40000
QUEUE_LEN_MS = 100

TRACE_DIR = "/home/aposta/projects/alfalfa/test/fcc_for_emulation"
TRACE_ID = [1,2,3,4,5]

# name -> [CC, CODEC, FEC]
CONFIGS = {
    "oracle":   ["oracle", "basic", 1.5],
    "gcc":      ["gcc", "basic", 0],
    "gcc-fec":  ["gcc", "basic", 1.5],
    "gcc-svc":  ["gcc", "svc", 1.5],
    "bbr":      ["bbr", "basic", 0],
    "bbr-fec":  ["bbr", "basic", 1.5],
    "bbr-svc":  ["bbr", "svc", 1.5],
    "salsify":  ["salsify", "basic", 0],
    "salsify-fec": ["salsify", "basic", 1.5],
    "salsify-svc": ["salsify", "svc", 1.5]
}

def construct_json_object(config, trace, key, postfix):
    cc, codec, fec = config
    ret = {
            "trace": trace,
            "codec": codec,
            "cc": cc,
            "fec_rate": fec,
            "length_ms": LENGTH,

            "queue_length_ms": QUEUE_LEN_MS,
        
            "output_folder": f"/home/aposta/projects/alfalfa/results/{key}/",
            "encoder_output": f"encoder-{postfix}.csv",
            "decoder_output": f"decoder-{postfix}.csv",
            "encoder_stats": f"encstat-{postfix}.csv",
            "decoder_stats": f"decstat-{postfix}.csv",
            "real_stats": f"real-{postfix}.csv",
            "pred_stats": f"pred-{postfix}.csv"
        }
    return ret

for key in CONFIGS.keys():
    for trace_id in TRACE_ID:
        for run_id in [1,2,3]:
            config = CONFIGS[key]
            trace_name = os.path.join(TRACE_DIR, str(trace_id)+".csv")
            assert os.path.isfile(trace_name), trace_name + " not exists!"
            postfix = str(trace_id) + "-" + str(run_id)
            json_obj = construct_json_object(config, trace_name, key, postfix)

            outfile_name = f"{key}-{trace_id}-{run_id}.json"
            with open(outfile_name, "w") as fout:
                json.dump(json_obj, fout)
