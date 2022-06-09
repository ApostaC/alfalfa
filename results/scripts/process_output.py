import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
import numpy as np

df = pd.read_csv("output.csv", header=None, names=["type", "trace", "id", "lat", "PSNR"])

def plot_one(df):
    marker_color = {
        "oracle" : ('.', "r"),
        "bbr" : (".", 'g'),
        "bbr-fec" : ("+", 'g'),
        "bbr-svc" : ("x", 'g'),
        "gcc" : (".", 'b'),
        "gcc-fec" : ("+", 'b'),
        "gcc-svc" : ("x", 'b'),
        "salsify" : (".", 'c'),
        "salsify-fec" : ("+", 'c'),
        "salsify-svc" : ("x", 'c'),
    }

    fig = plt.figure()
    for key in marker_color.keys():
        marker, color = marker_color[key]
        plt.scatter(df.query("type == @key")["lat"], df.query("type == @key")["PSNR"], marker=marker, c=color, label=key)

    plt.grid()
    plt.xlabel("tail latency (95%)")
    plt.ylabel("Avg PSNR")
    plt.legend()
    return fig

pdf_pages = PdfPages("result_scatter.pdf")

for tid in [3]:
    print("processing trace: ", tid)
    fig = plot_one(df.query("trace == @tid"))
    pdf_pages.savefig(fig)

pdf_pages.close()
