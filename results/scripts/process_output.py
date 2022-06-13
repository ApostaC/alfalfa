import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
import numpy as np

util_df = pd.read_csv("util.csv", header=None, names=["type", "trace", "id", "X", "Y"])
psnr_df = pd.read_csv("psnr.csv", header=None, names=["type", "trace", "id", "X", "Y"])

def plot_one(df, xlabel, ylabel):
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
        plt.scatter(df.query("type == @key")["X"], df.query("type == @key")["Y"], marker=marker, c=color, label=key)

    plt.grid()
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()
    return fig

pdf_pages = PdfPages("result_scatter.pdf")

print(psnr_df)
print(util_df)
for tid in [3]:
    print("processing trace: ", tid)
    fig = plot_one(psnr_df.query("trace == @tid"), "tail latency (95%)", "Avg PSNR")
    pdf_pages.savefig(fig)
    fig = plot_one(util_df.query("trace == @tid"), "tail util (95%)", "median util")
    pdf_pages.savefig(fig)

fig = plt.figure()
pdf_pages.savefig(fig)
pdf_pages.close()
