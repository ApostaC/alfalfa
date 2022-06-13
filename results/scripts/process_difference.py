import numpy as np
import os, sys
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def read_df(filename):
    """
    format: <timestamp> <pacing rate> <target rate> <loss rate>
    """
    df = pd.read_csv(filename, header=None, names=["timestamp", "pacing_rate", "target_rate", "loss_rate"]);
    df["pacing_rate"] = df["pacing_rate"] / 125 # byte per sec --> kbps
    df["target_rate"] = df["target_rate"] / 125 # byte per sec --> kbps
    return df

def read_dec_df(filename):
    """
    format: <frame_id> <loss> <fec rate> 
    """
    df = pd.read_csv(filename, header=None, names=["frame_id", "fec_rate", "loss_rate"]);
    return df


def plot_field_one(field, real_df, legend=None, ylabel=None):
    fig = plt.figure()
    plt.plot(real_df["timestamp"], real_df[field], label=legend);
    plt.xlabel("timestamp (ms)")
    if ylabel:
        plt.ylabel(ylabel)
    plt.title(field)
    plt.legend()
    plt.grid()
    return fig

def plot_field(field, real_df, pred_df, ylabel=None):
    fig = plt.figure()
    plt.plot(real_df["timestamp"], real_df[field], label="real");
    plt.plot(pred_df["timestamp"], pred_df[field], label="pred");
    plt.xlabel("timestamp (ms)")
    if ylabel:
        plt.ylabel(ylabel)
    plt.legend()
    plt.title(field)
    plt.grid()
    return fig

def plot_cdf(df, field, xlabel = None):
    fig = plt.figure()
    plt.plot(df[field], df.index/len(df))
    plt.ylabel("CDF")
    if not (xlabel is None):
        plt.xlabel(xlabel)
    plt.grid()
    return fig

def plot_scatter(x, y, xlabel, ylabel):
    fig = plt.figure()
    plt.scatter(x, y)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid()
    return fig


def get_value(df, field, time):
    tmp = df.where(df["timestamp"] >= time).dropna().sub(time)
    idx = df.idxmax()
    if len(tmp) > 0:
        idx = tmp.idxmin()
    return df.loc[idx, field][:1].item()

if len(sys.argv) != 5:
    print("Usage: {} <real_csv> <pred_csv> <encoder_stats> <decoder_stats>".format(sys.argv[0]))
    exit(0)

pdf_pages = PdfPages('stats.pdf')
real_df = read_df(sys.argv[1])
pred_df = read_df(sys.argv[2])
enc_df = read_df(sys.argv[3])
dec_df = read_dec_df(sys.argv[4])
print(dec_df)
enc_df["timestamp"] = enc_df["timestamp"] + 50 # fix the timestamp shift (before frame --> after frame)


fig1 = plot_field("pacing_rate", real_df, pred_df, "kbps")
#fig2 = plot_field("target_rate", real_df, pred_df, "kbps")
fig2 = plot_field_one("target_rate", real_df, "bandwidth", "kbps")

fig3 = plot_field("loss_rate", real_df, pred_df, "loss ratio")
fig4 = plot_scatter(dec_df["loss_rate"], dec_df["fec_rate"], "loss rate", "fec rate")

#last_ts = 0
#real_losses = []
#fec_rates = []
#for row in enc_df.iterrows():
#    ind, series = row
#    frame_fec = series["loss_rate"]
#    ts = series["timestamp"]
#    if last_ts != ts: 
#        last_ts = ts
#        continue
#    last_ts = ts
#    if frame_fec == 0:
#        continue
#    real_loss = get_value(pred_df, "loss_rate", ts + 500)
#    real_losses += [real_loss]
#    fec_rates += [frame_fec]
#    print(ts, frame_fec, real_loss)
#fig4 = plot_scatter(real_losses, fec_rates, "real loss", "fec rate")

pdf_pages.savefig(fig1)
pdf_pages.savefig(fig2)
pdf_pages.savefig(fig3)
pdf_pages.savefig(fig4)

pdf_pages.close()
