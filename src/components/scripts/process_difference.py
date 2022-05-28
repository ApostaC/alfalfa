import numpy as np
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


pdf_pages = PdfPages('stats.pdf')
real_df = read_df("real.csv")
pred_df = read_df("pred.csv")
pred_df = read_df("/tmp/encoder.csv")

print(real_df)

# find the error of loss
pred_ts = pred_df["timestamp"]
real_ts = real_df["timestamp"]
def find_good(x, real_ts):
    tmp = real_ts.where(real_ts >= x).dropna().sub(x)
    if len(tmp) > 0:
        return tmp.idxmin()
    return real_ts.idxmax()

idx = np.fromiter(map(lambda x: find_good(x, real_ts), pred_ts), dtype=int)
val = np.fromiter(map(lambda i: real_df.loc[i, "loss_rate"], idx), dtype=float)
error_df = pred_df[["loss_rate"]]
error_df["real_loss"] = val
error_df["error"] = error_df["loss_rate"] - error_df["real_loss"]
error_df = error_df.sort_values("error").reset_index(drop=True)

print(error_df)


fig1 = plot_field("pacing_rate", real_df, pred_df, "kbps")
fig2 = plot_field("target_rate", real_df, pred_df, "kbps")
fig3 = plot_field("loss_rate", real_df, pred_df, "loss ratio")
fig4 = plot_cdf(error_df, "error", "loss pred error")

pdf_pages.savefig(fig1)
pdf_pages.savefig(fig2)
pdf_pages.savefig(fig3)
pdf_pages.savefig(fig4)

pdf_pages.close()
