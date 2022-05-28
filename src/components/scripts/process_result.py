import cv2
import sys
from PIL import Image
import numpy as np
import pandas as pd
from tqdm import tqdm

MPEG_MIN_QP = 10

def PSNR(Y1_raw, Y1_com):
    Y1_com = Y1_com.to(Y1_raw.device)
    log10 = torch.log(torch.FloatTensor([10])).squeeze(0).to(Y1_raw.device)
    train_mse = torch.mean(torch.pow(Y1_raw - Y1_com, 2))
    quality = 10.0*torch.log(1/train_mse)/log10
    return quality

class BaseMPEGModel:
    """
    provide interface for loading csvs
    """
    def __init__(self):
        self.profile = pd.DataFrame()
        self.freeze_psnr = 0
        self.nframes = 0

    def estimate_latency(self, size, loss, base_latency):
        """
        Input:
        t3["latency"] = t3["dec_ts"] - t3["enc_ts"]
            size: the encoded size of that frame (no fec)
            loss: the loss rate
        Output:
            latency: the result latency
        """
        n_pkts = np.ceil(size / PKT_SZ)
        ret_times = estimate_retrans_overhead(loss, n_pkts)
        est_lat = (ret_times - 1) * base_latency * 2 + base_latency
        return est_lat

    def load_video_profile(self, video_profile):
        """
        Input:
            video_profile: csv, format is <frame_id> <size> <psnr> <qp>
        """
        global MPEG_MIN_QP
        self.profile = pd.read_csv(video_profile)
        freeze_psnr_row = self.profile.query("frame_id == -1")
        self.freeze_psnr = float(freeze_psnr_row["psnr"])
        self.profile = self.profile.query("qp > @MPEG_MIN_QP")
        self.nframes = max(self.profile["frame_id"])
        print("Load the information for {} frames".format(self.nframes))
        return self

    def fit_size_for_frame(self, frame_id, size):
        """
        Input:
            frame_id: id of the frame
            size: the total size for a frame (NO FEC, NO SVC)
        Output:
            size: the real frame size
            psnr: the psnr of the frame
        """
        temp = self.profile.query("frame_id == @frame_id")
        tgt_size = size 

        min_possible_size = min(temp["size"])
        worst_psnr = min(temp["psnr"])

        result_index = temp['size'].sub(tgt_size).abs().idxmin()
        temp = temp.query("index == @result_index")

        #temp = temp.query("size < @tgt_size")
        #if len(temp) == 0:
        #    return min_possible_size, worst_psnr

        #temp = temp.sort_values("size", ascending=False).head(1)
        return float(temp["size"]), float(temp["psnr"])
    
    def _compute_lost_packets(self, n_pkts, loss):
        """
        n_pkts: number of pkts
        loss: loss ratio
        returns: the number of lost packets
        """
        e = n_pkts * loss
        lb = np.floor(e)
        lprob = e - lb
        #print(n_pkts, loss, lprob, lb)
        if np.random.uniform() > lprob:
            return lb
        else:
            return lb + 1

    def cal_latency(self, sizes, losses, base_latencys):
        """
        Input:
            sizes: list of frame size in bytes
            losses: pkt loss for each frame

        Output:
            latencys: the estimated latency for each frame
        """
        return base_latencys

    def fit_trace(self, sizes):
        """
        Input:
            sizes: list of frame size in bytes
            losses: pkt loss for each frame

        Output:
            psnrs: the final psnr for each frame
        """
        sizes = sizes[:self.nframes]
        psnrs = []

        for frame_id in range(len(sizes)):
            size = sizes[frame_id]

            ''' find the suitable frame size and psnr '''
            code_size, psnr = self.fit_size_for_frame(frame_id, size)
            psnrs.append(psnr)
        return psnrs

model = BaseMPEGModel()
model.load_video_profile("../../../test/tom-norman.csv")

postfix = 0
if sys.argv[1:]:
    postfix = int(sys.argv[1])
print("Using postfix=", postfix)

enc_df = pd.read_csv(f"./sender-{postfix}.csv", header=None, names=["frame_id", "enc_ts", "size"])
dec_df = pd.read_csv(f"./receiver-{postfix}.csv", header=None, names=["frame_id", "dec_ts", "size"])
t1 = enc_df[["frame_id", "enc_ts"]]
t2 = dec_df[["frame_id", "dec_ts"]]
t3 = pd.merge(t1, t2, on="frame_id")
t3["latency"] = t3["dec_ts"] - t3["enc_ts"]
t3 = t3[:model.nframes]
print(t3)

psnrs = model.fit_trace(enc_df["size"])
t3["psnr"] = psnrs
t3["util"] = t3["psnr"] - t3["latency"] / 200
print("AVG PSNR = ", np.mean(psnrs))
print("95% Latency = ", t3["latency"].quantile(0.95))
print("AVG SIZE = ", np.mean(enc_df["size"]))

print("MED U = ", t3["util"].quantile(0.5))
print("Tail U = ", t3["util"].quantile(0.05))
