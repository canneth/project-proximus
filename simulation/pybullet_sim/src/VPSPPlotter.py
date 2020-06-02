
import csv
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
from matplotlib.animation import FuncAnimation
import numpy as np

from PyplotPlayer import PyplotPlayer

if __name__ == "__main__":
    # Load data from csv
    csv_file_path = Path(".") / "data" / "vpsp_data.csv"
    csv_data = []
    csv_headers = []
    with open(csv_file_path, 'r') as f:
        reader = csv.reader(f) # change contents to floats
        csv_headers = next(reader)
        for row in reader: # each row is a list
            csv_data.append(row)
    csv_headers = np.array(csv_headers)
    csv_data = np.array(csv_data).astype(np.float)

    # Prepare data
    data = csv_data
    # data = np.delete(csv_data, slice(0, 200), axis = 0)
    t = data[:, 0].reshape(-1, 1).reshape(-1, 1)
    p_b_vpsp_x = data[:, 1].reshape(-1, 1)
    p_b_vpsp_y = data[:, 2].reshape(-1, 1)
    FL_cw_vp_x = data[:, 3].reshape(-1, 1)
    FL_cw_vp_y = data[:, 4].reshape(-1, 1)
    FR_cw_vp_x = data[:, 5].reshape(-1, 1)
    FR_cw_vp_y = data[:, 6].reshape(-1, 1)
    BL_cw_vp_x = data[:, 7].reshape(-1, 1)
    BL_cw_vp_y = data[:, 8].reshape(-1, 1)
    BR_cw_vp_x = data[:, 9].reshape(-1, 1)
    BR_cw_vp_y = data[:, 10].reshape(-1, 1)
    FL_ccw_vp_x = data[:, 11].reshape(-1, 1)
    FL_ccw_vp_y = data[:, 12].reshape(-1, 1)
    FR_ccw_vp_x = data[:, 13].reshape(-1, 1)
    FR_ccw_vp_y = data[:, 14].reshape(-1, 1)
    BL_ccw_vp_x = data[:, 15].reshape(-1, 1)
    BL_ccw_vp_y = data[:, 16].reshape(-1, 1)
    BR_ccw_vp_x = data[:, 17].reshape(-1, 1)
    BR_ccw_vp_y = data[:, 18].reshape(-1, 1)
    FL_vt_x = data[:, 19].reshape(-1, 1)
    FL_vt_y = data[:, 20].reshape(-1, 1)
    FR_vt_x = data[:, 21].reshape(-1, 1)
    FR_vt_y = data[:, 22].reshape(-1, 1)
    BL_vt_x = data[:, 23].reshape(-1, 1)
    BL_vt_y = data[:, 24].reshape(-1, 1)
    BR_vt_x = data[:, 25].reshape(-1, 1)
    BR_vt_y = data[:, 26].reshape(-1, 1)
    FL_phase = data[:, 27].reshape(-1, 1)
    FR_phase = data[:, 28].reshape(-1, 1)
    BL_phase = data[:, 29].reshape(-1, 1)
    BR_phase = data[:, 30].reshape(-1, 1)
    data_labels = [
        "p_b_vpsp",
        "FL_cw_vp",
        "FR_cw_vp",
        "BL_cw_vp",
        "BR_cw_vp",
        "FL_ccw_vp",
        "FR_ccw_vp",
        "BL_ccw_vp",
        "BR_ccw_vp",
        "FL_vt",
        "FR_vt",
        "BL_vt",
        "BR_vt"
    ]
    data_dict = {key: None for key in data_labels}
    data_dict["p_b_vpsp"] = np.block([p_b_vpsp_x, p_b_vpsp_y])
    data_dict["FL_cw_vp"] = np.block([FL_cw_vp_x, FL_cw_vp_y])
    data_dict["FL_ccw_vp"] = np.block([FL_ccw_vp_x, FL_ccw_vp_y])
    data_dict["FR_cw_vp"] = np.block([FR_cw_vp_x, FR_cw_vp_y])
    data_dict["FR_ccw_vp"] = np.block([FR_ccw_vp_x, FR_ccw_vp_y])
    data_dict["BL_cw_vp"] = np.block([BL_cw_vp_x, BL_cw_vp_y])
    data_dict["BL_ccw_vp"] = np.block([BL_ccw_vp_x, BL_ccw_vp_y])
    data_dict["BR_cw_vp"] = np.block([BR_cw_vp_x, BR_cw_vp_y])
    data_dict["BR_ccw_vp"] = np.block([BR_ccw_vp_x, BR_ccw_vp_y])
    data_dict["FL_vt"] = np.block([FL_vt_x, FL_vt_y])
    data_dict["FR_vt"] = np.block([FR_vt_x, FR_vt_y])
    data_dict["BL_vt"] = np.block([BL_vt_x, BL_vt_y])
    data_dict["BR_vt"] = np.block([BR_vt_x, BR_vt_y])

    
    fig = plt.figure()
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    def draw_frame(i):

        ax.clear()
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.5, 0.5)
        for data_label in list(data_dict.keys()):
            x = data_dict[data_label][i, 0]
            y = data_dict[data_label][i, 1]
            # If associated leg is in swing, plot VPSP vertex as empty circle, else solid circle
            if (data_label == "FL_vt" and FL_phase[i] == 0):
                ax.scatter(x, y, facecolors = "none", edgecolors = "r", s = 100)
            elif (data_label == "FR_vt" and FR_phase[i] == 0):
                ax.scatter(x, y, facecolors = "none", edgecolors = "r", s = 100)
            elif (data_label == "BL_vt" and BL_phase[i] == 0):
                ax.scatter(x, y, facecolors = "none", edgecolors = "r", s = 100)
            elif (data_label == "BR_vt" and BR_phase[i] == 0):
                ax.scatter(x, y, facecolors = "none", edgecolors = "r", s = 100)
            else:
                ax.scatter(x, y)
                pass
            ax.annotate(
                data_label, # this is the text
                (x, y), # this is the point to label
                textcoords = "offset points", # how to position the text
                xytext = (0, 1), # distance from text to points (x,y)
                ha = 'center' # horizontal alignment can be left, right or center
            )
            plt.title("t: {}".format(t.reshape(-1)[i]), loc = "left")

    ani = PyplotPlayer(fig, draw_frame, maxi = len(data[:, 0]) - 1, interval = 100)
    plt.show()