import numpy as np
import matplotlib.pyplot as plt

TIMESTAMP_COLUMN = 45

LEFT_FINGER_COLUMN_IDX = 46
RIGHT_FINGER_COLUMN_IDX = 55
PRISMATIC_COLUMN_IDX = 58
REVOLUTE_1_COLUMN_IDX = 61
REVOLUTE_2_COLUMN_IDX = 64
WRIST_COLUMN_IDX = 67

LINE_WIDTH = 0.7

def get_xy_axis(data, step):
    sim_timestamp_raw = data[:, TIMESTAMP_COLUMN]

    mask = sim_timestamp_raw != ''
    sim_timestamp_axis = sim_timestamp_raw[mask].astype(np.float64)

    #sim_timestamp_axis = timestamp_axis - np.min(timestamp_axis)

    efforts_raw = data[:, [LEFT_FINGER_COLUMN_IDX,
                       RIGHT_FINGER_COLUMN_IDX,
                       PRISMATIC_COLUMN_IDX,
                       REVOLUTE_1_COLUMN_IDX,
                       REVOLUTE_2_COLUMN_IDX,
                       WRIST_COLUMN_IDX]]

    mask = np.all(efforts_raw != '', axis=1)
    efforts = efforts_raw[mask].astype(np.float64)

    g_partial_axis = np.sum(np.abs(efforts), axis=1)

    return sim_timestamp_axis[::step], g_partial_axis[::step]

def main():    
    raw_data = np.loadtxt('performance_data.csv', dtype=str,
                            delimiter=',', skiprows=1)

    time, g_partial = get_xy_axis(raw_data,1)

    plt.plot(time, g_partial, color='blue',
                linewidth=LINE_WIDTH)

    plt.title('TIEMPO VS G-PARCIAL')
    plt.xlabel('TIME')
    plt.ylabel('G-PARTIAL')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
  main()