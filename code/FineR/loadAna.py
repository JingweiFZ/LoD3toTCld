import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import utils


# Load the data
def load_data(filename):
    dfa = pd.read_csv(filename, delimiter="\t", header=None)
    return dfa


def hitogram(data, a):
    # Plot the histogram
    # 计算最大值和最小值，以及确定直方图的边界
    min_value = int(min(data))
    max_value = int(max(data))
    interval = a  # 替换为你希望的间隔值

    # 创建边界
    bin_edges = np.arange(min_value, max_value + interval, interval)

    # 生成直方图
    hist, _ = np.histogram(data, bins=bin_edges)

    # 添加标题和标签
    plt.title("数据分布直方图")
    plt.xlabel("值")
    plt.ylabel("频数")

    # 显示直方图
    plt.hist(data, bins=bin_edges, edgecolor='black')  # 用于可视化直方图

    # 保存统计结果到文件
    with open("histogram_stats.txt", "w") as f:
        f.write("Bin, Frequency\n")
        for i, freq in enumerate(hist):
            f.write(f"{bin_edges[i]} - {bin_edges[i + 1]}, {freq}\n")

    # 显示直方图
    plt.hist(data, bins=bin_edges, edgecolor='black')  # 用于可视化直方图

    # 保存直方图图像到文件
    plt.savefig("histogram_plot.png")

    # 显示直方图
    plt.show()

    return hist


if __name__ == '__main__':
    file_dir = ''
    file_name = ''
    save_dir = ''

    print('Histogram Analysis')

    print('***** load the data from file *****')

    load_all = True
    load_single = False

    data = pd.DataFrame()

    if load_all:
        fileset = utils.getfilelist(file_dir)
        for f in fileset:
            df = load_data(file_dir + f)
            data = pd.concat([data, df], axis=0, ignore_index=None)
    elif load_single:
        data = load_data(file_dir + file_name)

    print('Print hitrogram')
    interval = 1
    hitogram(data, interval)

    print('Finish!')
