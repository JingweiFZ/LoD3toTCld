import os.path

import pandas as pd
import numpy as np
import h5py
import utils

def convert_h5py(filename, savefile):
    # 读取文本数据到Pandas DataFrame（替换为你的数据文件和分隔符）
    df = pd.read_csv(filename, delimiter='\t')

    # 提取x、y、z、r、g、b和label数据
    xyz = df[:, 0:3]  # x, y, z坐标
    rgb_data = np.stack(df[:, 3] * 3, axis=-1)
    rgb = rgb_data  # r, g, b颜色
    label = df[:, 6]  # label

    # 创建HDF5文件
    with h5py.File(savefile, 'w') as hf:
        # 将Pandas DataFrame写入HDF5文件中
        hf.create_dataset('xyz', data=xyz)

        # 存储点云颜色
        hf.create_dataset('rgb', data=rgb)

        # 存储点云label
        hf.create_dataset('label', data=label)

    print("HDF5文件已创建并数据已写入。")

if __name__ == '__main__':
    file_dir='G:/Application/TBBR/train/'
    filename=''
    save_dir=''

    load_all=True
    load_single=False

    if load_all:
        print('load all the files from '+file_dir)
        fileset = utils.getfilelist(file_dir)
        for f in fileset:
            Name = os.path.splitext(f)[0]
            if f.endswith(".txt"):
                convert_h5py(file_dir+f,save_dir+Name+".h5py")
    elif load_single:
        print('load files '+filename)
        Name=os.path.splitext(filename)[0]
        convert_h5py(filename, save_dir+Name+'.h5py')
