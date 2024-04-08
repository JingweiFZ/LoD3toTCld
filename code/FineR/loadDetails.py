import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import utils
import math
from sklearn.linear_model import LinearRegression


# Load the data
def load_data(filename):
    dfa = pd.read_csv(filename, delimiter="\t", header=None)
    dfa.columns = ['k1', 'k2', 'k3', 'x', 'y', 'z', 'w', 'intensity', 'std', 'label']
    return dfa


def load_detail(filename):
    dfa = pd.read_csv(filename, delimiter='\t', header=None)
    dfa.columns = ['k1', 'k2', 'k3', 'x', 'y', 'z', 'intensity', 'distance', 'angle']
    return dfa


def uniqueIndex(data):
    # Get the unique values and their frequency in column 'a'
    unique_combines = data[['k1', 'k2', 'k3']].drop_duplicates()
    uniqueValues, occurCount = np.unique(data, return_counts=True)
    # print("Unique Values : " , uniqueValues)
    # print("Occurrence Count : ", occurCount)
    return unique_combines


def Anafit(df):
    uniquekey = uniqueIndex(df)

    for index, row in uniquekey.iterrows():
        a = row['k1']
        b = row['k2']
        c = row['k3']

        filtered_df = df[(df['a'] == a) & (df['b'] == b) & (df['c'] == c)]


if __name__ == '__main__':
    file_dir = 'G:/Application/workdir/details/labeled/'
    filename = 'merge_voxel_ang_0_11_labeled.txt'
    save_dir = ''

    load_all = True
    load_single = False

    data = pd.DataFrame()

    if load_all:
        '''load all the files from directory'''
        print('Load all the files from ' + file_dir)
        filelist = utils.getfilelist(file_dir)

        for f in filelist:
            if f.endswith(".txt"):
                df = load_detail(file_dir + f)
                data = pd.concat([data, df], axis=0, ignore_index=True)
    elif load_single:
        data = load_detail(file_dir + filename)

    save_ukey = save_dir + 'unique_keys.txt'
    print('Print and save the unique key: ' + save_ukey)
    uni_keys = uniqueIndex(df)
    np.savez(save_ukey, uni_keys)

    print('Analyze the details')
    data['cos'] = math.cos(data['angle'])

    print(uni_keys)
