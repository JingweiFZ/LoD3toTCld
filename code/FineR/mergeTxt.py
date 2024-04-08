import os

def mergeTxt(file_dir, save_filename):
    "merge text files"
    # Get a list of all files in the directory
    print('load file_dir:'+file_dir)
    file_names = [f for f in os.listdir(file_dir) if os.path.isfile(os.path.join(file_dir, f))]

    # 打开合并后的文件以写入模式
    with open(save_filename, 'w') as merged_file:
        # 遍历每个文件并逐行合并到合并后的文件中

        # Print the list of file names
        for file_name in file_names:
            print('read file:'+ file_name)
            with open(file_dir+file_name, 'r') as current_file:
                # 读取当前文件的内容并写入合并后的文件
                merged_file.write(current_file.read())
                # 添加分隔符（可选）
                #"merged_file.write("\n---\n")  # 可以根据需要添加分隔符

        print(f"All files have been merged into {save_filename}.")


if __name__ == '__main__':
    file_dir='G:/Application/workdir/average/labeled/'
    filename=''
    save_filename='G:/Application/workdir/average/Whole.txt'

    mergeTxt(file_dir,save_filename)