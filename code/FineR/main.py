# 这是一个示例 Python 脚本。

# 按 Shift+F10 执行或将其替换为您的代码。
# 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。
import numpy as np
import json
from PIL import Image
import matplotlib.pyplot as plt
import cv2
import os


def print_hi(name):
    # 在下面的代码行中使用断点来调试脚本。
    print(f'Hi, {name}')  # 按 Ctrl+F8 切换断点。

def extract_nested_field(data, field_name):
    if isinstance(data, dict):
        if field_name in data:
            print(data[field_name])
        for key, value in data.items():
            extract_nested_field(value, field_name)
    elif isinstance(data, list):
        for item in data:
            extract_nested_field(item, field_name)


def print_json_fields(data, parent_field=None):
    if isinstance(data, dict):
        for key, value in data.items():
            field_name = f"{parent_field}.{key}" if parent_field else key
            print(field_name)
            print_json_fields(value, field_name)
    elif isinstance(data, list):
        for i, item in enumerate(data):
            field_name = f"{parent_field}[{i}]" if parent_field else str(i)
            print(field_name)
            print_json_fields(item, field_name)

# 递归函数来获取所有字段并保存到文本文件
def get_and_save_json_fields(data, parent_field=None, file=None):
    if isinstance(data, dict):
        for key, value in data.items():
            field_name = f"{parent_field}.{key}" if parent_field else key
            file.write(field_name + "\n")
            get_and_save_json_fields(value, field_name, file)
    elif isinstance(data, list):
        for i, item in enumerate(data):
            field_name = f"{parent_field}[{i}]" if parent_field else str(i)
            file.write(field_name + "\n")
            get_and_save_json_fields(item, field_name, file)

# 打开文本文件以保存字段信息


# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    print_hi('PyCharm')

    filedir = 'G:/Application/TBBR/'
    with open(filedir+'Flug1_100-104Media_coco.json', 'r') as f:
        fjson = json.load(f)

    # 遍历所有字段并打印它们
    info=fjson['info']
    images=fjson['images']
    annotations=fjson['annotations']

    tir_dir='G:/Application/TBBR/train/tir/'
    rgb_dir='G:/Application/TBBR/train/rgb/'
    label_dir='G:/Application/TBBR/train/label/'



    for image_file in images:
        image_name=image_file['file_name']
        image_id=image_file['id']
        image_height=image_file['height']
        image_width=image_file['width']

        mid = 'Flug1_100/train/images/'
        image_name = filedir + mid+image_name

        parts = image_name.split('/')
        # 获取最后一个部分
        last_part = parts[-1]
        # 使用split()方法将最后一个部分分割成文件名和扩展名
        filename, extension = last_part.split('.')

        if os.path.exists(image_name):
            print(image_name)
            data=np.load(image_name)
            rgb=data[:,:,0:3]
            tir=data[:,:,3]
            rgb=rgb.astype(np.uint8)
            tir=tir.astype(np.uint8)

            cv2.imwrite(rgb_dir+filename+'.jpg',rgb)
            cv2.imwrite(tir_dir+filename+'.jpg',tir)

            # matching_annotations = [annotation["segmentation"] for annotation in annotations if
            #                         annotation["image_id"] == image_id]

            #image = Image.new('1', (image_width, image_height), 0)
            image = np.zeros((image_height, image_width), dtype=np.uint8)
            for annotation in annotations:
                if annotation["image_id"] == image_id:
                    for seg in annotation["segmentation"]:

                        #seg=annotation["segmentation"][0]

                        # 使用列表切片将x坐标和y坐标分开
                        x_coordinates = seg[::2]  # 奇数位是x坐标
                        y_coordinates = seg[1::2]  # 偶数位是y坐标

                        # 构建多边形的坐标
                        polygon_coordinates = []
                        for x, y in zip(x_coordinates, y_coordinates):
                            polygon_coordinates.append((x, y))
                        # 绘制多边形
                        #"for coords in polygon_coordinates:
                        pts = np.array(polygon_coordinates, dtype=np.int32)
                        pts = pts.reshape((-1, 2))
                        cv2.fillPoly(image, [pts], 200)  # 用1填充多边形内部

            cv2.imwrite(label_dir+filename+'_label'+'.jpg', image)
            #         image.close()
            print('Finish! ')
        else:
            print('File not exist: '+image_name)

    #     break
    # with open(filedir+"Flug1_100-104Media_coco.txt", "w") as file:
    #     get_and_save_json_fields(fjson, file=file)
    #
    # data=np.load('D:/download/Flug1_105/test/images/Flug1_105/DJI_0022_R.npy')
    #
    # da = data[:,:,4]
    # # 显示图像
    # plt.imshow(da, cmap='gray')  # 使用灰度图像的 colormap，适用于单通道图像
    # plt.title('Image Title')
    # plt.axis('off')  # 隐藏坐标轴
    # plt.show()
   

# 访问 https://www.jetbrains.com/help/pycharm/ 获取 PyCharm 帮助
