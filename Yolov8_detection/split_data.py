
import os
import glob
import xml.etree.ElementTree as ET
from PIL import Image
import random

data_dir = r"C:\Users\hp\Downloads\data"
save_dir = r"C:\Users\hp\Downloads\data_space\Annotations"

def rename_data(data_dir, train_ratio=0.8, val_ratio=0.1, test_ratio=0.1):
    """
    Split data into train, val, and test sets.
    """
    img_dirs = glob.glob(data_dir+"/*")
    for img_dir in img_dirs:
        dir_name = os.path.basename(img_dir)
        img_files = glob.glob(os.path.join(img_dir, "*.xml"))
        for i, img_file in enumerate(img_files):
            tree = ET.parse(img_file)
            root = tree.getroot()  
            tree.write(os.path.join(save_dir, dir_name+os.path.basename(img_file)), encoding='utf-8', xml_declaration=True)
            print(f"Saved {i+1}/{len(img_files)} images to {save_dir}")
            # image = Image.open(img_file)
            # image.save(os.path.join(save_dir, dir_name+"_"+os.path.basename(img_file)))
            # print(f"Saved {i+1}/{len(img_files)} images to {save_dir}")

annotation_dir = r"C:\Users\hp\Downloads\data_space\Annotations"
img_dir = r"C:\Users\hp\Downloads\data_space\images"
save_split_dir = r"C:\Users\hp\Downloads\data_space\ImageSets"

def split_data(annotation_dir, img_dir, train_ratio=0.8, val_ratio=0.1, test_ratio=0.1):
    """
    Split data into train, val, and test sets.
    """
    total_xml = os.listdir(annotation_dir)
 
    num = len(total_xml)
    list = range(num)
    tv = int(num * (train_ratio + val_ratio))
    tr = int(num * train_ratio)
    trainval = random.sample(list, tv)
    train = random.sample(trainval, tr)
    
    ftrainval = open(os.path.join(save_split_dir, 'trainval.txt'), 'w')
    ftest = open(os.path.join(save_split_dir, 'test.txt'), 'w')
    ftrain = open(os.path.join(save_split_dir, 'train.txt'), 'w')
    fval = open(os.path.join(save_split_dir, 'val.txt'), 'w')
    
    for i in list:
        name = total_xml[i][:-4] + '\n'
        if i in trainval:
            ftrainval.write(name)
            if i in train:
                ftrain.write(name)
            else:
                fval.write(name)
        else:
            ftest.write(name)
    
    ftrainval.close()
    ftrain.close()
    fval.close()

split_data(annotation_dir, img_dir)

# rename_data(data_dir)
