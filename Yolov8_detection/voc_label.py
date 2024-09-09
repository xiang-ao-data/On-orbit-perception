import xml.etree.ElementTree as ET
import os
from os import getcwd
 
sets = ['train', 'val', 'test']
classes = ['spacecraft']
# abs_path = os.getcwd()
# print(abs_path)
 
 
def convert(size, box):
    dw = 1. / (size[0])
    dh = 1. / (size[1])
    x = (box[0] + box[1]) / 2.0 - 1
    y = (box[2] + box[3]) / 2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x * dw
    w = w * dw
    y = y * dh
    h = h * dh
    return x, y, w, h
 
 
def convert_annotation(annotation_path,label_path,image_id):
    in_file = open(os.path.join(annotation_path,'%s.xml' % (image_id)), encoding='UTF-8')
    out_file = open(os.path.join(label_path,'%s.txt' % (image_id)), 'w')
    print(os.path.join(annotation_path,'%s.xml' % (image_id)))
    tree = ET.parse(in_file)
    root = tree.getroot()
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)
    for obj in root.iter('object'):
        # difficult = obj.find('difficult').text
        difficult = obj.find('difficult').text
        cls = obj.find('name').text
        if cls not in classes or int(difficult) == 1:
            continue
        cls_id = classes.index(cls)
        xmlbox = obj.find('bndbox')
        b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text),
             float(xmlbox.find('ymax').text))
        b1, b2, b3, b4 = b
        # 标注越界修正
        if b2 > w:
            b2 = w
        if b4 > h:
            b4 = h
        b = (b1, b2, b3, b4)
        bb = convert((w, h), b)
        out_file.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')
 
save_split_dir = r"C:\Users\hp\Downloads\data_space\ImageSets"
img_dir = r"C:\Users\hp\Downloads\data_space\images"
dir = r"C:\Users\hp\Downloads\data_space"
label_dir = r"C:\Users\hp\Downloads\data_space\labels"
annotation_dir = r"C:\Users\hp\Downloads\data_space\annotations"

for image_set in sets:
    image_ids = open(os.path.join(save_split_dir, '%s.txt' % (image_set))).read().strip().split()
    list_file = open(dir+'\%s.txt' % (image_set), 'w')
    for image_id in image_ids:
        list_file.write(img_dir+ '\%s.png\n' % (image_id))
        convert_annotation(annotation_dir,label_dir,image_id)
list_file.close()