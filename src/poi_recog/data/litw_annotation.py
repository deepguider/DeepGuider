import xml.etree.ElementTree as ET
import os
import numpy as np
import argparse


def convert_annotation(xml_file, classes):
    """Extract VOC-style annotations from XML input fileself.
    Returns:
      list of [int, bbox], one for each object, where int is the class id and
      bbox=(xmin, ymin, xmax, ymax) is the bounding box specification.
    """
    with open(xml_file, 'r') as in_file:
        tree = ET.parse(in_file)
        root = tree.getroot()

        output = []
        for obj in root.iter('object'):
            difficult = obj.find('difficult').text
            cls = obj.find('name').text
            if cls not in classes or int(difficult)==1:
                continue
            cls_id = classes.index(cls)
            xmlbox = obj.find('bndbox')
            bboxes = (int(xmlbox.find('xmin').text), int(xmlbox.find('ymin').text), int(xmlbox.find('xmax').text), int(xmlbox.find('ymax').text))

            output.append([cls_id, bboxes])
    return output


def main(args):
    img_path = args.img_path
    classes = args.classes
    out_path = args.out_path
        
    if not os.path.exists(out_path):
        os.mkdir(out_path)

    train_file = open(os.path.join(out_path, 'train.txt'), 'w')
    test_file = open(os.path.join(out_path, 'test.txt'), 'w')

    with open(classes, 'r') as f:
        classes = [line.replace('\n','') for line in f.readlines()]

    with open(os.path.join(out_path, 'classes.txt'), 'w') as f:
        if not args.closedset:
            f.write('logo\n')
        else:
            for c in classes:
                f.write('{}\n'.format(c))

    for folder in os.listdir(img_path):
        folder_path = os.path.abspath(os.path.join(img_path, folder))
        if ' ' in folder_path:
            os.rename(folder_path, folder_path.replace(' ', ''))
            folder_path = folder_path.replace(' ', '')
        if not os.path.isdir(folder_path):
            continue

        if args.split_class_or_file == 0:
            rand = np.random.rand()
        for file in os.listdir(folder_path):
            filename = os.path.splitext(file)[0]
            if (not file.endswith('.xml')) or (not os.path.exists(os.path.join(folder_path,filename+'.jpg'))):
                continue

            if args.split_class_or_file == 1:
                rand = np.random.rand()

            list_file = train_file if rand > args.split_ratio else test_file

            list_file.write(os.path.normpath(os.path.join(folder_path, filename+'.jpg')))

            for cls_id, bboxes in convert_annotation(os.path.join(folder_path, file), classes):
                # squash all logos to one class
                if not args.closedset:
                    cls_id = 0

                list_file.write(" " + ",".join([str(a) for a in bboxes]) + ',' + str(cls_id))

            list_file.write('\n')

    train_file.close()
    test_file.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
            description='Process image annotation from VOC style to keras-yolo3 style.', 
            formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--img_path', type=str, 
            default='./LogosInTheWild-v2/data_cleaned/voc_format',
            help='path to directory containing images and xml annotations')
    parser.add_argument('--out_path', type=str, 
            default='./preprocessed/',
            help='path to directory for storing output.')
    parser.add_argument('--classes', type=str, 
            default='./LogosInTheWild-v2/data_cleaned/brands.txt',
            help='path to txt file listing all possible object classes')
    parser.add_argument('--split_ratio', type=float, default = 0.3,
            help='fraction of dataset set apart for test')
    parser.add_argument('--split_class_or_file', type=int, default = 1,
            help='Do train/test split at class (0) or file level (1)')
    parser.add_argument('--closedset', default=False, action="store_true",
            help='If specified, annotate logo objects class by class instead of as one class')
    parser.add_argument('--seed', default=False, action="store_true",
            help='If specified, set seed = 0 for reproducible train/test split')
    args = parser.parse_args()

    if args.seed: 
        np.random.seed(0)

    main(args)
