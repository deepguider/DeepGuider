'''
Data preprocessing for training the model.
Extract voc-style annotations from .xml file.
Modified from Logohunter, https://github.com/ilmonteux/logohunter
'''

import numpy as np
import os
import argparse
import xml.etree.ElementTree as ET

from tqdm import tqdm
from pathlib import Path


def main(args):
    if not os.path.exists(args.result_path):
        os.mkdir(args.result_path)

    train_file = open(os.path.join(args.result_path, 'yolo_train_list.txt'), 'w')
    test_file = open(os.path.join(args.result_path, 'test_list.txt'), 'w')
    DB_file = open(os.path.join(args.result_path, 'DB_list.txt'), 'w')
    
    cleaned_data = os.path.join(args.cleaned_data, 'brands.txt')
    if not os.path.exists(cleaned_data):
        raise FileNotFoundError("Could not find 'brands.txt' file.")
    with open(cleaned_data, 'r') as f:
        classes = [line.replace('\n', '') for line in f.readlines()]
    
    classes_file = os.path.join(args.result_path, 'classes.txt')
    with open(classes_file, 'w') as f:
        for c in classes:
            f.write(f'{c}\n')
    openset_file = os.path.join(args.result_path, 'openset_classes.txt')
    with open(openset_file, 'w') as f:
        f.write('logo')
    
    total = 0
    num_train = 0
    cat_path = list(Path(os.path.join(args.cleaned_data, 'voc_format')).iterdir())
    for cat in tqdm(cat_path):
        cat = str(cat)
        os.rename(cat, cat.replace(' ', ''))
        cat = cat.replace(' ', '')
        xml_path = list(Path(cat).iterdir())
        for xml in xml_path:
            if str(xml)[-4:] != '.xml':
                continue
            total += 1

            rand = np.random.rand()
            save_file = train_file if rand > args.split_ratio else test_file
            xml = os.path.abspath(str(xml))

            save_file.write(os.path.normpath(xml[:-4] + '.jpg'))
            if rand > args.split_ratio:
                DB_file.write(os.path.normpath(xml[:-4] + '.jpg'))
                num_train += 1

            for cls_id, bboxes in xml_extractor(str(xml), classes):
                save_file.write(' ' + ','.join([str(b) for b in bboxes]) + ',' + str(0))
                if rand > args.split_ratio:
                    DB_file.write(' ' + ','.join([str(b) for b in bboxes]) + ',' + str(cls_id))
            save_file.write('\n')
            if rand > args.split_ratio:
                DB_file.write('\n')

    train_file.close()
    test_file.close()
    DB_file.close()

    print(f'Total dataset: {total}, Training set: {num_train}')


def xml_extractor(xml_file, classes):
    with open(xml_file, 'r') as f:
        tree = ET.parse(f)
        root = tree.getroot()
        
    anno = []
    for obj in root.iter('object'):
        cls = obj.find('name').text
        difficult = obj.find('difficult').text
        if cls not in classes or int(difficult) == 1:
            continue
        cls_id = classes.index(cls)
        bbox = obj.find('bndbox')
        bbox = (int(bbox.find('xmin').text), int(bbox.find('ymin').text), int(bbox.find('xmax').text), int(bbox.find('ymax').text))
        
        anno.append([cls_id, bbox])

    return anno


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_path', type=str, default='./data/LogosInTheWild-v2',
            help='Path to directory contatining images and xml annotations')
    parser.add_argument('--result_path', type=str, default='./data/preprocessed',
            help='Path to save the preprocessed data')
    parser.add_argument('--cleaned_data', type=str, 
            default='./data/LogosInTheWild-v2/cleaned_data')
    parser.add_argument('--split_ratio', type=float, default=0.2,
            help='Fraction of dataset split')
    args = parser.parse_args()

    main(args)
