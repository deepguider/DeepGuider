'''
Modified version for Logo Detection and Recognition 
from keras-yolo3 package, https://github.com/qqwweee/keras-yolo3
'''

import _init_paths
import numpy as np
import argparse
import os

import keras.backend as K
from keras.layers import Input, Lambda
from keras.models import Model
from keras.optimizers import Adam, SGD
from keras.callbacks import TensorBoard, ModelCheckpoint, ReduceLROnPlateau, EarlyStopping

from model import keras_yolo3
from keras_yolo3.yolo3.model import preprocess_true_boxes, yolo_body, tiny_yolo_body, yolo_loss
from keras_yolo3.yolo3.utils import get_random_data


def train(args):
    log_dir = os.path.join(args.model_path, args.log_dir)
    if not os.path.isdir(log_dir):
        os.mkdir(log_dir)
    print(log_dir + '\n')
    
    if args.is_openset:
        class_names = ['logo']
        n_classes = len(class_names)
    else: # closedset
        with open(args.classes_path, 'r') as f:
            class_names = [c.strip() for c in f.readlines()]
        n_classes = len(class_names)

    if args.dataset.lower() == 'litw':
        input_shape = (416, 416)
    else: # set according to the dataset
        input_shape = (256, 256)
    
    if not args.is_tiny:
        yolo_anchors_path = os.path.join(args.model_path, 'model_data/yolo_anchors.txt')
        weights_path = os.path.join(args.model_path, 'model_data/yolo_weights.h5')
    else: # original version
        yolo_anchors_path = os.path.join(args.model_path, 'model_data/tiny_yolo_anchors.txt')
        weights_path = os.path.join(args.model_path, 'model_data/yolo_tiny_weights.h5')

    with open(yolo_anchors_path, 'r') as f:
        anchors = f.readline()
    anchors = [float(a) for a in anchors.split(',')]
    anchors = np.array(anchors).reshape(-1, 2)
    
    model = create_model(args.is_tiny, input_shape, anchors, n_classes, freeze_body=2, weights_path=weights_path)

    logging = TensorBoard(log_dir=log_dir)
    checkpoint = ModelCheckpoint(os.path.join(log_dir, 'checkpoint.h5'),
        monitor='val_loss', save_weights_only=True, save_best_only=True, period=5)
    reduce_lr = ReduceLROnPlateau(monitor='val_loss', factor=0.1, patience=3, verbose=1)
    early_stopping = EarlyStopping(monitor='val_loss', min_delta=0, patience=10, verbose=1)

    with open(args.annotation_path) as f:
        anno = f.readlines()
    np.random.seed(1234)
    np.random.shuffle(anno)

    val_split = 0.1
    num_val = int(len(anno) * val_split)
    num_train = len(anno) - num_val
    
    if args.optimizer.lower() == 'adam':
        optimizer = Adam(lr=args.learning_rate)
    else: # SGD
        optimizer = SGD(lr=args.learning_rate)

    '''Train with frozen layers first, to get a stable loss.
    Adjust num epochs to your dataset. This step is enough to obtain a not bad model.'''
    model.compile(optimizer=optimizer, loss={'yolo_loss': lambda y_true, y_pred: y_pred})

    print(f'Train on {num_train} samples, validation on {num_val} samples, with batch size {args.batch_size}')
    train_generator = data_generator_wrapper(anno[:num_train], args.batch_size, input_shape, 
                                        anchors, n_classes)
    val_generator = data_generator_wrapper(anno[num_train:], args.batch_size, input_shape,
                                        anchors, n_classes)
    model.fit_generator(train_generator,
                        steps_per_epoch=max(1, num_train//args.batch_size),
                        validation_data=val_generator,
                        validation_steps=max(1, num_val//args.batch_size),
                        epochs=50,
                        initial_epoch=0,
                        callbacks=[logging, checkpoint])
    model.save_weights(log_dir + '/trained_weights_stage_1.h5')

    '''Unfreeze and continue training to fine-tune.
    Train longer if the result is not good.'''
    batch_size = args.batch_size // 2
    for i in range(len(model.layers)):
        model.layers[i].trainable = True
    # recompile to apply the change
    model.compile(optimizer=Adam(lr=1e-4), loss={'yolo_loss': lambda y_true, y_pred: y_pred})
    print(f'Unfreeze and train on {num_train} samples, validation on {num_val} samples, with batch size {batch_size}')
    
    train_generator = data_generator_wrapper(anno[:num_train], batch_size, input_shape,
                                            anchors, n_classes)
    val_generator = data_generator_wrapper(anno[num_train:], batch_size, input_shape,
                                            anchors, n_classes)
    model.fit_generator(train_generator,
                        steps_per_epoch=max(1, num_train//batch_size),
                        validation_data=val_generator,
                        validation_steps=max(1, num_val//batch_size),
                        epochs=100,
                        initial_epoch=50,
                        callbacks=[logging, checkpoint, reduce_lr, early_stopping])
    model.save_weights(log_dir + '/trained_weights_final.h5')
    print('Done...!')


def create_model(is_tiny, input_shape, anchors, n_classes, load_pretrained=True, 
        freeze_body=2, weights_path='model/keras_yolo3/model_data/yolo_weights.h5'):
    K.clear_session() # get a new session
    img = Input(shape=(None, None, 3))
    h, w = input_shape
    n_anchors = len(anchors)

    if not is_tiny:
        y_true = [Input(shape=(h//{0:32, 1:16, 2:8}[l], w//{0:32, 1:16, 2:8}[l], \
                        n_anchors//3, n_classes+5)) for l in range(3)]
        model_body = yolo_body(img, n_anchors//3, n_classes)
        ignore_thresh = 0.5
    else: # 'tiny'
        y_true = [Input(shape=(h//{0:32, 1:16}[l], w//{0:32, 1:16}[l],\
                        n_anchors//2, n_classes+5)) for l in range(2)]
        model_body = tiny_yolo_body(img, n_anchors//2, n_classes)
        ignore_thresh = 0.7
    print(f'Create YOLOv3 model with {n_anchors} anchors and {n_classes} classes')
    
    if load_pretrained:
        model_body.load_weights(weights_path, by_name=True, skip_mismatch=True)
        print(f'Load weights from {weights_path}')
        if freeze_body in [1, 2]:
            # Freeze darknet53 body or freeze all but 3 output layers.
            if not is_tiny:
                num = (185, len(model_body.layers)-3)[freeze_body-1]
            else:
                num = (20, len(model_body.layers)-2)[freeze_body-1]
            for i in range(num):
                model_body.layers[i].trainable = False
            print(f'Freeze the first {num} layers of total {model_body.layers} layers.')
    
    model_loss = Lambda(yolo_loss, output_shape=(1, ), name='yolo_loss',
            arguments={'anchors': anchors, 'num_classes': n_classes, 'ignore_thresh': ignore_thresh})([*model_body.output, *y_true])
    model = Model([model_body.input, *y_true], model_loss)

    return model


def data_generator(annotation, batch_size, input_shape, anchors, n_classes):
    '''data generator for fit_generator'''
    n = len(annotation)
    i = 0
    while True:
        image_data = []
        box_data = []
        for b in range(batch_size):
            if i == 0:
                np.random.shuffle(annotation)
            image, box = get_random_data(annotation[i], input_shape, random=True)
            image_data.append(image)
            box_data.append(box)
            i = (i+1) % n
        image_data = np.array(image_data)
        box_data = np.array(box_data)
        y_true = preprocess_true_boxes(box_data, input_shape, anchors, n_classes)
        yield [image_data, *y_true], np.zeros(batch_size) # produce a sequence of values


def data_generator_wrapper(annotation, batch_size, input_shape, anchors, n_classes):
    n = len(annotation)
    if n == 0 or batch_size <= 0:
        return None
    return data_generator(annotation, batch_size, input_shape, anchors, n_classes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default='train',
            help='Mode', choices=['train', 'test'])
    parser.add_argument('--dataset', type=str, default='litw', # logos-in-the-wild dataset
            help='Dataset', choices=['litw'])
    parser.add_argument('--model_path', type=str, default='model/keras_yolo3',
            help='Model path')
    parser.add_argument('--log_dir', type=str, default='logs/0002',
            help='Log directory path')
    parser.add_argument('--classes_path', type=str, default='data/preprocessed/classes.txt',
            help='Path where class_list.txt is stored')
    parser.add_argument('--annotation_path', type=str, default='data/preprocessed/yolo_train_list.txt',
            help='Path where annotation is stored')
    parser.add_argument('--is_tiny', type=bool, default=False,
            help='Select model size')
    parser.add_argument('--optimizer', type=str, default='Adam',
            help='Optimizer', choices=['Adam', 'SGD'])
    parser.add_argument('--learning_rate', type=float, default=1e-3,
            help='Learning rate')
    parser.add_argument('--batch_size', type=int, default=32,
            help='Batch size')
    parser.add_argument('--is_openset', type=bool, default=True,
            help='How to train the model')
    args = parser.parse_args()
    
    #if mode.lower() == 'train':
    train(args)
