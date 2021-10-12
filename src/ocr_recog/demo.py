import os

import string

import torch
import torch.utils.data
import torch.nn.functional as F
import time
from dataset import RawDataset,RawDataset_wPosition, AlignCollate
import numpy as np
from craft.Detection_txt import Detection_txt
import cv2
from PIL import ImageFont,Image,ImageDraw
from ipdb import set_trace as bp

#pred_filtering() by ETRI
from match_poiname import pred_filtering

def saveResult(img, boxes, pred_list, dirname, res_imagefileName):
    img = np.array(img)
    #res_img_file = dirname + 'result.jpg'
    #if res_imagefileName is not None :
    #    res_img_file = res_imagefileName
    res_img_file = dirname + 'result_' + res_imagefileName

    valid = []
    for i, box in enumerate(boxes):
        if pred_list[i][2] < 0.3:
            continue
        valid.append(pred_list[i])
        poly = np.array(box).astype(np.int32).reshape((-1))
        poly = poly.reshape(-1, 2)
        cv2.polylines(img, [poly.reshape((-1, 1, 2))], True, color=(0, 255, 0), thickness=3)
        ptColor = (0, 255, 255)

        # font = cv2.FONT_HERSHEY_SIMPLEX
        # font_scale = 0.5
        # cv2.putText(img, "{},{:.3f}".format(pred_list[i][1],pred_list[i][2].item() ), (poly[0][0]+1, poly[0][1]+1), font, font_scale, (0, 0, 0), thickness=1)

        font_size= 20
        # font = ImageFont.truetype("batang.ttf", 20)
        font = ImageFont.truetype("font/gulim.ttf", font_size)

        b, g, r, a = 0, 255, 0, 0
        #b, g, r, a = 255, 0, 0, 0
        img_pil = Image.fromarray(img)
        draw = ImageDraw.Draw(img_pil)
        text_y = max(poly[0][1]-font_size,0)
        draw.text((poly[0][0], text_y), "{},{:.3f}".format(pred_list[i][1],pred_list[i][2]), font=font, fill=(b, g, r, a))
        img = np.array(img_pil)

    cv2.imwrite(res_img_file, img)
    print('\ncheck result : ' + res_img_file)
    return valid

#per image
def detect_ocr(config, image, timestamp, save_img):
    
    if save_img:
        basename = os.path.basename(image)
    detection_list, img, boxes = Detection_txt(config,image,config.net)

    # print(detection_list)
    t = time.time()

    device = config.device
    #print("config device", device)
    model = config.model
    converter = config.converter

    # 32 * 100
    AlignCollate_demo = AlignCollate(imgH=config.imgH, imgW=config.imgW, keep_ratio_with_pad=config.PAD)
    # demo_data = RawDataset(root=image, opt=config)  # use RawDataset
    demo_data = RawDataset_wPosition(root=detection_list, opt=config)  # use RawDataset


    demo_loader = torch.utils.data.DataLoader(
        demo_data, batch_size=config.batch_size,
        shuffle=False,
        num_workers=int(config.workers),
        collate_fn=AlignCollate_demo, pin_memory=True)

    #(< PIL.Image.Image image mode=L size=398x120 at 0x7F376DAF30B8 >, './demo_image/demo_12.png')

    # predict
    model.eval()
    with torch.no_grad():
        log = open(f'{config.logfilepath}', 'a')
        dashed_line = '-' * 80
        head = f'{"coordinates":25s}\t{"predicted_labels":25s}\tconfidence score'
        if save_img: print(f'{dashed_line}\n{head}\n{dashed_line}')
        log.write(f'{dashed_line}\n{head}\n{dashed_line}\n')

        pred_list = []
        new_boxes = []
        for image_tensors, coordinate_list in demo_loader:
            batch_size = image_tensors.size(0)
            # print(image_tensors.shape)

            image = image_tensors.to(device)
            # For max length prediction
            length_for_pred = torch.IntTensor([config.batch_max_length] * batch_size).to(device)
            text_for_pred = torch.LongTensor(batch_size, config.batch_max_length + 1).fill_(0).to(device)


            preds = model(image, text_for_pred, is_train=False)

            # select max probabilty (greedy decoding) then decode index to character
            _, preds_index = preds.max(2)
            preds_str = converter.decode(preds_index, length_for_pred)

            preds_prob = F.softmax(preds, dim=2)
            preds_max_prob, _ = preds_prob.max(dim=2)

            for coordinate, pred, pred_max_prob in zip(coordinate_list, preds_str, preds_max_prob):

                pred_EOS = pred.find('[s]')

                pred = pred[:pred_EOS]  # prune after "end of sentence" token ([s])


#pred_filtering() by ETRI
                thre_filter = 0.6
                fitered_pred, dist_conf = pred_filtering(pred)


                if pred_EOS == 0: 
                    confidence_score = 0.0
                else:
                    pred_max_prob = pred_max_prob[:pred_EOS]
                    # calculate confidence score (= multiply of pred_max_prob)
                    confidence_score = pred_max_prob.cumprod(dim=0)[-1].item()


#pred_filtering() by ETRI
                if dist_conf >= thre_filter:
                    pred = fitered_pred
                    confidence_score = dist_conf


                #print(f'{coordinate}\t{pred:25s}\t{confidence_score:0.4f}')
                coordinate = list(coordinate)
                pred_list.append([coordinate, pred, confidence_score])
                #print(f'{coordinate}\t{pred:25s}\t{confidence_score:0.4f}')
                log.write(f'{coordinate}\t{pred:25s}\t{confidence_score:0.4f}\n')

        log.close()
    recog_time = time.time() - t
    config.recog_time = config.recog_time+recog_time

    # print("\nrun time (recognition) : {:.2f} , {:.2f} s".format(recog_time,config.recog_time))

    if save_img:
        pred_list = saveResult(img, boxes, pred_list, 
                               config.result_folder, basename)
        
    return  pred_list, timestamp
