import string
import argparse

import torch
import torch.backends.cudnn as cudnn
import torch.utils.data
import torch.nn.functional as F

from dataset import RawDataset, AlignCollate



#per image
def detect_ocr(config, image, timestamp):


    device = config.device
    model = config.model
    converter = config.converter


    AlignCollate_demo = AlignCollate(imgH=config.imgH, imgW=config.imgW, keep_ratio_with_pad=config.PAD)
    demo_data = RawDataset(root=image, opt=config)  # use RawDataset
    demo_loader = torch.utils.data.DataLoader(
        demo_data, batch_size=config.batch_size,
        shuffle=False,
        num_workers=int(config.workers),
        collate_fn=AlignCollate_demo, pin_memory=True)

    # predict
    model.eval()
    with torch.no_grad():
        for image_tensors, image_path_list in demo_loader:
            batch_size = image_tensors.size(0)
            image = image_tensors.to(device)
            # For max length prediction
            length_for_pred = torch.IntTensor([config.batch_max_length] * batch_size).to(device)
            text_for_pred = torch.LongTensor(batch_size, config.batch_max_length + 1).fill_(0).to(device)


            preds = model(image, text_for_pred, is_train=False)

            # select max probabilty (greedy decoding) then decode index to character
            _, preds_index = preds.max(2)
            preds_str = converter.decode(preds_index, length_for_pred)

            log = open(f'{config.logfilepath}', 'a')
            dashed_line = '-' * 80
            head = f'{"image_path":25s}\t{"predicted_labels":25s}\tconfidence score'

            print(f'{dashed_line}\n{head}\n{dashed_line}')
            log.write(f'{dashed_line}\n{head}\n{dashed_line}\n')

            preds_prob = F.softmax(preds, dim=2)
            preds_max_prob, _ = preds_prob.max(dim=2)
            for img_name, pred, pred_max_prob in zip(image_path_list, preds_str, preds_max_prob):

                pred_EOS = pred.find('[s]')
                pred = pred[:pred_EOS]  # prune after "end of sentence" token ([s])
                pred_max_prob = pred_max_prob[:pred_EOS]

                # calculate confidence score (= multiply of pred_max_prob)
                confidence_score = pred_max_prob.cumprod(dim=0)[-1]

                print(f'{img_name:25s}\t{pred:25s}\t{confidence_score:0.4f}')
                log.write(f'{img_name:25s}\t{pred:25s}\t{confidence_score:0.4f}\n')

            log.close()

    return pred, timestamp