from PIL import Image
import os
import time

def count_korean1():

    fw = open('../dataset/gt_korean.txt','w',encoding='utf-8')

    with open('../dataset/gt.txt','r',encoding='utf-8') as f:
        cnt = 0
        for s in f :
            if s.split(',')[1] == 'Korean' :
                cnt = cnt + 1
                fw.write(s.split(',')[0]+'\t'+s.split(',')[2])
                print(s)
    print(cnt)
    fw.close()

def count_korean():

    fw = open('../dataset/gt_korean_2.txt','w',encoding='utf-8')

    with open('../dataset/gt_korean.txt','r',encoding='utf-8') as f:
        cnt = 0
        for s in f :
            fw.write('Korean_png/'+s)
            cnt = cnt + 1
    print(cnt)
    fw.close()

def crop_n_save_img():
    start = time.time()
    root = '/home_hongdo/sungeun.kim/dataset/ocr_korean/'
    root = '/home2/sungeun.kim/project/DeepGuider/src/ocr_recog/utils/'

    img = '/home_hongdo/sungeun.kim/dataset/ocr_korean/ant+hill_1_0.jpg'
    gt = '/home_hongdo/sungeun.kim/dataset/ocr_korean/ant+hill_1_0.txt'

    img_root = '/home_hongdo/sungeun.kim/dataset/ocr_korean/Korean'  #gt_txts
    gt_txt_root = '/home_hongdo/sungeun.kim/dataset/ocr_korean/Korean_gt/Korean'
    img_root_new = '/home_hongdo/sungeun.kim/dataset/ocr_korean/Korean_crop/'

    img_gt_new = '/home_hongdo/sungeun.kim/dataset/ocr_korean/gt.txt'

    fw = open(img_gt_new, 'w', encoding='utf-8')


    cnt = 1

    if os.path.isdir(img_root):
        for dirpath, dirnames, filenames in os.walk(gt_txt_root):
            for name in filenames:

                filename  = os.path.join(dirpath, name)
                # print(filename)

                nameonly, ext = os.path.splitext(name)

                image_path = os.path.join(img_root, nameonly+'.jpg')
                # print(image_path )




                with open(filename, 'r', encoding='utf-8') as f:
                    # image_path = gt[:-4]+'.jpg'
                    img = Image.open(image_path).convert('RGB')
                    for s in f :
                        x1 = float(s.split(',')[0])
                        y1 = float(s.split(',')[1])
                        x2= float(s.split(',')[2])
                        y2= float(s.split(',')[3])
                        x3= float(s.split(',')[4])
                        y3= float(s.split(',')[5])
                        x4= float(s.split(',')[6])
                        y4= float(s.split(',')[7])

                        label = s.split(',')[-1]

                        x_min = min(x1,x2,x3,x4)
                        x_max = max(x1,x2,x3,x4)

                        y_min = min(y1, y2, y3, y4)
                        y_max = max(y1, y2, y3, y4)


                        crop_position = (x_min,y_min,x_max,y_max)

                        imgCrop = img.crop(crop_position)
                        new_img_path = img_root_new+str(cnt)+'.png'
                        imgCrop.save(new_img_path)

                        fw.write(new_img_path+'\t'+label)
                        print(cnt)
                        cnt=cnt+1

    print(cnt-1)
    fw.close()


    print('Initialization Done! It tooks {:.2f} mins.\n'.format((time.time() - start) / 60))

def crop_n_save_img_v2():
    start = time.time()

    img_path = '/home2/sungeun.kim/project/DeepGuider/src/ocr_recog/demo_image/all/demo_11_all.png'
    gt_txt_root = '/home2/sungeun.kim/project/CRAFT-pytorch/result/res_demo_11_all.txt'
    img_root_new = '/home2/sungeun.kim/project/CRAFT-pytorch/result/crop/'

    cnt = 0

    with open(gt_txt_root, 'r', encoding='utf-8') as f:
        # image_path = gt[:-4]+'.jpg'
        img = Image.open(img_path).convert('RGB')

        for s in f:

            x1 = float(s.split(',')[0])
            y1 = float(s.split(',')[1])
            x2 = float(s.split(',')[2])
            y2 = float(s.split(',')[3])
            x3 = float(s.split(',')[4])
            y3 = float(s.split(',')[5])
            x4 = float(s.split(',')[6])
            y4 = float(s.split(',')[7])



            x_min = min(x1, x2, x3, x4)
            x_max = max(x1, x2, x3, x4)

            y_min = min(y1, y2, y3, y4)
            y_max = max(y1, y2, y3, y4)

            crop_position = (x_min, y_min, x_max, y_max)
            print(crop_position)

            imgCrop = img.crop(crop_position)
            new_img_path = img_root_new + str(cnt) + '.png'
            imgCrop.save(new_img_path)
            cnt  = cnt +1

    print('Initialization Done! It tooks {:.2f} mins.\n'.format((time.time() - start) / 60))
# count_korean()
crop_n_save_img_v2()