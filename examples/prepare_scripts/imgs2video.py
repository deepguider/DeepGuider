import cv2
import numpy as np
import os
import glob
import imutils  # pip install imutils
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--in_imgpath', type=str, default='../pano_img/20210726_ETRIbuilding12_7floor', help = '') 
    parser.add_argument('--ext', type=str, default='JPG', help = '') 
    parser.add_argument('--out_videopath', type=str, default='./video_out.avi', help = '') 
    parser.add_argument('--draw_rect', action='store_true', help='')
    parser.add_argument('--no_display', action='store_true', help='')
    parser.add_argument('--fps', type=float, default=29, help = '')
    parser.add_argument('--watermark', type=str, default=[], help = '') 
    opt = parser.parse_args()

    avi_out = None
    frame_cnt = 0
    imglist = glob.glob(os.path.join(opt.in_imgpath, "*." + opt.ext))
    imglist.sort()

    for frame_cnt, fname in enumerate(imglist):
        # Read image from video
        frame = cv2.imread(fname)
        H,W,C = frame.shape
        frame = imutils.resize(frame, height=480)

        # Do your own process
        if len(opt.watermark) > 0:
            text1 = "[{}] frame - {}".format(opt.watermark, frame_cnt)
            (x, y) = (20, 20)
            cv2.putText(frame, text1, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        # Write result into video
        out_cv = cv2.resize(frame, (1280, 720))  ## Resize is required for writing image to mp4 file.
        if avi_out == None:
            h, w = out_cv.shape[:2]
            avi_out = cv2.VideoWriter(opt.out_videopath, 0x7634706d, opt.fps, (w, h)) # write ad mp4
        avi_out.write(out_cv)
        print("Frame : {} \r".format(frame_cnt), end='')
        frame_cnt += 1

        # Display result
        if opt.no_display == False:
            cv2.imshow('ReadWriteVideo', out_cv)
        retkey = cv2.waitKey(1) & 0xff
        if retkey == ord('q'):  # quit
            break
        elif retkey == 27:  # esc to quit
            break
        else:
            continue

    cv2.destroyAllWindows()
