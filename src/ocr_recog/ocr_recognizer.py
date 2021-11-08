import os
os.environ["CUDA_VISIBLE_DEVICES"]="0"  #CUDA_VISIBLE_DEVICES=0 (always use the first GPU only)

import time
import string
import argparse

import torch
import torch.backends.cudnn as cudnn
import torch.utils.data

from utils import AttnLabelConverter
from model import Model

from demo import detect_ocr
from craft.craft import CRAFT
from collections import OrderedDict

#####################################
# 21.06.04 Astrid
# https://github.com/googleapis/oauth2client/issues/642#issuecomment-279643203
'''
Solving this error 
File "./../src/ocr_recog/ocr_recognizer.py", line 41, in __init__
    self.opt_craft, self.opt_recog = self.setup_parser()
  File "./../src/ocr_recog/ocr_recognizer.py", line 120, in setup_parser
    parser_craft = argparse.ArgumentParser(description='CRAFT Text Detection')
  File "/usr/lib/python3.6/argparse.py", line 1635, in __init__
    prog = _os.path.basename(_sys.argv[0])
AttributeError: module 'sys' has no attribute 'argv'
'''
import sys

if not hasattr(sys, 'argv'):
    sys.argv  = ['']
#####################################

def str2bool(v):
    return v.lower() in ("yes", "y", "true", "t", "1")

def copyStateDict(state_dict):
    if list(state_dict.keys())[0].startswith("module"):
        start_idx = 1
    else:
        start_idx = 0
    new_state_dict = OrderedDict()
    for k, v in state_dict.items():
        name = ".".join(k.split(".")[start_idx:])
        new_state_dict[name] = v
    return new_state_dict

class OCRRecognizer:
    def __init__(self):
        self.net = None #detect
        self.model = None #recog
        self.converter = None
        #self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        self.res_imagefileName = None

        self.opt_craft, self.opt_recog = self.setup_parser()

        self.args_craft= vars(self.opt_craft)
        self.args = vars(self.opt_recog)

        self.detect_time = 0.0
        self.recog_time = 0.0
        self.total_time =0.0
        # print("~~~~~~~~ Hyperparameters used: ~~~~~~~")
        # for x, y in self.args.items():
        #     print("{} : {}".format(x, y))
        self.__dict__.update(self.args_craft)
        self.__dict__.update(self.args)


    def initialize(self):

        start = time.time()



        # self.saved_model = '/home_hongdo/sungeun.kim/checkpoints/ocr/ocr_train_addKorean_synth/best_accuracy.pth'
        # self.craft_trained_model = '/home_hongdo/sungeun.kim/checkpoints/ocr/ocr_train/craft_mlt_25k.pth'
        # self.saved_model = '/home_hongdo/sungeun.kim/checkpoints/ocr/ocr_train_v2/best_accuracy.pth'
        # self.craft_trained_model = '/home_hongdo/sungeun.kim/checkpoints/ocr/ocr_train_v2/best_accuracy_craft.pth'
        #
        # official

        self.saved_model = './data_ocr/best_accuracy.pth'
        self.craft_trained_model = './data_ocr/best_craft.pth'
        self.logfilepath = './data_ocr/log_ocr_result.txt'
        
        if torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.cuda = True
            cudnn.benchmark = False
        else:
            self.device = torch.device('cpu')
            self.cuda = False
            cudnn.benchmark = True


        """ vocab / character number configuration """
        # if self.sensitive:
        #     self.character = string.printable[:-6]  # same with ASTER setting (use 94 char).
        cudnn.deterministic = True

        #self.num_gpu = torch.cuda.device_count()

        """ model configuration """
        # detetion
        self.net = CRAFT(self).to(self.device)  # initialize
        print('Loading detection weights from checkpoint ' + self.craft_trained_model)
        self.net.load_state_dict(copyStateDict(torch.load(self.craft_trained_model, map_location=self.device)))
        #self.net = torch.nn.DataParallel(self.net).to(self.device)
        self.net.to(self.device)

        self.converter = AttnLabelConverter(self.character)
        self.num_class = len(self.converter.character)

        if self.rgb:
            self.input_channel = 3
        self.model = Model(self, self.num_class).to(self.device)
        # load model
        #self.model = torch.nn.DataParallel(self.model).to(self.device)
        print('Loading recognition weights from checkpoint %s' % self.saved_model)
        #ckpt = torch.load(self.saved_model, map_location=self.device)
        self.model.load_state_dict(torch.load(self.saved_model, map_location=self.device))
        self.model.to(self.device)
        
        print('Initialization Done! It tooks {:.2f} sec.\n'.format(time.time() - start))
        return True

    def setup_parser(self):
        """
        Sets up an argument parser
        """

        parser_craft = argparse.ArgumentParser(description='CRAFT Text Detection')

        parser_craft.add_argument('--craft_trained_model', default='weights/craft_mlt_25k.pth', type=str,
                                  help='pretrained model')
        parser_craft.add_argument('--text_threshold', default=0.7, type=float, help='text confidence threshold')
        parser_craft.add_argument('--low_text', default=0.4, type=float, help='text low-bound score')
        parser_craft.add_argument('--link_threshold', default=0.4, type=float, help='link confidence threshold')
        parser_craft.add_argument('--cuda', default=False, type=str2bool, help='Use cuda for inference')
        parser_craft.add_argument('--canvas_size', default=1280, type=int, help='image size for inference')
        parser_craft.add_argument('--mag_ratio', default=1.5, type=float, help='image magnification ratio')
        parser_craft.add_argument('--poly', default=False, action='store_true', help='enable polygon type')
        parser_craft.add_argument('--show_time', default=False, action='store_true', help='show processing time')
        parser_craft.add_argument('--test_folder', default='/data/', type=str, help='folder path to input images')
        parser_craft.add_argument('--result_folder', default='./results/', type=str, help='result folder path')
        parser_craft.add_argument('--refine', default=False, action='store_true', help='enable link refiner')
        parser_craft.add_argument('--refiner_model', default='weights/craft_refiner_CTW1500.pth', type=str,
                                  help='pretrained refiner model')

        args_craft = parser_craft.parse_args()

        parser_recog = argparse.ArgumentParser(description='ocr recognition')
        parser_recog.add_argument('--image_path', help='path to image_folder or image_file which contains text images')
        parser_recog.add_argument('--workers', type=int, help='number of data loading workers', default=4)
        parser_recog.add_argument('--batch_size', type=int, default=1, help='input batch size')
        parser_recog.add_argument('--saved_model', help="path to saved_model to evaluation")
        parser_recog.add_argument('--logfilepath', help="path to log to demo")

        """ Data processing """
        parser_recog.add_argument('--batch_max_length', type=int, default=25, help='maximum-label-length')
        parser_recog.add_argument('--imgH', type=int, default=32, help='the height of the input image')
        parser_recog.add_argument('--imgW', type=int, default=100, help='the width of the input image')
        parser_recog.add_argument('--rgb', action='store_true', help='use rgb input')
        # parser.add_argument('--character', type=str, default='0123456789abcdefghijklmnopqrstuvwxyz', help='character label')

        parser_recog.add_argument('--character', type=str,
                            default='0123456789abcdefghijklmnopqrstuvwxyz가각간갇갈감갑값갓강갖같갚갛개객걀걔거걱건걷걸검겁것겉게겨격겪견결겹경곁계고곡곤곧골곰곱곳공과관광괜괴굉교구국군굳굴굵굶굽궁권귀귓규균귤그극근글긁금급긋긍기긴길김깅깊까깍깎깐깔깜깝깡깥깨꺼꺾껌껍껏껑께껴꼬꼭꼴꼼꼽꽂꽃꽉꽤꾸꾼꿀꿈뀌끄끈끊끌끓끔끗끝끼낌나낙낚난날낡남납낫낭낮낯낱낳내냄냇냉냐냥너넉넌널넓넘넣네넥넷녀녁년념녕노녹논놀놈농높놓놔뇌뇨누눈눕뉘뉴늄느늑는늘늙능늦늬니닐님다닥닦단닫달닭닮담답닷당닿대댁댐댓더덕던덜덟덤덥덧덩덮데델도독돈돌돕돗동돼되된두둑둘둠둡둥뒤뒷드득든듣들듬듭듯등디딩딪따딱딴딸땀땅때땜떠떡떤떨떻떼또똑뚜뚫뚱뛰뜨뜩뜯뜰뜻띄라락란람랍랑랗래랜램랫략량러럭런럴럼럽럿렁렇레렉렌려력련렬렵령례로록론롬롭롯료루룩룹룻뤄류륙률륭르른름릇릎리릭린림립릿링마막만많말맑맘맙맛망맞맡맣매맥맨맵맺머먹먼멀멈멋멍멎메멘멩며면멸명몇모목몬몰몸몹못몽묘무묵묶문묻물뭄뭇뭐뭘뭣므미민믿밀밉밌및밑바박밖반받발밝밟밤밥방밭배백뱀뱃뱉버번벌범법벗베벤벨벼벽변별볍병볕보복볶본볼봄봇봉뵈뵙부북분불붉붐붓붕붙뷰브븐블비빌빔빗빚빛빠빡빨빵빼뺏뺨뻐뻔뻗뼈뼉뽑뿌뿐쁘쁨사삭산살삶삼삿상새색샌생샤서석섞선설섬섭섯성세섹센셈셋셔션소속손솔솜솟송솥쇄쇠쇼수숙순숟술숨숫숭숲쉬쉰쉽슈스슨슬슴습슷승시식신싣실싫심십싯싱싶싸싹싼쌀쌍쌓써썩썰썹쎄쏘쏟쑤쓰쓴쓸씀씌씨씩씬씹씻아악안앉않알앓암압앗앙앞애액앨야약얀얄얇양얕얗얘어억언얹얻얼엄업없엇엉엊엌엎에엔엘여역연열엷염엽엿영옆예옛오옥온올옮옳옷옹와완왕왜왠외왼요욕용우욱운울움웃웅워원월웨웬위윗유육율으윽은을음응의이익인일읽잃임입잇있잊잎자작잔잖잘잠잡잣장잦재쟁쟤저적전절젊점접젓정젖제젠젯져조족존졸좀좁종좋좌죄주죽준줄줌줍중쥐즈즉즌즐즘증지직진질짐집짓징짙짚짜짝짧째쨌쩌쩍쩐쩔쩜쪽쫓쭈쭉찌찍찢차착찬찮찰참찻창찾채책챔챙처척천철첩첫청체쳐초촉촌촛총촬최추축춘출춤춥춧충취츠측츰층치칙친칠침칫칭카칸칼캄캐캠커컨컬컴컵컷케켓켜코콘콜콤콩쾌쿄쿠퀴크큰클큼키킬타탁탄탈탑탓탕태택탤터턱턴털텅테텍텔템토톤톨톱통퇴투툴툼퉁튀튜트특튼튿틀틈티틱팀팅파팎판팔팝패팩팬퍼퍽페펜펴편펼평폐포폭폰표푸푹풀품풍퓨프플픔피픽필핏핑하학한할함합항해핵핸햄햇행향허헌험헤헬혀현혈협형혜호혹혼홀홈홉홍화확환활황회획횟횡효후훈훌훔훨휘휴흉흐흑흔흘흙흡흥흩희흰히힘',
                            help='character label')

        parser_recog.add_argument('--sensitive', action='store_true', help='for sensitive character mode')
        parser_recog.add_argument('--PAD', action='store_true', help='whether to keep ratio then pad for image resize')
        """ Model Architecture """
        parser_recog.add_argument('--num_fiducial', type=int, default=20, help='number of fiducial points of TPS-STN')
        parser_recog.add_argument('--input_channel', type=int, default=1,
                            help='the number of input channel of Feature extractor')
        parser_recog.add_argument('--output_channel', type=int, default=512,
                            help='the number of output channel of Feature extractor')
        parser_recog.add_argument('--hidden_size', type=int, default=256, help='the size of the LSTM hidden state')

        args_recog= parser_recog.parse_args()

        return args_craft , args_recog



    def apply(self, image, timestamp, save_img=False):
        #coordinate : list
        save_log = False
        pred, timestamp = detect_ocr(self, image, timestamp, save_img, save_log)
        return pred, timestamp