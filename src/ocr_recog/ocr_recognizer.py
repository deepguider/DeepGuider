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

        self.saved_model = './checkpoints/ocr_res_kr/best_accuracy.pth'
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
        self.model = torch.nn.DataParallel(self.model).to(self.device)
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
        parser_craft.add_argument('--result_folder', default='./uvc2/', type=str, help='result folder path')
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
        
        parser_recog.add_argument('--language', type=str, default="en")
        parser_recog.add_argument('--character', type=str,
                            default='0123456789abcdefghijklmnopqrstuvwxyz',
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
        
        if args_recog.language == 'kr':
            args_recog.character = '0123456789abcdefghijklmnopqrstuvwxyz가각간갇갈감갑값갓갔강갖같갚갛개객갤갬갭갯갱갸걀걍걔거걱건걷걸검겁것겅겉게겐겔겟겠겨격겪견결겸겹겼경곁계고곡곤곧골곰곱곳공곶과곽관괄괌광괘괜괭괴굉교구국군굳굴굵굶굼굽굿궁궈권궐궤귀귄귈귐규균귤그극근글긁금급긋긍기긱긴길김깁깃깅깆깊까깍깎깐깔깜깝깟깡깥깨깬깻깽꺠꺵꺼꺽꺾껌껍껏껑께껴껼꼬꼭꼰꼴꼼꼽꼿꽁꽂꽃꽈꽉꽐꽝꽥꾀꾸꾹꾼꿀꿇꿈꿉꿍꿔꿨꿩꿰뀌뀐뀝끄끈끊끌끓끔끗끙끝끼끽낀낄낌나낙낚난날낡낢남납낫났낭낮낯낱낳내낵낸낼냄냅냇냉냐냑냠냥너넉넌널넑넓넘넙넛넝넣네넥넨넬넴넵넷녀녁년념녕녘녜노녹논놀놈놉놋농높놓놔뇌뇨뇸누눅눈눌눔눕눗눠뉘뉴뉼늄느늑는늘늙능늦늪늬니닉닌닐님닙닛닝다닥닦단닫달닭닮닳담답닷당닿대댁댄댈댐댑댓댕더덕덖던덜덟덤덥덧덩덫덮데덱덴델뎀뎃뎅뎌뎐도독돈돋돌돔돕돗동돼되된될됨됩두둑둔둘둠둣둥둬뒤뒷뒹듀듈듕드득든듣들듦듬듭듯등듸딍디딕딘딜딝딤딥딧딨딩딪따딱딴딸땀땄땅땋때땐땜땠땡떄떙떠떡떤떨떴떵떻떼떽뗄뗌뗏뗴또똑똘똣똥뚜뚝뚤뚫뚱뛰뛸뜀뜨뜩뜬뜯뜰뜸뜻띄띠띤띵라락란랄람랍랏랐랑랗래랙랜램랩랫랬랭랲랴략량러럭런럴럼럽럿렀렁렇레렉렌렐렘렙렛렝렢려력련렬렴렵렷렸령례로록론롤롬롭롯롱뢰료룡루룩룬룰룸룹룻룽뤂뤄뤼륀류륙륜률륨륫륭르륵른를름릅릇릉릎릐릠리릭린릳릴림립릿링마막만많맏말맑맘맙맛맜망맞맡매맥맨맬맴맵맷맹맺먀먕머먹먼멀멈멋멍메멕멘멜멤멥멧멩며멱면멸명몇모목몫몬몯몰몸못몽뫼묘무묵묶문묻물뭄뭇뭉뭍뭐뭔뭘뮈뮌뮐뮤뮨뮬므믄믈믐미믹민믿밀밈밋밌밍밎및밑바박밖반받발밝밟밤밥밧방밭배백밴밸뱀뱃뱅뱉버벅번벌범법벗벙벚베벡벤벧벨벳벵벼벽변별볍병볕보복볶본볼봄봅봇봉봐봤뵈뵙뵤부북분붇불붉붐붑붓붕붙뷔뷰브븍븐블븟븨비빅빈빌빔빕빗빙빚빛빠빡빤빨빱빵빻빼빽뺀뺄뺌뺏뺑뺴뺵뻐뻑뻔뻘뻤뻥뻬뼁뼈뼘뼛뽀뽁뽈뽐뽑뽕뾰뿅뿌뿍뿐뿔뿜뿡쁘쁜쁠쁨삐삔삘삠사삭산살삶삼삽삿샀상새색샌샐샘샛생샤샥샨샬샴샵샷샹샾섀섄서석섞선설섬섭섯섰성세섹센셀셈셉셋셍셔션셜셧셨셩셰셸소속손솓솔솜솝솟송솥쇄쇠쇳쇼숀숄숌숍숏숖수숙순숟술숨숫숭숮숯숲숴쉐쉘쉬쉰쉴쉼쉽쉿슈슐슘슛스슨슬슴습슷승슽슾슿싀시식신실싫심십싯싰싱싶싸싹싼쌀쌈쌉쌋쌌쌍쌓쌔쌘쌤쌩썅썜썡써썩썬썰썸썹썼썽쎄쎈쎌쎙쎴쏘쏙쏠쏩쏭쏴쑈쑝쑤쑥쑬쒰쓔쓰쓱쓴쓸씀씅씌씨씩씬씰씷씹씻씽아악안앉않알앓암압앗았앙앞애액앤앨앰앱앳앵야약얀얄얇얉얌얍양얕얗얘어억언얹얻얼엄업없엇었엉엌엎에엑엔엘엠엡엣엥여역엮연열염엽엿였영옅옆예옌옐옘옙옛오옥온올옮옳옴옵옷옹옻와왁완왈왑왓왔왕왜외왼요욕욘욜욤용우욱운울움웃웅웆워웍원월웜웠웡웨웩웬웰웹위윅윈윌윗윙유육윤율윰융윷으윽은을읊음읍응읗의이익인일읽잃임입잇있잉잊잌잍잎자작잔잖잘잠잡잣장잦재잭잰잴잼잿쟁쟈쟉쟌쟝저적전절젊젋점접젓정젖제젝젠젤젬젯젱져젼졌조족졲존졸좀좁종좆좋좌죄죈죌죠죤주죽준줃줄줌줍줏중줘줬쥐쥘쥬쥰쥴즈즉즌즐즘즙증지직진짇질짐집짓징짖짙짚짜짝짠짤짦짧짬짭짱째쨈쨌쨍쨰쩌쩍쩐쩜쩝쩡쩨쩰쪄쪼쪽쫀쫄쫌쫑쫒쫓쬘쭈쭉쭌쮸쯔쯤찌찍찐찔찜찝찡찢찧차착찬찮찰참찹찻창찾챂채책챈챌챔챕챙챠챤처척천철첨첩첫청체첵첸첼쳅쳇쳐쳤초촉촌촐촘촛총촬최쵸쵹추축춘출춤춧충춰췌취츄츠측츰층치칙친칠칡침칩칫칭칲카칵칸칼캄캅캇캉캐캔캘캠캡캣캥캬커컥컨컬컴컵컷컸컹케켄켈켐켓켜코콕콘콜콞콤콥콧콩콰콴콸쾅쾌쾰쿄쿠쿡쿤쿨쿰쿱쿳쿵쿼퀀퀄퀘퀴퀵퀸퀼큐큔큘큠크큰클큼큽킁키킥킨킬킴킵킷킹타탁탄탈탉탐탑탓탕태택탠탤탬탭탱터턱턴털텀텁텃텅테텍텐텔템텝텡텨톈토톡톤톨톰톱톳통톹톺퇘퇴퇼투툭툰툴퉁튀튜튤트특튼틀틈틔티틱틴틸팀팁팃팅파팍판팔팜팝팟팠팡팥패팩팬팰팸팻팽퍙퍼펀펄펌펍펑펖페펙펜펠펨펩펫펭펴편펼평폐포폭폰폴폼퐁푀표푸푹푼풀품풋풍퓌퓨퓰퓸프픈플픔픙피픽핀필핌핏핑하학한핟할핥함합핫항핳해핵핸핼햄햅햇했행햐향허헉헌헐험헛헝헤헨헬헴헵헷헸헹혀혁현혈혐협혓혔형혜호혹혼홀홈홉홋홍화확환활홧황횃회획횐횟횡효횽후훅훈훌훔훗훠훤훨훼휀휘휙휠휴휼흄흉흐흑흔흘흙흠흡흥흩희흰히힉힌힐힘힙힛힝'
        else:
            args_recog.character = '0123456789abcdefghijklmnopqrstuvwxyz'

        return args_craft , args_recog



    def apply(self, image, timestamp, save_img=False):
        #coordinate : list
        save_log = False
        pred, timestamp = detect_ocr(self, image, timestamp, save_img, save_log)
        return pred, timestamp