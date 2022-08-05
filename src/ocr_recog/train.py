import os
import sys
import time
import random
import string
import argparse

import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
import torch.nn.init as init
import torch.optim as optim
import torch.utils.data
import numpy as np

from utils import AttnLabelConverter, Averager
from dataset import hierarchical_dataset, AlignCollate, Batch_Balanced_Dataset
from model import Model
from test import validation


import getpass
from tensorboardX import SummaryWriter
""" Transformation : TPS, FeatureExtraction : VGG , SequenceModeling : BiLSTM , Prediction : Attn """

save_dir = 'checkpoints'

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def train(opt):

    if opt.use_tb:
        tb_dir = f'./log/{opt.experiment_name}'
        print('tensorboard : ',tb_dir)
        if not os.path.exists(tb_dir):
            os.makedirs(tb_dir)
        writer = SummaryWriter(log_dir=tb_dir)

    """ dataset preparation """
    if not opt.data_filtering_off:
        print('Filtering the images containing characters which are not in opt.character')
        print('Filtering the images whose label is longer than opt.batch_max_length')
        # see https://github.com/clovaai/deep-text-recognition-benchmark/blob/6593928855fb7abb999a99f428b3e4477d4ae356/dataset.py#L130

    opt.select_data = opt.select_data.split('-')
    opt.batch_ratio = opt.batch_ratio.split('-')
    train_dataset = Batch_Balanced_Dataset(opt)

    # log = open(f'./saved_models/{opt.experiment_name}/log_dataset.txt', 'a')
    log = open(f'{save_dir}/{opt.experiment_name}/log_dataset.txt', 'a')
    AlignCollate_valid = AlignCollate(imgH=opt.imgH, imgW=opt.imgW, keep_ratio_with_pad=opt.PAD)
    valid_dataset, valid_dataset_log = hierarchical_dataset(root=opt.valid_data, opt=opt)
    valid_loader = torch.utils.data.DataLoader(
        valid_dataset, batch_size=opt.batch_size,
        shuffle=True,  # 'True' to check training progress with validation function.
        num_workers=int(opt.workers),
        collate_fn=AlignCollate_valid, pin_memory=True)
    log.write(valid_dataset_log)
    print('-' * 80)
    log.write('-' * 80 + '\n')
    log.close()

    """ model configuration """
    converter = AttnLabelConverter(opt.character)
    opt.num_class = len(converter.character)

    if opt.rgb:
        opt.input_channel = 3

    # sekim for transfer learning
    model = Model(opt, 38)

    print('model input parameters', opt.imgH, opt.imgW, opt.num_fiducial, opt.input_channel, opt.output_channel,
          opt.hidden_size, opt.num_class, opt.batch_max_length)

    # weight initialization
    for name, param in model.named_parameters():
        if 'localization_fc2' in name:
            print(f'Skip {name} as it is already initialized')
            continue
        try:
            if 'bias' in name:
                init.constant_(param, 0.0)
            elif 'weight' in name:
                init.kaiming_normal_(param)
        except Exception as e:  # for batchnorm.
            if 'weight' in name:
                param.data.fill_(1)
            continue

    # data parallel for multi-GPU
    model = torch.nn.DataParallel(model).to(device)
    
    if opt.saved_model != '':
        print(f'loading pretrained model from {opt.saved_model}')
        if opt.FT:
            model.load_state_dict(torch.load(opt.saved_model), strict=False)
        else:
            model.load_state_dict(torch.load(opt.saved_model))
            
    # sekim change last layer
    in_feature = model.module.Prediction.generator.in_features
    model.module.Prediction.attention_cell.rnn = nn.LSTMCell(256 + opt.num_class, 256).to(device)
    model.module.Prediction.generator = nn.Linear(in_feature, opt.num_class).to(device)

    print(model.module.Prediction.generator)
    print("Model:")
    print(model)

    model.train()

    """ setup loss """
    criterion = torch.nn.CrossEntropyLoss(ignore_index=0).to(device)  # ignore [GO] token = ignore index 0
    # loss averager
    loss_avg = Averager()

    # filter that only require gradient decent
    filtered_parameters = []
    params_num = []
    for p in filter(lambda p: p.requires_grad, model.parameters()):
        filtered_parameters.append(p)
        params_num.append(np.prod(p.size()))
    print('Trainable params num : ', sum(params_num))
    # [print(name, p.numel()) for name, p in filter(lambda p: p[1].requires_grad, model.named_parameters())]

    # setup optimizer
    if opt.adam:
        optimizer = optim.Adam(filtered_parameters, lr=opt.lr, betas=(opt.beta1, 0.999))
    else:
        optimizer = optim.Adadelta(filtered_parameters, lr=opt.lr, rho=opt.rho, eps=opt.eps)
    print("Optimizer:")
    print(optimizer)

    """ final options """

    # with open(f'./saved_models/{opt.experiment_name}/opt.txt', 'a') as opt_file:
    with open(f'{save_dir}/{opt.experiment_name}/opt.txt', 'a') as opt_file:
        opt_log = '------------ Options -------------\n'
        args = vars(opt)
        for k, v in args.items():
            opt_log += f'{str(k)}: {str(v)}\n'
        opt_log += '---------------------------------------\n'
        print(opt_log)
        opt_file.write(opt_log)

    """ start training """
    start_iter = 0

    if opt.saved_model != '':
        try:
            start_iter = int(opt.saved_model.split('_')[-1].split('.')[0])
            print("-------------------------------------------------")
            print(f'continue to train, start_iter: {start_iter}')
        except:
            pass

    start_time = time.time()
    best_accuracy = -1
    best_norm_ED = -1
    i = start_iter

    while(True):
        # train part
        image_tensors, labels = train_dataset.get_batch()
        image = image_tensors.to(device)
        text, length = converter.encode(labels, batch_max_length=opt.batch_max_length)
        batch_size = image.size(0)

        preds = model(image, text[:, :-1])  # align with Attention.forward
        target = text[:, 1:]  # without [GO] Symbol
        cost = criterion(preds.view(-1, preds.shape[-1]), target.contiguous().view(-1))

        model.zero_grad()
        cost.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), opt.grad_clip)  # gradient clipping with 5 (Default)
        optimizer.step()

        loss_avg.add(cost)

        # validation part
        if i % opt.valInterval == 0:
            elapsed_time = time.time() - start_time
            # for log
            with open(f'{save_dir}/{opt.experiment_name}/log_train.txt', 'a') as log:
            # with open(f'./saved_models/{opt.experiment_name}/log_train.txt', 'a') as log:
                model.eval()
                with torch.no_grad():
                    valid_loss, current_accuracy, current_norm_ED, preds, confidence_score, labels, infer_time, length_of_data = validation(
                        model, criterion, valid_loader, converter, opt)

                model.train()

                # training loss and validation loss

                loss_log = f'[{i}/{opt.num_iter}] Train loss: {loss_avg.val():0.5f}, Valid loss: {valid_loss:0.5f}, Elapsed_time: {elapsed_time:0.5f}'
                loss_avg.reset()
                if opt.use_tb:
                    writer.add_scalar('OCR_loss/train_loss', loss_avg.val(), i)
                    writer.add_scalar('OCR_loss/validation_loss', valid_loss, i)

                current_model_log = f'{"Current_accuracy":17s}: {current_accuracy:0.3f}, {"Current_norm_ED":17s}: {current_norm_ED:0.2f}'

                # keep best accuracy model (on valid dataset)
                if current_accuracy > best_accuracy:
                    best_accuracy = current_accuracy
                    # torch.save(model.state_dict(), f'./saved_models/{opt.experiment_name}/best_accuracy.pth')
                    torch.save(model.state_dict(), f'{save_dir}/{opt.experiment_name}/best_accuracy.pth')
                if current_norm_ED > best_norm_ED:
                    best_norm_ED = current_norm_ED
                    # torch.save(model.state_dict(), f'./saved_models/{opt.experiment_name}/best_norm_ED.pth')
                    torch.save(model.state_dict(), f'{save_dir}/{opt.experiment_name}/best_norm_ED.pth')
                if i % 4000 == 0 and i != 0:
                    torch.save(model.state_dict(), f'{save_dir}/{opt.experiment_name}/epoch_{i}.pth')
                best_model_log = f'{"Best_accuracy":17s}: {best_accuracy:0.3f}, {"Best_norm_ED":17s}: {best_norm_ED:0.2f}'

                loss_model_log = f'{loss_log}\n{current_model_log}\n{best_model_log}'
                print(loss_model_log)
                log.write(loss_model_log + '\n')

                # show some predicted results
                dashed_line = '-' * 80
                head = f'{"Ground Truth":25s} | {"Prediction":25s} | Confidence Score & T/F'
                predicted_result_log = f'{dashed_line}\n{head}\n{dashed_line}\n'
                for gt, pred, confidence in zip(labels[:5], preds[:5], confidence_score[:5]):

                    gt = gt[:gt.find('[s]')]
                    pred = pred[:pred.find('[s]')]

                    predicted_result_log += f'{gt:25s} | {pred:25s} | {confidence:0.4f}\t{str(pred == gt)}\n'
                predicted_result_log += f'{dashed_line}'
                print(predicted_result_log)
                log.write(predicted_result_log + '\n')

        # save model per 1e+5 iter.
        if (i + 1) % 1e+5 == 0:
            # torch.save(model.state_dict(), f'./saved_models/{opt.experiment_name}/iter_{i+1}.pth')
            torch.save(model.state_dict(), f'{save_dir}/{opt.experiment_name}/iter_{i + 1}.pth')

        if i == opt.num_iter:
            print('end the training')
            sys.exit()
        i += 1


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment_name', required=True, help='Where to store logs and models')
    parser.add_argument('--train_data', required=True, help='path to training dataset')
    parser.add_argument('--valid_data', required=True, help='path to validation dataset')
    parser.add_argument('--manualSeed', type=int, default=1111, help='for random seed setting')
    parser.add_argument('--workers', type=int, help='number of data loading workers', default=4)
    parser.add_argument('--batch_size', type=int, default=192, help='input batch size')
    parser.add_argument('--num_iter', type=int, default=500000, help='number of iterations to train for')
    parser.add_argument('--valInterval', type=int, default=2000, help='Interval between each validation')
    parser.add_argument('--saved_model', default='', help="path to model to continue training")
    parser.add_argument('--FT', action='store_true', help='whether to do fine-tuning')
    parser.add_argument('--adam', action='store_true', help='Whether to use adam (default is Adadelta)')
    parser.add_argument('--lr', type=float, default=1, help='learning rate, default=1.0 for Adadelta')
    parser.add_argument('--beta1', type=float, default=0.9, help='beta1 for adam. default=0.9')
    parser.add_argument('--rho', type=float, default=0.95, help='decay rate rho for Adadelta. default=0.95')
    parser.add_argument('--eps', type=float, default=1e-8, help='eps for Adadelta. default=1e-8')
    parser.add_argument('--grad_clip', type=float, default=5, help='gradient clipping value. default=5')
    
    """ Data processing """
    parser.add_argument('--select_data', type=str, default='MJ-ST',
                        help='select training data (default is MJ-ST, which means MJ and ST used as training data)')
    parser.add_argument('--batch_ratio', type=str, default='0.5-0.5',
                        help='assign ratio for each selected data in the batch')
    parser.add_argument('--total_data_usage_ratio', type=str, default='1.0',
                        help='total data usage ratio, this ratio is multiplied to total number of data.')
    parser.add_argument('--batch_max_length', type=int, default=25, help='maximum-label-length')
    parser.add_argument('--imgH', type=int, default=32, help='the height of the input image')
    parser.add_argument('--imgW', type=int, default=100, help='the width of the input image')
    parser.add_argument('--rgb', action='store_true', help='use rgb input')
    parser.add_argument('--language', type=str, default="en")
    parser.add_argument('--character', type=str,
                        default='0123456789abcdefghijklmnopqrstuvwxyz', 
                        help='character label')
    parser.add_argument('--sensitive', action='store_true', help='for sensitive character mode')
    parser.add_argument('--PAD', action='store_true', help='whether to keep ratio then pad for image resize')
    parser.add_argument('--data_filtering_off', action='store_true', help='for data_filtering_off mode')
    
    """ Model Architecture """
    parser.add_argument('--num_fiducial', type=int, default=20, help='number of fiducial points of TPS-STN')
    parser.add_argument('--input_channel', type=int, default=1,
                        help='the number of input channel of Feature extractor')
    parser.add_argument('--output_channel', type=int, default=512,
                        help='the number of output channel of Feature extractor')
    parser.add_argument('--hidden_size', type=int, default=256, help='the size of the LSTM hidden state')
    parser.add_argument('--use_tb', action='store_true')


    opt = parser.parse_args()
    
    if opt.language == 'kr':
        opt.character = '0123456789abcdefghijklmnopqrstuvwxyz가각간갇갈감갑값갓갔강갖같갚갛개객갤갬갭갯갱갸걀걍걔거걱건걷걸검겁것겅겉게겐겔겟겠겨격겪견결겸겹겼경곁계고곡곤곧골곰곱곳공곶과곽관괄괌광괘괜괭괴굉교구국군굳굴굵굶굼굽굿궁궈권궐궤귀귄귈귐규균귤그극근글긁금급긋긍기긱긴길김깁깃깅깆깊까깍깎깐깔깜깝깟깡깥깨깬깻깽꺠꺵꺼꺽꺾껌껍껏껑께껴껼꼬꼭꼰꼴꼼꼽꼿꽁꽂꽃꽈꽉꽐꽝꽥꾀꾸꾹꾼꿀꿇꿈꿉꿍꿔꿨꿩꿰뀌뀐뀝끄끈끊끌끓끔끗끙끝끼끽낀낄낌나낙낚난날낡낢남납낫났낭낮낯낱낳내낵낸낼냄냅냇냉냐냑냠냥너넉넌널넑넓넘넙넛넝넣네넥넨넬넴넵넷녀녁년념녕녘녜노녹논놀놈놉놋농높놓놔뇌뇨뇸누눅눈눌눔눕눗눠뉘뉴뉼늄느늑는늘늙능늦늪늬니닉닌닐님닙닛닝다닥닦단닫달닭닮닳담답닷당닿대댁댄댈댐댑댓댕더덕덖던덜덟덤덥덧덩덫덮데덱덴델뎀뎃뎅뎌뎐도독돈돋돌돔돕돗동돼되된될됨됩두둑둔둘둠둣둥둬뒤뒷뒹듀듈듕드득든듣들듦듬듭듯등듸딍디딕딘딜딝딤딥딧딨딩딪따딱딴딸땀땄땅땋때땐땜땠땡떄떙떠떡떤떨떴떵떻떼떽뗄뗌뗏뗴또똑똘똣똥뚜뚝뚤뚫뚱뛰뛸뜀뜨뜩뜬뜯뜰뜸뜻띄띠띤띵라락란랄람랍랏랐랑랗래랙랜램랩랫랬랭랲랴략량러럭런럴럼럽럿렀렁렇레렉렌렐렘렙렛렝렢려력련렬렴렵렷렸령례로록론롤롬롭롯롱뢰료룡루룩룬룰룸룹룻룽뤂뤄뤼륀류륙륜률륨륫륭르륵른를름릅릇릉릎릐릠리릭린릳릴림립릿링마막만많맏말맑맘맙맛맜망맞맡매맥맨맬맴맵맷맹맺먀먕머먹먼멀멈멋멍메멕멘멜멤멥멧멩며멱면멸명몇모목몫몬몯몰몸못몽뫼묘무묵묶문묻물뭄뭇뭉뭍뭐뭔뭘뮈뮌뮐뮤뮨뮬므믄믈믐미믹민믿밀밈밋밌밍밎및밑바박밖반받발밝밟밤밥밧방밭배백밴밸뱀뱃뱅뱉버벅번벌범법벗벙벚베벡벤벧벨벳벵벼벽변별볍병볕보복볶본볼봄봅봇봉봐봤뵈뵙뵤부북분붇불붉붐붑붓붕붙뷔뷰브븍븐블븟븨비빅빈빌빔빕빗빙빚빛빠빡빤빨빱빵빻빼빽뺀뺄뺌뺏뺑뺴뺵뻐뻑뻔뻘뻤뻥뻬뼁뼈뼘뼛뽀뽁뽈뽐뽑뽕뾰뿅뿌뿍뿐뿔뿜뿡쁘쁜쁠쁨삐삔삘삠사삭산살삶삼삽삿샀상새색샌샐샘샛생샤샥샨샬샴샵샷샹샾섀섄서석섞선설섬섭섯섰성세섹센셀셈셉셋셍셔션셜셧셨셩셰셸소속손솓솔솜솝솟송솥쇄쇠쇳쇼숀숄숌숍숏숖수숙순숟술숨숫숭숮숯숲숴쉐쉘쉬쉰쉴쉼쉽쉿슈슐슘슛스슨슬슴습슷승슽슾슿싀시식신실싫심십싯싰싱싶싸싹싼쌀쌈쌉쌋쌌쌍쌓쌔쌘쌤쌩썅썜썡써썩썬썰썸썹썼썽쎄쎈쎌쎙쎴쏘쏙쏠쏩쏭쏴쑈쑝쑤쑥쑬쒰쓔쓰쓱쓴쓸씀씅씌씨씩씬씰씷씹씻씽아악안앉않알앓암압앗았앙앞애액앤앨앰앱앳앵야약얀얄얇얉얌얍양얕얗얘어억언얹얻얼엄업없엇었엉엌엎에엑엔엘엠엡엣엥여역엮연열염엽엿였영옅옆예옌옐옘옙옛오옥온올옮옳옴옵옷옹옻와왁완왈왑왓왔왕왜외왼요욕욘욜욤용우욱운울움웃웅웆워웍원월웜웠웡웨웩웬웰웹위윅윈윌윗윙유육윤율윰융윷으윽은을읊음읍응읗의이익인일읽잃임입잇있잉잊잌잍잎자작잔잖잘잠잡잣장잦재잭잰잴잼잿쟁쟈쟉쟌쟝저적전절젊젋점접젓정젖제젝젠젤젬젯젱져젼졌조족졲존졸좀좁종좆좋좌죄죈죌죠죤주죽준줃줄줌줍줏중줘줬쥐쥘쥬쥰쥴즈즉즌즐즘즙증지직진짇질짐집짓징짖짙짚짜짝짠짤짦짧짬짭짱째쨈쨌쨍쨰쩌쩍쩐쩜쩝쩡쩨쩰쪄쪼쪽쫀쫄쫌쫑쫒쫓쬘쭈쭉쭌쮸쯔쯤찌찍찐찔찜찝찡찢찧차착찬찮찰참찹찻창찾챂채책챈챌챔챕챙챠챤처척천철첨첩첫청체첵첸첼쳅쳇쳐쳤초촉촌촐촘촛총촬최쵸쵹추축춘출춤춧충춰췌취츄츠측츰층치칙친칠칡침칩칫칭칲카칵칸칼캄캅캇캉캐캔캘캠캡캣캥캬커컥컨컬컴컵컷컸컹케켄켈켐켓켜코콕콘콜콞콤콥콧콩콰콴콸쾅쾌쾰쿄쿠쿡쿤쿨쿰쿱쿳쿵쿼퀀퀄퀘퀴퀵퀸퀼큐큔큘큠크큰클큼큽킁키킥킨킬킴킵킷킹타탁탄탈탉탐탑탓탕태택탠탤탬탭탱터턱턴털텀텁텃텅테텍텐텔템텝텡텨톈토톡톤톨톰톱톳통톹톺퇘퇴퇼투툭툰툴퉁튀튜튤트특튼틀틈틔티틱틴틸팀팁팃팅파팍판팔팜팝팟팠팡팥패팩팬팰팸팻팽퍙퍼펀펄펌펍펑펖페펙펜펠펨펩펫펭펴편펼평폐포폭폰폴폼퐁푀표푸푹푼풀품풋풍퓌퓨퓰퓸프픈플픔픙피픽핀필핌핏핑하학한핟할핥함합핫항핳해핵핸핼햄햅햇했행햐향허헉헌헐험헛헝헤헨헬헴헵헷헸헹혀혁현혈혐협혓혔형혜호혹혼홀홈홉홋홍화확환활홧황횃회획횐횟횡효횽후훅훈훌훔훗훠훤훨훼휀휘휙휠휴휼흄흉흐흑흔흘흙흠흡흥흩희흰히힉힌힐힘힙힛힝'
    else:
        opt.character = '0123456789abcdefghijklmnopqrstuvwxyz'

    # print(opt)
    # print(opt.saved_model)
    # print(opt.experiment_name)
    # opt.saved_model = "aaa"
    # print(opt.saved_model)
    # input("abc")

    # os.makedirs(f'./saved_models/{opt.experiment_name}', exist_ok=True)
    os.makedirs(f'{save_dir}/{opt.experiment_name}', exist_ok=True)

    """ vocab / character number configuration """
    if opt.sensitive:
        # opt.character += 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        opt.character = string.printable[:-6]  # same with ASTER setting (use 94 char).

    """ Seed and GPU setting """
    # print("Random Seed: ", opt.manualSeed)
    random.seed(opt.manualSeed)
    np.random.seed(opt.manualSeed)
    torch.manual_seed(opt.manualSeed)
    torch.cuda.manual_seed(opt.manualSeed)

    cudnn.benchmark = True
    cudnn.deterministic = True
    opt.num_gpu = torch.cuda.device_count()
    # print('device count', opt.num_gpu)
    if opt.num_gpu > 1:
        print('------ Use multi-GPU setting ------')
        print('if you stuck too long time with multi-GPU setting, try to set --workers 0')
        # check multi-GPU issue https://github.com/clovaai/deep-text-recognition-benchmark/issues/1
        opt.workers = opt.workers * opt.num_gpu
        opt.batch_size = opt.batch_size * opt.num_gpu

        """ previous version
        print('To equlize batch stats to 1-GPU setting, the batch_size is multiplied with num_gpu and multiplied batch_size is ', opt.batch_size)
        opt.batch_size = opt.batch_size * opt.num_gpu
        print('To equalize the number of epochs to 1-GPU setting, num_iter is divided with num_gpu by default.')
        If you dont care about it, just commnet out these line.)
        opt.num_iter = int(opt.num_iter / opt.num_gpu)
        """

    train(opt)

