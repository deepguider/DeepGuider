import os
import time
import string
import argparse
import re

import torch
import torch.backends.cudnn as cudnn
import torch.utils.data
import torch.nn.functional as F
import numpy as np
from nltk.metrics.distance import edit_distance

from utils import AttnLabelConverter, Averager
from dataset import RawDataset,hierarchical_dataset, AlignCollate
from model import Model


device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def benchmark_all_eval(model, criterion, converter, opt, calculate_infer_time=False):
    """ evaluation with 10 benchmark evaluation datasets """
    # The evaluation datasets, dataset order is same with Table 1 in our paper.
    # eval_data_list = ['IIIT5k_3000', 'SVT', 'IC03_860', 'IC03_867', 'IC13_857',
    #                   'IC13_1015', 'IC15_1811', 'IC15_2077', 'SVTP', 'CUTE80']
    eval_data_list = ['IC13_857', 'IC13_1015']#, 'val_kr']

    if calculate_infer_time:
        evaluation_batch_size = 1  # batch_size should be 1 to calculate the GPU inference time per image.
    else:
        evaluation_batch_size = opt.batch_size

    list_accuracy = []
    total_forward_time = 0
    total_evaluation_data_number = 0
    total_correct_number = 0
    log = open(f'./result/{opt.experiment_name}/log_all_evaluation.txt', 'a')
    dashed_line = '-' * 80
    print(dashed_line)
    log.write(dashed_line + '\n')
    for eval_data in eval_data_list:
        eval_data_path = os.path.join(opt.eval_data, eval_data)
        AlignCollate_evaluation = AlignCollate(imgH=opt.imgH, imgW=opt.imgW, keep_ratio_with_pad=opt.PAD)
        eval_data, eval_data_log = hierarchical_dataset(root=eval_data_path, opt=opt)
        evaluation_loader = torch.utils.data.DataLoader(
            eval_data, batch_size=evaluation_batch_size,
            shuffle=False,
            num_workers=int(opt.workers),
            collate_fn=AlignCollate_evaluation, pin_memory=True)

        _, accuracy_by_best_model, norm_ED_by_best_model, _, _, _, infer_time, length_of_data = validation(
            model, criterion, evaluation_loader, converter, opt)
        list_accuracy.append(f'{accuracy_by_best_model:0.3f}')
        total_forward_time += infer_time
        total_evaluation_data_number += len(eval_data)
        total_correct_number += accuracy_by_best_model * length_of_data
        log.write(eval_data_log)
        print(f'Acc {accuracy_by_best_model:0.3f}\t normalized_ED {norm_ED_by_best_model:0.3f}')
        log.write(f'Acc {accuracy_by_best_model:0.3f}\t normalized_ED {norm_ED_by_best_model:0.3f}\n')
        print(dashed_line)
        log.write(dashed_line + '\n')

    averaged_forward_time = total_forward_time / total_evaluation_data_number * 1000
    total_accuracy = total_correct_number / total_evaluation_data_number
    params_num = sum([np.prod(p.size()) for p in model.parameters()])

    evaluation_log = 'accuracy: '
    for name, accuracy in zip(eval_data_list, list_accuracy):
        evaluation_log += f'{name}: {accuracy}\t'
    evaluation_log += f'total_accuracy: {total_accuracy:0.3f}\t'
    evaluation_log += f'averaged_infer_time: {averaged_forward_time:0.3f}\t# parameters: {params_num/1e6:0.3f}'
    print(evaluation_log)
    log.write(evaluation_log + '\n')
    log.close()

    return None


def validation(model, criterion, evaluation_loader, converter, opt):
    """ validation or evaluation """
    n_correct = 0
    norm_ED = 0
    length_of_data = 0
    infer_time = 0
    valid_loss_avg = Averager()

    for i, (image_tensors, labels) in enumerate(evaluation_loader):
        batch_size = image_tensors.size(0)
        length_of_data = length_of_data + batch_size
        image = image_tensors.to(device)
        # For max length prediction
        length_for_pred = torch.IntTensor([opt.batch_max_length] * batch_size).to(device)
        text_for_pred = torch.LongTensor(batch_size, opt.batch_max_length + 1).fill_(0).to(device)

        text_for_loss, length_for_loss = converter.encode(labels, batch_max_length=opt.batch_max_length)

        start_time = time.time()

        preds = model(image, text_for_pred, is_train=False)
        forward_time = time.time() - start_time

        preds = preds[:, :text_for_loss.shape[1] - 1, :]
        target = text_for_loss[:, 1:]  # without [GO] Symbol
        cost = criterion(preds.contiguous().view(-1, preds.shape[-1]), target.contiguous().view(-1))

        # select max probabilty (greedy decoding) then decode index to character
        _, preds_index = preds.max(2)
        preds_str = converter.decode(preds_index, length_for_pred)
        labels = converter.decode(text_for_loss[:, 1:], length_for_loss)

        infer_time += forward_time
        valid_loss_avg.add(cost)

        # calculate accuracy & confidence score
        preds_prob = F.softmax(preds, dim=2)
        preds_max_prob, _ = preds_prob.max(dim=2)
        confidence_score_list = []
        for gt, pred, pred_max_prob in zip(labels, preds_str, preds_max_prob):

            gt = gt[:gt.find('[s]')]
            pred_EOS = pred.find('[s]')
            pred = pred[:pred_EOS]  # prune after "end of sentence" token ([s])
            pred_max_prob = pred_max_prob[:pred_EOS]

            # To evaluate 'case sensitive model' with alphanumeric and case insensitve setting.
            # if opt.sensitive and opt.data_filtering_off:
            #     pred = pred.lower()
            #     gt = gt.lower()
            #     alphanumeric_case_insensitve = '0123456789abcdefghijklmnopqrstuvwxyz'
            #     out_of_alphanumeric_case_insensitve = f'[^{alphanumeric_case_insensitve}]'
            #     pred = re.sub(out_of_alphanumeric_case_insensitve, '', pred)
            #     gt = re.sub(out_of_alphanumeric_case_insensitve, '', gt)

            if pred == gt:
                n_correct += 1

            '''
            (old version) ICDAR2017 DOST Normalized Edit Distance https://rrc.cvc.uab.es/?ch=7&com=tasks
            "For each word we calculate the normalized edit distance to the length of the ground truth transcription."
            if len(gt) == 0:
                norm_ED += 1
            else:
                norm_ED += edit_distance(pred, gt) / len(gt)
            '''

            # ICDAR2019 Normalized Edit Distance
            if len(gt) == 0 or len(pred) == 0:
                norm_ED += 0
            elif len(gt) > len(pred):
                norm_ED += 1 - edit_distance(pred, gt) / len(gt)
            else:
                norm_ED += 1 - edit_distance(pred, gt) / len(pred)

            # calculate confidence score (= multiply of pred_max_prob)
            try:
                confidence_score = pred_max_prob.cumprod(dim=0)[-1]
            except:
                confidence_score = 0  # for empty pred case, when prune after "end of sentence" token ([s])
            confidence_score_list.append(confidence_score)
            # print(pred, gt, pred==gt, confidence_score)

    accuracy = n_correct / float(length_of_data) * 100
    norm_ED = norm_ED / float(length_of_data)  # ICDAR2019 Normalized Edit Distance

    return valid_loss_avg.val(), accuracy, norm_ED, preds_str, confidence_score_list, labels, infer_time, length_of_data




def test(opt):
    """ model configuration """
    converter = AttnLabelConverter(opt.character)
    opt.num_class = len(converter.character)

    if opt.rgb:
        opt.input_channel = 3
    model = Model(opt,opt.num_class)
    print('model input parameters', opt.imgH, opt.imgW, opt.num_fiducial, opt.input_channel, opt.output_channel,
          opt.hidden_size, opt.num_class, opt.batch_max_length)

    model = torch.nn.DataParallel(model).to(device)

    # load model
    print('loading pretrained model from %s' % opt.saved_model)
    model.load_state_dict(torch.load(opt.saved_model, map_location=device))
    opt.experiment_name = '_'.join(opt.saved_model.split('/')[1:])
    # print(model)

    """ keep evaluation model and result logs """
    os.makedirs(f'./result/{opt.experiment_name}', exist_ok=True)
    os.system(f'cp {opt.saved_model} ./result/{opt.experiment_name}/')

    """ setup loss """
    criterion = torch.nn.CrossEntropyLoss(ignore_index=0).to(device)  # ignore [GO] token = ignore index 0

    """ evaluation """
    model.eval()
    with torch.no_grad():
        if opt.benchmark_all_eval:  # evaluation with 10 benchmark evaluation datasets
            benchmark_all_eval(model, criterion, converter, opt)
        else:
            log = open(f'./result/{opt.experiment_name}/log_evaluation.txt', 'a')
            AlignCollate_evaluation = AlignCollate(imgH=opt.imgH, imgW=opt.imgW, keep_ratio_with_pad=opt.PAD)
            eval_data, eval_data_log = hierarchical_dataset(root=opt.eval_data, opt=opt)
            evaluation_loader = torch.utils.data.DataLoader(
                eval_data, batch_size=opt.batch_size,
                shuffle=False,
                num_workers=int(opt.workers),
                collate_fn=AlignCollate_evaluation, pin_memory=True)
            _, accuracy_by_best_model, _, _, _, _, _, _ = validation(
                model, criterion, evaluation_loader, converter, opt)
            log.write(eval_data_log)
            print(f'{accuracy_by_best_model:0.3f}')
            log.write(f'{accuracy_by_best_model:0.3f}\n')
            log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--eval_data', required=True, help='path to evaluation dataset')
    parser.add_argument('--benchmark_all_eval', action='store_true', help='evaluate 10 benchmark evaluation datasets')

    parser.add_argument('--workers', type=int, help='number of data loading workers', default=4)
    parser.add_argument('--batch_size', type=int, default=192, help='input batch size')
    parser.add_argument('--saved_model', required=True, help="path to saved_model to evaluation")
    """ Data processing """
    parser.add_argument('--batch_max_length', type=int, default=25, help='maximum-label-length')
    parser.add_argument('--imgH', type=int, default=32, help='the height of the input image')
    parser.add_argument('--imgW', type=int, default=100, help='the width of the input image')
    parser.add_argument('--rgb', action='store_true', help='use rgb input')
    # parser.add_argument('--character', type=str, default='0123456789abcdefghijklmnopqrstuvwxyz', help='character label')
    parser.add_argument('--language', type=str, default="en")
    parser.add_argument('--character', type=str,
                        default='0123456789abcdefghijklmnopqrstuvwxyz', 
                        help='character label')


    parser.add_argument('--sensitive', action='store_true', help='for sensitive character mode')
    parser.add_argument('--PAD', action='store_true', help='whether to keep ratio then pad for image resize')
    parser.add_argument('--data_filtering_off', action='store_true', help='for data_filtering_off mode')
    """ Model Architecture """
    parser.add_argument('--num_fiducial', type=int, default=20, help='number of fiducial points of TPS-STN')
    parser.add_argument('--input_channel', type=int, default=1, help='the number of input channel of Feature extractor')
    parser.add_argument('--output_channel', type=int, default=512,
                        help='the number of output channel of Feature extractor')
    parser.add_argument('--hidden_size', type=int, default=256, help='the size of the LSTM hidden state')

    opt = parser.parse_args()

    if opt.language == 'kr':
        opt.character = '0123456789abcdefghijklmnopqrstuvwxyz가각간갇갈감갑값갓갔강갖같갚갛개객갤갬갭갯갱갸걀걍걔거걱건걷걸검겁것겅겉게겐겔겟겠겨격겪견결겸겹겼경곁계고곡곤곧골곰곱곳공곶과곽관괄괌광괘괜괭괴굉교구국군굳굴굵굶굼굽굿궁궈권궐궤귀귄귈귐규균귤그극근글긁금급긋긍기긱긴길김깁깃깅깆깊까깍깎깐깔깜깝깟깡깥깨깬깻깽꺠꺵꺼꺽꺾껌껍껏껑께껴껼꼬꼭꼰꼴꼼꼽꼿꽁꽂꽃꽈꽉꽐꽝꽥꾀꾸꾹꾼꿀꿇꿈꿉꿍꿔꿨꿩꿰뀌뀐뀝끄끈끊끌끓끔끗끙끝끼끽낀낄낌나낙낚난날낡낢남납낫났낭낮낯낱낳내낵낸낼냄냅냇냉냐냑냠냥너넉넌널넑넓넘넙넛넝넣네넥넨넬넴넵넷녀녁년념녕녘녜노녹논놀놈놉놋농높놓놔뇌뇨뇸누눅눈눌눔눕눗눠뉘뉴뉼늄느늑는늘늙능늦늪늬니닉닌닐님닙닛닝다닥닦단닫달닭닮닳담답닷당닿대댁댄댈댐댑댓댕더덕덖던덜덟덤덥덧덩덫덮데덱덴델뎀뎃뎅뎌뎐도독돈돋돌돔돕돗동돼되된될됨됩두둑둔둘둠둣둥둬뒤뒷뒹듀듈듕드득든듣들듦듬듭듯등듸딍디딕딘딜딝딤딥딧딨딩딪따딱딴딸땀땄땅땋때땐땜땠땡떄떙떠떡떤떨떴떵떻떼떽뗄뗌뗏뗴또똑똘똣똥뚜뚝뚤뚫뚱뛰뛸뜀뜨뜩뜬뜯뜰뜸뜻띄띠띤띵라락란랄람랍랏랐랑랗래랙랜램랩랫랬랭랲랴략량러럭런럴럼럽럿렀렁렇레렉렌렐렘렙렛렝렢려력련렬렴렵렷렸령례로록론롤롬롭롯롱뢰료룡루룩룬룰룸룹룻룽뤂뤄뤼륀류륙륜률륨륫륭르륵른를름릅릇릉릎릐릠리릭린릳릴림립릿링마막만많맏말맑맘맙맛맜망맞맡매맥맨맬맴맵맷맹맺먀먕머먹먼멀멈멋멍메멕멘멜멤멥멧멩며멱면멸명몇모목몫몬몯몰몸못몽뫼묘무묵묶문묻물뭄뭇뭉뭍뭐뭔뭘뮈뮌뮐뮤뮨뮬므믄믈믐미믹민믿밀밈밋밌밍밎및밑바박밖반받발밝밟밤밥밧방밭배백밴밸뱀뱃뱅뱉버벅번벌범법벗벙벚베벡벤벧벨벳벵벼벽변별볍병볕보복볶본볼봄봅봇봉봐봤뵈뵙뵤부북분붇불붉붐붑붓붕붙뷔뷰브븍븐블븟븨비빅빈빌빔빕빗빙빚빛빠빡빤빨빱빵빻빼빽뺀뺄뺌뺏뺑뺴뺵뻐뻑뻔뻘뻤뻥뻬뼁뼈뼘뼛뽀뽁뽈뽐뽑뽕뾰뿅뿌뿍뿐뿔뿜뿡쁘쁜쁠쁨삐삔삘삠사삭산살삶삼삽삿샀상새색샌샐샘샛생샤샥샨샬샴샵샷샹샾섀섄서석섞선설섬섭섯섰성세섹센셀셈셉셋셍셔션셜셧셨셩셰셸소속손솓솔솜솝솟송솥쇄쇠쇳쇼숀숄숌숍숏숖수숙순숟술숨숫숭숮숯숲숴쉐쉘쉬쉰쉴쉼쉽쉿슈슐슘슛스슨슬슴습슷승슽슾슿싀시식신실싫심십싯싰싱싶싸싹싼쌀쌈쌉쌋쌌쌍쌓쌔쌘쌤쌩썅썜썡써썩썬썰썸썹썼썽쎄쎈쎌쎙쎴쏘쏙쏠쏩쏭쏴쑈쑝쑤쑥쑬쒰쓔쓰쓱쓴쓸씀씅씌씨씩씬씰씷씹씻씽아악안앉않알앓암압앗았앙앞애액앤앨앰앱앳앵야약얀얄얇얉얌얍양얕얗얘어억언얹얻얼엄업없엇었엉엌엎에엑엔엘엠엡엣엥여역엮연열염엽엿였영옅옆예옌옐옘옙옛오옥온올옮옳옴옵옷옹옻와왁완왈왑왓왔왕왜외왼요욕욘욜욤용우욱운울움웃웅웆워웍원월웜웠웡웨웩웬웰웹위윅윈윌윗윙유육윤율윰융윷으윽은을읊음읍응읗의이익인일읽잃임입잇있잉잊잌잍잎자작잔잖잘잠잡잣장잦재잭잰잴잼잿쟁쟈쟉쟌쟝저적전절젊젋점접젓정젖제젝젠젤젬젯젱져젼졌조족졲존졸좀좁종좆좋좌죄죈죌죠죤주죽준줃줄줌줍줏중줘줬쥐쥘쥬쥰쥴즈즉즌즐즘즙증지직진짇질짐집짓징짖짙짚짜짝짠짤짦짧짬짭짱째쨈쨌쨍쨰쩌쩍쩐쩜쩝쩡쩨쩰쪄쪼쪽쫀쫄쫌쫑쫒쫓쬘쭈쭉쭌쮸쯔쯤찌찍찐찔찜찝찡찢찧차착찬찮찰참찹찻창찾챂채책챈챌챔챕챙챠챤처척천철첨첩첫청체첵첸첼쳅쳇쳐쳤초촉촌촐촘촛총촬최쵸쵹추축춘출춤춧충춰췌취츄츠측츰층치칙친칠칡침칩칫칭칲카칵칸칼캄캅캇캉캐캔캘캠캡캣캥캬커컥컨컬컴컵컷컸컹케켄켈켐켓켜코콕콘콜콞콤콥콧콩콰콴콸쾅쾌쾰쿄쿠쿡쿤쿨쿰쿱쿳쿵쿼퀀퀄퀘퀴퀵퀸퀼큐큔큘큠크큰클큼큽킁키킥킨킬킴킵킷킹타탁탄탈탉탐탑탓탕태택탠탤탬탭탱터턱턴털텀텁텃텅테텍텐텔템텝텡텨톈토톡톤톨톰톱톳통톹톺퇘퇴퇼투툭툰툴퉁튀튜튤트특튼틀틈틔티틱틴틸팀팁팃팅파팍판팔팜팝팟팠팡팥패팩팬팰팸팻팽퍙퍼펀펄펌펍펑펖페펙펜펠펨펩펫펭펴편펼평폐포폭폰폴폼퐁푀표푸푹푼풀품풋풍퓌퓨퓰퓸프픈플픔픙피픽핀필핌핏핑하학한핟할핥함합핫항핳해핵핸핼햄햅햇했행햐향허헉헌헐험헛헝헤헨헬헴헵헷헸헹혀혁현혈혐협혓혔형혜호혹혼홀홈홉홋홍화확환활홧황횃회획횐횟횡효횽후훅훈훌훔훗훠훤훨훼휀휘휙휠휴휼흄흉흐흑흔흘흙흠흡흥흩희흰히힉힌힐힘힙힛힝'
    else:
        opt.character = '0123456789abcdefghijklmnopqrstuvwxyz'

    """ vocab / character number configuration """
    if opt.sensitive:
        opt.character = string.printable[:-6]  # same with ASTER setting (use 94 char).

    cudnn.benchmark = True
    cudnn.deterministic = True
    opt.num_gpu = torch.cuda.device_count()

    test(opt)
