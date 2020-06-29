from traj_analysis import summarize_traj_error_from_dict
import pickle
import numpy as np
import matplotlib.pyplot as plt

results_path = 'synthetic_results_200623'
traj = 'Sine'
sigmas = np.arange(0.1, 1.1, 0.1)
freqs = np.arange(1, 11)
summary = np.median
fsize = 14
lwidth = 2
save_file = ''
save_path = 'synthetic_report_200623_' + traj

def gen_file_list(fformat, vals, methods=['CV', 'ZG', 'HT'], path=results_path + '/'):
    flist = []
    for m in methods:
        for v in vals:
            fname = fformat % (v, m)
            flist.append(path + fname)
    return flist


print('\n## ' + traj)
f = open(results_path + '.pickle', 'rb')
err_rec = pickle.load(f)
f.close()


print('\n### Position Error w.r.t. GPS Noise')
for f in freqs:
    freq = '%02dHz' % f
    err_gps = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,0m,0.10).%s.*.csv', sigmas, ['GPS']), summarize=summary))
    err_cv  = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.10).%s.*.csv', sigmas, ['CV']), summarize=summary))
    err_ht  = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.10).%s.*.csv', sigmas, ['HT']), summarize=summary))
    err_zg  = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.50).%s.*.csv', sigmas, ['ZG']), summarize=summary))

    fig = plt.figure()
    plt.plot(sigmas, err_gps[:,0], 'k--', label='GPS', linewidth=lwidth)
    plt.plot(sigmas, err_cv[:,0],  'b-',  label='CV',  linewidth=lwidth)
    plt.plot(sigmas, err_ht[:,0],  'g-',  label='HT',  linewidth=lwidth)
    plt.plot(sigmas, err_zg[:,0],  'r-',  label='ZG',  linewidth=lwidth)
    plt.grid(True, alpha=0.5)
    plt.xlim((sigmas[0], sigmas[-1]))
    plt.ylim((0, 1.4))
    plt.xticks(sigmas, fontsize=fsize)
    plt.yticks(None, fontsize=fsize)
    plt.xlabel('GPS Noise $\sigma_G$ [m]', fontsize=fsize)
    plt.ylabel('Position Error $e_p$ [m]', fontsize=fsize)
    plt.legend(loc='best', fontsize=fsize, framealpha=0.2)
    plt.title('GPS Frequency: ' + freq, fontsize=fsize)
    if len(save_file) > 0:
        fig.savefig(save_path + '/e_p-sigma(freq' + freq + ').' + save_file, bbox_inches='tight')
    else:
        plt.show()


print('\n### Position Error w.r.t. GPS Frequency')
for sig in sigmas:
    sigma = '%.1f' % sig
    err_gps = []
    err_cv  = []
    err_ht  = []
    err_zg  = []
    for f in freqs:
        freq = '%02dHz' % f
        e_gps = summarize_traj_error_from_dict(err_rec, results_path + '/' + traj + '(' + freq + ',00s,1)(' + sigma + ',0m,0.10).GPS.*.csv', summarize=summary)
        e_cv  = summarize_traj_error_from_dict(err_rec, results_path + '/' + traj + '(' + freq + ',00s,1)(' + sigma + ',1m,0.10).CV.*.csv', summarize=summary)
        e_ht  = summarize_traj_error_from_dict(err_rec, results_path + '/' + traj + '(' + freq + ',00s,1)(' + sigma + ',1m,0.10).HT.*.csv', summarize=summary)
        e_zg  = summarize_traj_error_from_dict(err_rec, results_path + '/' + traj + '(' + freq + ',00s,1)(' + sigma + ',1m,0.50).ZG.*.csv', summarize=summary)
        err_gps.append(e_gps[0])
        err_cv.append(e_cv[0])
        err_ht.append(e_ht[0])
        err_zg.append(e_zg[0])
    err_gps = np.array(err_gps)
    err_cv  = np.array(err_cv)
    err_ht  = np.array(err_ht)
    err_zg  = np.array(err_zg)

    fig = plt.figure()
    plt.plot(freqs, err_gps[:,0], 'k--', label='GPS', linewidth=lwidth)
    plt.plot(freqs, err_cv[:,0],  'b-',  label='CV',  linewidth=lwidth)
    plt.plot(freqs, err_ht[:,0],  'g-',  label='HT',  linewidth=lwidth)
    plt.plot(freqs, err_zg[:,0],  'r-',  label='ZG',  linewidth=lwidth)
    plt.grid(True, alpha=0.5)
    plt.xlim((freqs[0], freqs[-1]))
    plt.ylim((0, 1.4))
    plt.xticks(freqs, fontsize=fsize)
    plt.yticks(None, fontsize=fsize)
    plt.xlabel('GPS Frequency $f_G$ [Hz]', fontsize=fsize)
    plt.ylabel('Position Error $e_p$ [m]', fontsize=fsize)
    plt.legend(loc='best', fontsize=fsize, framealpha=0.2)
    plt.title('GPS Noise: ' + sigma, fontsize=fsize)
    if len(save_file) > 0:
        fig.savefig(save_path + '/e_p-freq(sigma' + sigma + ').' + save_file, bbox_inches='tight')
    else:
        plt.show()


print('\n### Position Error w.r.t. GPS Noise and Frequency')
box_size = (sigmas[0] - 0.05, sigmas[-1] + 0.05, freqs[0] - 0.5, freqs[-1] + 0.5)
box_ratio = 0.1
err_name = ['Position Error $e_p$ [m]', 'Orientation Error $e_o$ [deg]']
err_max = [1.2, 45]
dff_max = [0.2, 10]
err_prefix = ['e_p-', 'e_o-']

ep_cv = []
ep_ht = []
ep_zg = []
eo_cv = []
eo_ht = []
eo_zg = []
for f in np.flip(freqs):
    freq = '%02dHz' % f
    all_cv = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.10).%s.*.csv', sigmas, ['CV']), summarize=summary))
    all_ht = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.10).%s.*.csv', sigmas, ['HT']), summarize=summary))
    all_zg = np.array(summarize_traj_error_from_dict(err_rec, gen_file_list(traj + '(' + freq + ',00s,1)(%.1f,1m,0.50).%s.*.csv', sigmas, ['ZG']), summarize=summary))

    ep_cv.append(all_cv[:,0].tolist())
    ep_ht.append(all_ht[:,0].tolist())
    ep_zg.append(all_zg[:,0].tolist())
    eo_cv.append(all_cv[:,1].tolist())
    eo_ht.append(all_ht[:,1].tolist())
    eo_zg.append(all_zg[:,1].tolist())

err_cv = [np.array(ep_cv), np.array(eo_cv) * 180 / np.pi]
err_ht = [np.array(ep_ht), np.array(eo_ht) * 180 / np.pi]
err_zg = [np.array(ep_zg), np.array(eo_zg) * 180 / np.pi]

err_label  = ['CV', 'HT', 'ZG', 'HT-CV', 'ZG-CV', 'HT-ZG']
for e in range(len(err_name)):
    err_data  = [err_cv[e], err_ht[e], err_zg[e], err_ht[e] - err_cv[e], err_zg[e] - err_cv[e], err_ht[e] - err_zg[e]]
    err_range = [(0, err_max[e]), (0, err_max[e]), (0, err_max[e]), (-dff_max[e], dff_max[e]), (-dff_max[e], dff_max[e]), (-dff_max[e], dff_max[e])]
    for idx, data in enumerate(err_data):
        fig = plt.figure()
        plt.imshow(data, cmap='RdGy', extent=box_size, aspect=box_ratio, vmin=err_range[idx][0], vmax=err_range[idx][1])
        plt.xticks(sigmas[::2], fontsize=fsize)
        plt.yticks(freqs[::2], fontsize=fsize)
        plt.xlabel('GPS Noise $\sigma_G$ [m]', fontsize=fsize)
        plt.ylabel('GPS Frequency $f_G$ [Hz]', fontsize=fsize)
        cb = plt.colorbar()
        cb.ax.tick_params(labelsize=fsize)
        plt.title(err_label[idx], fontsize=fsize)
        if len(save_file) > 0:
            fig.savefig(save_path + '/' + err_prefix[e] + 'freq-sigma(' + err_label[idx] + ').' + save_file, bbox_inches='tight')
        else:
            plt.show()
