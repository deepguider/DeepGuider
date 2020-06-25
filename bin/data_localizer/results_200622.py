from traj_analysis import report_traj_error_from_dict
from traj_analysis import summarize_traj_error_from_dict
import pickle
import numpy as np
import matplotlib.pyplot as plt

results_path = 'results_200622'
summary = np.median
fsize = 14
lwidth = 2
save_file = ''
save_path = 'report_200622'

def gen_file_list(fformat, vals, methods=['CV', 'ZG', 'HT'], path=results_path + '/'):
    flist = []
    for m in methods:
        for v in vals:
            fname = fformat % (v, m)
            flist.append(path + fname)
    return flist


print('\n## ' + results_path)
f = open(results_path + '.pickle', 'rb')
err_rec = pickle.load(f)
f.close()


print('\n### All Errors w.r.t. { Method, GPS Offset }')
for gps_freq in ['10Hz', '01Hz']:
    print('\n### Localization Accuracy with ' + gps_freq + ' Hz GPS Data')
    for traj in ['Line', 'Circle', 'Sine', 'Square']:
        print('#### ' + traj)
        print(report_traj_error_from_dict(err_rec, None), end='')
        for method in [',0.10).GPS', ',0.10).CV', ',0.10).HT', ',0.50).ZG']:
            for gps_offset in ['0m', '1m']:
                print(report_traj_error_from_dict(err_rec, results_path + '/' + traj + '(' + gps_freq + ',00s,1)(0.5,' + gps_offset + method + '.*.csv', summarize=summary, header=False), end='')


print('\n### Position Error w.r.t. { Method, GPS Offset }')
for traj in ['Line', 'Circle', 'Sine', 'Square']:
    for gps_freq in ['10Hz', '01Hz']:
        err = summarize_traj_error_from_dict(err_rec, [
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,0m,0.10).GPS.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,0m,0.10).CV.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,0m,0.10).HT.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,0m,0.50).ZG.*.csv',

            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,1m,0.10).GPS.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,1m,0.10).CV.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,1m,0.10).HT.*.csv',
            'results_200622/' + traj + '(' + gps_freq + ',00s,1)(0.5,1m,0.50).ZG.*.csv'], summarize=summary)
        X = np.arange(2)
        fig = plt.figure()
        ax = fig.add_axes([0, 0, 1, 1])
        ax.bar(X + 0.00, [err[1][0], err[5][0]], color='b', label='CV', width=0.2)
        ax.bar(X + 0.25, [err[2][0], err[6][0]], color='g', label='HT', width=0.2)
        ax.bar(X + 0.50, [err[3][0], err[7][0]], color='r', label='ZG', width=0.2)
        ax.set_xticks(X + 0.25)
        ax.yaxis.set_tick_params(labelsize=fsize)
        ax.set_xticklabels(['Centered GPS', 'Off-centered GPS'], fontsize=fsize)
        ax.set_ylabel('Position Error $e_p$ [m]', fontsize=fsize)
        ax.set_ylim([0, 0.9])
        plt.grid(axis='y', alpha=0.5)
        plt.legend(loc='best', fontsize=fsize, framealpha=0.2)
        ax.plot([0 - 0.125 - 0.005, 1 + 0.25 * 2 + 0.125 + 0.005], [err[0][0], err[0][0]], 'k--', label="GPS", linewidth=lwidth)
        plt.text(1 + 0.25 * 2, err[0][0] + 0.02, "GPS", fontsize=fsize)
        plt.title(traj + ' (' + gps_freq + ')', fontsize=fsize)
        if len(save_file) > 0:
            fig.savefig(save_path + '/' + traj + '(' + gps_freq + ').' + save_file, bbox_inches='tight')
        else:
            plt.show()
