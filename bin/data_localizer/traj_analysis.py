import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import glob
import pickle

def draw_graph(title, datax, datay, label=None, lwidth=2, lcolor=None, lalpha=1):
    plt.figure(title)
    line, = plt.plot(datax, datay, label=label, linewidth=lwidth, alpha=lalpha)
    if lcolor != None:
        line.set_color(lcolor)

def config_graph(title, xlabel=None, ylabel=None, xrange=None, yrange=None, xy_equal=False, xy_tick=(1, 10), show_title=False, show_legend=True, fsize=14):
    plt.figure(title)
    plt.grid(which='both')
    if xlabel != None:
        plt.xlabel(xlabel, fontsize=fsize)
    if ylabel != None:
        plt.ylabel(ylabel, fontsize=fsize)
    if xrange != None:
        plt.xlim(xrange)
    if yrange != None:
        plt.ylim(yrange)
    plt.xticks(None, fontsize=fsize)
    plt.yticks(None, fontsize=fsize)
    if xy_equal:
        plt.gca().set_aspect('equal')
        plt.gca().xaxis.set_minor_locator(ticker.MultipleLocator(xy_tick[0]))
        plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(xy_tick[1]))
        plt.gca().yaxis.set_minor_locator(ticker.MultipleLocator(xy_tick[0]))
        plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(xy_tick[1]))
        plt.gca().grid(which='minor', alpha=0.5)
        plt.gca().grid(which='major', alpha=1)
    if show_title:
        plt.title(title, fontsize=fsize)
    if show_legend:
        plt.legend(loc='best', fontsize=fsize, framealpha=0.2)

def trim_radian(radian):
    trim = radian - (radian // (2 * np.pi)) * (2 * np.pi)
    mask = trim >= np.pi
    trim[mask] = trim[mask] - 2 * np.pi
    mask = trim < -np.pi
    trim[mask] = trim[mask] + 2 * np.pi
    return trim

def calc_traj_error(true, traj):
    delta = true - traj
    e_p = np.sqrt(np.multiply(delta[:,1], delta[:,1]) + np.multiply(delta[:,2], delta[:,2]))
    e_o = np.abs(trim_radian(delta[:,3]))
    e_v = np.abs(delta[:,4])
    e_w = np.abs(delta[:,5])
    return e_p, e_o, e_v, e_w

def load_csv_file(fname):
    return np.loadtxt(fname, delimiter=',', comments='#')

def draw_traj(fnames, xy_tick=(1, 10), show_more=True):
    title = str(fnames);
    if (type(fnames) is not list) and (type(fnames) is not tuple):
        fnames = [ fnames ]
    for fname in fnames:
        traj = load_csv_file(fname)
        draw_graph(title, traj[:,1], traj[:,2], fname)
        if show_more:
            draw_graph(title + ': Orientation', traj[:,0], traj[:,3] * 180 / np.pi, fname)
            draw_graph(title + ': Linear Velocity', traj[:,0], traj[:,4], fname)
            draw_graph(title + ': Angular Velocity', traj[:,0], traj[:,5] * 180 / np.pi, fname)
    config_graph(title, 'X [m]', 'Y [m]', xy_equal=True, xy_tick=xy_tick)
    if show_more:
        config_graph(title + ': Orientation', 'Time [sec]', 'Orientation [deg]')
        config_graph(title + ': Linear Velocity', 'Time [sec]', 'Linear Velocity [m/s]')
        config_graph(title + ': Angular Velocity', 'Time [sec]', 'Angular Velocity [deg/s]')

def summarize_traj_error(ftruth, fnames, summarize=np.median):
    summary = []
    if ftruth == None or fnames == None or len(fnames) == 0:
        return summary
    if (type(fnames) is not list) and (type(fnames) is not tuple):
        fnames = [ fnames ]

    true = load_csv_file(ftruth)
    for fname in fnames:
        files = glob.glob(fname)
        if len(files) > 0:
            e_p = []
            e_o = []
            e_v = []
            e_w = []
            for f in files:
                traj = load_csv_file(f)
                e_ps, e_os, e_vs, e_ws = calc_traj_error(true, traj)
                e_p.append(summarize(e_ps))
                e_o.append(summarize(e_os))
                e_v.append(summarize(e_vs))
                e_w.append(summarize(e_ws))
            summary.append([summarize(e_p), summarize(e_o), summarize(e_v), summarize(e_w)])
    return summary

def report_traj_error(ftruth, fnames, summarize=np.median, header=True):
    report = ''
    if header:
        report += '| {:>8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('e_p', 'e_o', 'e_v', 'e_w', 'Name')
        report += '| {:8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('-' * 8, '-' * 8, '-' * 8, '-' * 8, '-' * 8)
    if ftruth == None or fnames == None or len(fnames) == 0:
        return report
    if (type(fnames) is not list) and (type(fnames) is not tuple):
        fnames = [ fnames ]

    summary = summarize_traj_error(ftruth, fnames, summarize)
    for idx, err in enumerate(summary):
        score = '| {:>8.3f} | {:>8.1f} | {:>8.3f} | {:>8.1f} |'.format(
                err[0], err[1] * 180 / np.pi, err[2], err[3] * 180 / np.pi)
        report += score + ' ' + fnames[idx] + ' |\n'
    return report

def pickle_traj_error(path, summarize=np.median, path_truth='truth', file_filter='*.csv', verbose=True):
    files = glob.glob(path + '/' + file_filter)
    if len(files) > 0:
        error_record = {}
        for idx, traj_file in enumerate(files):
            sep = traj_file.find('/')
            if sep < 0:
                sep = traj_file.find('\\')
            if sep < 0:
                break
            fname = traj_file[sep + 1:]

            sep = fname.find(',')
            if sep < 0:
                break
            sep = fname.find(',', sep + 1)
            if sep < 0:
                break
            true_file = path_truth + '/' + fname[:sep] + ').pose.csv'

            true = load_csv_file(true_file)
            traj = load_csv_file(traj_file)
            e_ps, e_os, e_vs, e_ws = calc_traj_error(true, traj)
            error_record[traj_file] = [ summarize(e_ps), summarize(e_os), summarize(e_vs), summarize(e_ws) ]

            if verbose and (idx % (len(files) / 100) == 0):
                print('Progress %d %%' % (idx * 100 / len(files)))

        f = open(path + '.pickle', 'wb')
        pickle.dump(error_record, f)
        f.close()

def summarize_traj_error_from_dict(error_record, fnames, summarize=np.median):
    summary = []
    if error_record == None or fnames == None or len(fnames) == 0:
        return summary
    if (type(fnames) is not list) and (type(fnames) is not tuple):
        fnames = [ fnames ]

    for fname in fnames:
        files = glob.glob(fname)
        if len(files) > 0:
            e_p = []
            e_o = []
            e_v = []
            e_w = []
            for f in files:
                err = error_record[f]
                if len(err) < 4:
                    raise
                e_p.append(err[0])
                e_o.append(err[1])
                e_v.append(err[2])
                e_w.append(err[3])
            summary.append([summarize(e_p), summarize(e_o), summarize(e_v), summarize(e_w)])
    return summary

def report_traj_error_from_dict(error_record, fnames, summarize=np.median, header=True):
    report = ''
    if header:
        report += '| {:>8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('e_p', 'e_o', 'e_v', 'e_w', 'Name')
        report += '| {:8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('-' * 8, '-' * 8, '-' * 8, '-' * 8, '-' * 8)
    if error_record == None or fnames == None or len(fnames) == 0:
        return report
    if (type(fnames) is not list) and (type(fnames) is not tuple):
        fnames = [ fnames ]

    summary = summarize_traj_error_from_dict(error_record, fnames, summarize)
    for idx, err in enumerate(summary):
        score = '| {:>8.3f} | {:>8.1f} | {:>8.3f} | {:>8.1f} |'.format(
                err[0], err[1] * 180 / np.pi, err[2], err[3] * 180 / np.pi)
        report += score + ' ' + fnames[idx] + ' |\n'
    return report
