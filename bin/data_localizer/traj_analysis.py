import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from glob import glob

def draw_graph(title, datax, datay, name=None, lwidth=2, lcolor=None, lalpha=1):
    plt.figure(title)
    line, = plt.plot(datax, datay, label=name, linewidth=lwidth, alpha=lalpha)
    if lcolor != None:
        line.set_color(lcolor)

def conf_graph(title, xlabel=None, ylabel=None, xrange=None, yrange=None, xy_equal=False, xy_equal_tick=(1, 10), show_title=False, show_legend=True, fsize=14):
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
    plt.gca().xaxis.set_tick_params(labelsize=fsize)
    plt.gca().yaxis.set_tick_params(labelsize=fsize)
    if xy_equal:
        plt.gca().set_aspect('equal')
        plt.gca().xaxis.set_minor_locator(ticker.MultipleLocator(xy_equal_tick[0]))
        plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(xy_equal_tick[1]))
        plt.gca().yaxis.set_minor_locator(ticker.MultipleLocator(xy_equal_tick[0]))
        plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(xy_equal_tick[1]))
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

def draw_traj(fnames, show_more=True):
    if (type(fnames) is list) or (type(fnames) is tuple):
        title = str(fnames);
        for fname in fnames:
            traj = load_csv_file(fname)
            draw_graph(title, traj[:,1], traj[:,2], fname)
            if show_more:
                draw_graph(title + ': Orientation', traj[:,0], traj[:,3] * 180 / np.pi, fname)
                draw_graph(title + ': Linear Velocity', traj[:,0], traj[:,4], fname)
                draw_graph(title + ': Angular Velocity', traj[:,0], traj[:,5] * 180 / np.pi, fname)
        conf_graph(title, 'X [m]', 'Y [m]', xy_equal=True)
        if show_more:
            conf_graph(title + ': Orientation', 'Time [sec]', 'Orientation [deg]')
            conf_graph(title + ': Linear Velocity', 'Time [sec]', 'Linear Velocity [m/s]')
            conf_graph(title + ': Angular Velocity', 'Time [sec]', 'Angular Velocity [deg/s]')
    else:
        traj = load_csv_file(fnames)
        draw_graph(fnames, traj[:,1], traj[:,2])
        conf_graph(fnames, 'X [m]', 'Y [m]', xy_equal=True)

def report_traj_error(fnames, summarize=np.median, header=True):
    report = ''
    if header:
        report += '| {:>8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('e_p', 'e_o', 'e_v', 'e_w', 'Name')
        report += '| {:8} | {:>8} | {:>8} | {:>8} | {:8} |\n'.format('-' * 8, '-' * 8, '-' * 8, '-' * 8, '-' * 8)
    if ((type(fnames) is list) or (type(fnames) is tuple)) and len(fnames) > 0:
        true = load_csv_file(fnames[0])
        for fname in fnames[1:]:
            files = glob(fname)
            if len(files) > 0:
                es_p = []
                es_o = []
                es_v = []
                es_w = []
                for file in files:
                    traj = load_csv_file(file)
                    e_ps, e_os, e_vs, e_ws = calc_traj_error(true, traj)
                    es_p.append(summarize(e_ps))
                    es_o.append(summarize(e_os))
                    es_v.append(summarize(e_vs))
                    es_w.append(summarize(e_ws))
                e_p = summarize(es_p)
                e_o = summarize(es_o)
                e_v = summarize(es_v)
                e_w = summarize(es_w)
                score = '| {:>8.3f} | {:>8.1f} | {:>8.3f} | {:>8.1f} |'.format(e_p, e_o * 180 / np.pi, e_v, e_w * 180 / np.pi)
                report += score + ' ' + fname + ' |\n'
    return report

