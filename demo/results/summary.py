#!/usr/bin/env python3

import sys
import os
import yaml
import numpy as np
import pylab as pl
import matplotlib

matplotlib.rcParams.update({'font.size': 16})
matplotlib.rc('font', family='sans-serif')
matplotlib.rc('text', usetex = True)

robot = sys.argv[1] if (len(sys.argv) > 1 and sys.argv[1] != '-e') else 'bicycle'
fontsize = 16


class Result:

    def __init__(self, src):

        self.src = src
        self.zoh = 'zoh' in os.path.basename(src)
        vals = os.path.basename(src).replace('zoh','')[:-5].split('_')
        self.bf = '' if vals[0].isdigit() else 'f'
        if self.bf:
            self.pop = int(vals[0][:-1])
        else:
            self.pop = int(vals[0])

        self.sub = 1

        if vals[1].isdigit():
            self.sub = int(vals[1])

        self.type = '_'.join(v for v in vals[1:] if not v.isdigit())
        self.data = None

    def matches(self, pop, zoh, bf):
        if self.pop != pop or self.bf != bf:
            return False
        if self.sub == 1:
            return True
        return self.zoh == zoh

    def mean(self):
        if self.data is None:
            self.read()
        return np.mean(self.data)

    def read(self):

        if self.data is not None:
            return
        with open(self.src) as data:

            self.data = [elem[1] for elem in yaml.safe_load(data)['data']]

            # self.data = np.array(yaml.safe_load(data)['data'])[:,1]
            self.data = np.array([v for v in self.data if not isinstance(v, str)])


    def __lt__(self, other):
        return self.sub < other.sub


legend = {'ctime': 'Comp. time',
            'rollout': 'Rollouts',
            'cost': 'Prediction cost',
            'error': 'Actual error',}

N = len(legend)

pl.close('all')


def add_create(d, key, val = {}):
    if key in d:
        d[key] = val
    else:
        d = {key: val}

yticks = pl.linspace(0,1,11)
yticklabels = [str(y)[:3] for y in yticks]


for exp in os.listdir(robot):

    root = robot + '/' + exp

    if not os.path.isdir(root) or not any(exp.startswith(s+'_') for s in ('single', 'multi')):
        continue

    method = exp.split('_')[0]
    results = [Result(f'{root}/{f}') for f in os.listdir(root) if f.endswith('yaml')]

    def get(pop, zoh, sub, t, bf):
        for r in results:
            if not r.matches(pop, zoh, bf):
                continue
            if r.sub == sub and r.type == t:
                return r.mean()
        return None

    def bars(x, data, labels, xlabel, ylabel, title, name=None, normalize=True):
        pl.figure(figsize=(8,5))
        # print(data.shape)
        N = data.shape[1]
        width = 0.9*1./N
        offset = -(N-1)*width/2
        pl.plot([min(x)-N/2*width, max(x)+N/2*width], [1,1],'k--', label=None)
        ymax = 1.
        for i in range(N):
            values = data[:,i]
            scale = 1
            if isinstance(normalize, bool) and normalize:
                scale = values[0]
            elif isinstance(normalize, (list, tuple)):
                print(normalize, i, data.shape)
                scale = data[0, normalize[i]]
            pl.bar(x+offset, values/scale, width=width, label=legend[labels[i]])
            if normalize:
                ymax = max(ymax, np.amax(values/scale))
            offset += width

        if normalize:
            pl.ylim(0,1.3*ymax)
        pl.xlabel(xlabel)
        pl.ylabel(ylabel)
        pl.grid(axis='y')
        # pl.yticks(yticks, yticklabels)
        pl.title(title)
        pl.legend(ncols = N//2, loc = 'upper center')
        pl.tight_layout()
        if name is not None:
            pl.savefig(name+'.pdf')


    pops = set(r.pop for r in results)
    print('pops:',pops)


    for zoh in (False, True):

        for pop in pops:

            for bf in ('f',''):

                # zoh = False

                # bars
                this = sorted([r for r in results if r.matches(pop, zoh, bf)])
                if not this:
                    continue
                subs = np.array(list(set(t.sub for t in this)))

                labels = list(legend.keys())

                data = np.array([[get(pop, zoh, sub, l, bf) for sub in subs] for l in labels]).T

                print(pop, bf, zoh)

                bars(subs, data, labels,
                    ('ZOH' if zoh else 'Linear') + ' subsampling (1 = no subsampling)',
                    'Comparison (1 for subsampling = 1)',
                    f'MPPI with {pop + 100*pop//2} runs' if bf else f'Population = {pop}',
                    f'{root}/{"zoh" if zoh else "lin"}_{pop}{bf}')

                # fields = [(t,bf) for t in ('ctime','cost') for bf in ('','f')]
                # data = np.array([[get(pop, zoh, sub, t, bf) for sub in subs] for t, bf in fields]).T
                # labels = [legend[t] + (' (f)' if bf else '') for t, bf in fields]
                # bars(subs, data, labels,
                #     ('ZOH' if zoh else 'Linear') + ' subsampling (1 = no subsampling)',
                #     'Mean value',
                #     f'Population = {pop}',
                #     f'{root}/{"zoh" if zoh else "lin"}_{pop}_comp',
                #     normalize=[0, 0, 2,2])



        pl.ion()
        pl.show()
