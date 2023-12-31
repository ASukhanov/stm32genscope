"""Waveform generation for MCUFEC"""

__version__ = 'v0.0.1 2023-12-19'
import sys
import numpy as np

top = 4095

if __name__ == "__main__":
    import argparse
    global pargs, plot, TextItem
    parser = argparse.ArgumentParser(description=__doc__
    ,formatter_class=argparse.ArgumentDefaultsHelpFormatter
    ,epilog=f'{sys.argv[0]}: {__version__}')
    parser.add_argument('-n', '--points', default=100,  help='Number of points')
    parser.add_argument('wave', default='saw', choices=['saw','sine','triangle'], help=\
      'Waveform')
    pargs = parser.parse_args()
    print(f'pargs: {pargs}')

    if pargs.wave == 'saw':
        ar = np.linspace(0,top,pargs.points-1)
        s = [pargs.points] + list(ar.astype(int)) + [0]
        txt = f'{s}'
        print('{' + txt[1:-1] + '}')
    elif pargs.wave == 'triangle':
        n = (pargs.points-1)//2 + 1
        ar1 = np.linspace(0, top, n)
        ar2 = np.linspace(ar1[-1], 0, n)
        s = [pargs.points] + list(ar1.astype(int)) + list(ar2.astype(int))
        #print(f'points generated: {len(s)}')
        txt = f'{s}'
        print('{' + txt[1:-1] + '}')
