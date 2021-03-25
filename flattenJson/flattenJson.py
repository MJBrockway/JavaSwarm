#import numpy as np
import json
import sys

def unpack_jdata(path='swarm.json'):
    with open(path, 'r') as f:
        state = json.load(f)
        f.close()
    args = {k:v for k,v in state['params'].items()}
    dsts = state['destinations']['coords']
    try:
        args['goalX'] = dsts[0][0]
        args['goalY'] = dsts[1][0]
    except:
        print("Empty dest parameter")
    
    xs = state['agents']['coords'][0]
    ys = state['agents']['coords'][1]
    return args, xs, ys
    
def saveFlatData(args, xs, ys, path='swarm.txt'):
    with open(path, 'w') as f:
        for item in args.items():
            f.write(f"{item[0]} {item[1]}\n".lower())
        f.write("# POS_X, POS_Y --\n")
        for x,y in zip(xs, ys):
            f.write(f"{x} {y}\n")
        f.close()


#####################################################

path = sys.argv[1]
pt = path.find('.')
if pt < 0:
    stub = path
else:
    stub = path[:pt]
    
args, xs, ys = unpack_jdata(path)
saveFlatData(args, xs, ys, stub+'.txt')

