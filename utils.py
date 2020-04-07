import re


def cityblock_distance(pos1, pos2):
    '''
    :param pos1: Is a string of x,y coordinates
    :param pos2: Is a string of x,y coordinates
    :return: the cityblock distance (manhatten distance)
    '''
    pos1 = tuple(int(x) for x in re.findall(r'\d+', pos1))
    pos2 = tuple(int(x) for x in re.findall(r'\d+', pos2))
    return abs(pos2[0] - pos1[0])+abs(pos2[1] - pos1[1])

