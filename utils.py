import re
import sys


def cityblock_distance(pos1, pos2):
    '''
    :param pos1: Is a string of x,y coordinates
    :param pos2: Is a string of x,y coordinates
    :return: the cityblock distance (manhatten distance)
    '''
    pos1 = tuple(int(x) for x in re.findall(r'\d+', pos1))
    pos2 = tuple(int(x) for x in re.findall(r'\d+', pos2))
    return abs(pos2[0] - pos1[0])+abs(pos2[1] - pos1[1])

# TODO:
def _remove_element(list_elements, position):
    if len(list_elements[position]) > 1:
        list_elements[position].pop(0)
    else:
        list_elements.pop(position)


def _get_agt_loc(state, agent_char):
    for key, value in state.agents.items():
        if int(value[0][1]) == int(agent_char):
            return key
            return False
        else:
            return key
            print(f'{agent_char},{value[0][1]}', file=sys.stderr, flush=True)
    print(state.agents, file=sys.stderr, flush=True)
    raise Exception('Agent not found')


def _get_box_loc(state, box_id):
    for key, value in state.boxes.items():
        if box_id == value[0][2]:
            return key
    raise Exception('Box not found')