from strategy import StrategyBFS
from state import State
from collections import defaultdict
import memory
import sys


def create_dijkstras_map(state_: State):

    result_dict = defaultdict(int)

    for goal_location in state_.goal_positions.keys():
        preprocessing_current_state = State(state_)
        strategy = StrategyBFS()
        strategy.reset_strategy()
        preprocessing_current_state._dijkstras_location = goal_location
        strategy.add_to_frontier(preprocessing_current_state)

        iterations = 0
        while True:

            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                raise Exception('Maximum mem used')

            if strategy.frontier_empty():
                print('Done with dijkstras', file=sys.stderr, flush=True)
                break

            leaf = strategy.get_and_remove_leaf()

            # Add element to dict
            result_dict[(goal_location, leaf._dijkstras_location)] = leaf.g

            strategy.add_to_explored(leaf)

            for child_state in leaf.get_children_dijkstras():  # The list of expanded states is shuffled randomly; see state.py.
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    return result_dict

def convert_unassigned_colors_to_walls(world_state:'State'):
    agent_ids = set(world_state.reverse_agent_dict().keys())

    agent_ids = set([str(x) for x in agent_ids])
    all_cols = list(world_state.colors_reverse.keys())

    colors=set()
    for col in all_cols:
        
        col_agt = False
        for k,v in world_state.agents.items():
            
            if v[0][0].strip() == col.strip():
                
                col_agt = True
        

        if not col_agt:
            colors.add(col.strip())



    to_be_popped=list()
    for location, values in world_state.boxes.items():
        if values[0][0] in colors:
            to_be_popped.append(location)
            world_state.walls[location] = True
    
    while len(to_be_popped)>0:
        # Remove the boxes that cannot be moved
        world_state.boxes.pop(to_be_popped.pop())

def convert_unassigned_colors_to_walls_connected_comp(world_state:'State'):

    connected_comp_boxes = defaultdict(set)

    for agt, v in world_state.agents.items():
        connected_comp_boxes[v[0][3]].add(v[0][0])
    
    _to_be_popped=[]
    for box_key, v in world_state.boxes.items():
        if v[0][0] not in connected_comp_boxes[v[0][3]]:
            _to_be_popped.append(box_key)

    while len(_to_be_popped)>0:
        _pop = _to_be_popped.pop()
        world_state.walls[_pop] = True
        
        # the blackboard works in a way where we are dependent on the internal goal id
        # world_state.boxes.pop(_pop)
