from utils import _get_agt_loc, _get_box_loc
import sys
"""
This module contains all the different heuristic functions for calculating the h_value in
"""


def h_replanner_pos(self: 'Heuristic', state: 'State', dist_function) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str
    #  self.h_max_two(state, coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str)

    if 'agent_to' not in self.data and 'agent_char' not in self.data:
        raise Exception('Using wrong heuristic. **kwargs must contain agent_data')

    agent_location = _get_agt_loc(state, self.data['agent_char'])

    if 'box_to' in self.data:

        box_location = _get_box_loc(state, self.data['box_id'])
        return max(dist_function(agent_location, self.data['agent_to']),
                   dist_function(box_location, self.data['box_to']))
    else:
        return dist_function(agent_location, self.data['agent_to'])


def h_goalassigner_box(self: 'Heuristic', state: 'State', dist_function) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str
    #  self.h_max_two(state, coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str)

    agent_location = _get_agt_loc(state, self.data['agent_char'])

    if 'box_to' in self.data:
        box_location = _get_box_loc(state, self.data['box_id'])
        return dist_function(box_location, self.data['box_to']) + dist_function(agent_location, box_location) - 1
    else:
        raise Exception('Using wrong heuristic. **kwargs must contain agent_data')


def h_goalassigner_pos(self: 'Heuristic', state: 'State', dist_function) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str

    if 'agent_to' not in self.data and 'agent_char' not in self.data:
        raise Exception('Using wrong heuristic. **kwargs must contain agent_data')
    agent_location = _get_agt_loc(state, self.data['agent_char'])
    return dist_function(agent_location, self.data['agent_to'])


def h_goalassigner_to_box(self: 'Heuristic', state: 'State', dist_function) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str
   

    if 'box_loc' not in self.data and 'agent_char' not in self.data:
        raise Exception('Using wrong heuristic. **kwargs must contain box_loc')
    agent_location = _get_agt_loc(state, self.data['agent_char'])
    return dist_function(agent_location, self.data['box_loc'])

def h_goalassigner_with_box(self: 'Heuristic', state: 'State', dist_functio) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str

    if 'box_id' not in self.data and 'agent_char' not in self.data and 'goal_loc' not in self.data:
        raise Exception('Using wrong heuristic. **kwargs must contain box_loc')
    box_location = _get_box_loc(state, self.data['box_id'])
    return state.dijkstras_map[(self.data['goal_loc'], box_location)]
