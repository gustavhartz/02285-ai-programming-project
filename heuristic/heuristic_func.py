"""
This module contains all the different heuristic functions for calculating the h_value in
"""


def h_replanner_pos(self: 'Heuristic', state: 'State', dist_function) -> 'int':
    # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str
    #  self.h_max_two(state, coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str)

    if 'agent_to' not in self.data and 'agent_char' not in self.data:
        raise Exception('Using wrong heuristic. **kwargs must contain agent_data')

    for key, value in state.agents.items():
        if value[0][1] == self.agent_char:
            agent_location = key

    for key, value in state.boxes.items():
        if value == value[0][2]:
            box_location = key

    if 'box_to' in self.data:
        return max(dist_function(agent_location, self.data['agent_to']),
                   dist_function(box_location, self.data['box_to']))
    else:
        return dist_function(agent_location, self.data['agent_to'])