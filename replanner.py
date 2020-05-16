from abc import ABCMeta, abstractmethod
from typing import List
from agent import search_agent
from collections import defaultdict
from utils import cityblock_distance
import sys
from action import ActionType, Action
from state import State
import config



'''
The goal of the assigner is from a given goal state


'''


class Replanner(metaclass=ABCMeta):

    def __init__(self, world_state: 'State' = None):
        self.world_state = None

    def replan_v1(self, world_state: 'State', agent, assigned_box: int, blocked_location: list):
        '''
        Search around object
        :param world_state:
        :param agent:
        :param assigned_box: Depreciated
        :param blocked_location:
        :return:
        '''
        self.world_state = State(world_state)
        assigned_box = agent.current_box_id
        _pop = [y for y, x in self.world_state.boxes.items() if (y not in blocked_location) or (x[0][2] == assigned_box)]
        agent_dict = self.world_state.reverse_agent_dict()

        _agts_in_blocked = [x[0][1] for y, x in self.world_state.agents.items() if (y in blocked_location)]

        # Remove boxes not in boxes_visible
        _agent_assinged_box = dict()

        # This insures that we have the box that the agent is currently assigned to
        for key, value in self.world_state.boxes.items():
            if value[0][2] == agent.current_box_id:
                break

        # Use that value to update temp_state and remove other boxes
        temp_state = State(self.world_state)
        for element in _pop:
            if element == key:
                continue
            else:
                temp_state.boxes.pop(element)

        # Remove other agents and let collision avoidance responsebility be at conflictmanager level.
        __to_be_removed = []
        for _k, _v in temp_state.agents.items():
            if _v[0][1] != agent.agent_char:
                if _v[0][1] not in _agts_in_blocked:
                    __to_be_removed.append(_k)

        while len(__to_be_removed) > 0:
            temp_state.agents.pop(__to_be_removed.pop())

        location = [int(x) for x in agent_dict[agent.agent_char][1].split(",")]
        agent_row = location[0]
        agent_col = location[1]
        box_not_moved = True
        action_counter = 0
        _replanned = False

        # Update world state for agent - check if it can be removed
        agent.world_state = State(temp_state)

        for action in agent.plan:
            action_counter += 1

            # To prevent to far replanning
            # TODO: make dependent on level size
            if action_counter> config.max_replanning_depth:
                return False

            if action.action_type is ActionType.Move:
                if box_not_moved:
                    if temp_state.is_free(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'):
                        temp_replan = agent.search_replanner_heuristic(temp_state, f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')
                        # remove old plan
                        merge_agent_plans(agent, temp_replan, action_counter)
                        _replanned = True
                        return True
                    else:
                        agent_row = agent_row + action.agent_dir.d_row
                        agent_col = agent_col + action.agent_dir.d_col
                else:
                    if temp_state.is_free(
                            f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and (temp_state.is_free(
                            f'{box_row},{box_col}')):

                        # Find box from
                        for k, v in temp_state.boxes.items():
                            if agent.current_box_id == v[0][2]:
                                k_temp = k
                                break
                            else:
                                k_temp = None

                        temp_replan = agent.search_replanner_heuristic(temp_state,
                                                        agent_to=f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}',
                                                        box_from=k_temp,
                                                        box_to=f'{box_row},{box_col}')

                        merge_agent_plans(agent, temp_replan, action_counter)
                        _replanned = True
                        return True

                    elif temp_state.is_free(
                            f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and not (temp_state.is_free(
                            f'{box_row},{box_col}')):
                        raise Exception('Weird case of movement for agent/replanning/shit')

                    else:
                        box_not_moved = False
                        agent_row = agent_row + action.agent_dir.d_row
                        agent_col = agent_col + action.agent_dir.d_col

            elif action.action_type is ActionType.Pull:
                if temp_state.is_free\
                            (f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'):

                    # Find box from
                    for k, v in temp_state.boxes.items():
                        if agent.current_box_id == v[0][2]:
                            k_temp = k
                            break
                        else:
                            k_temp = None

                    temp_replan = agent.search_replanner_heuristic(temp_state,
                                                                agent_to=f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}',
                                                                box_from=k_temp,
                                                                box_to=f'{box_row},{box_col}')

                    merge_agent_plans(agent, temp_replan, action_counter)
                    _replanned=True
                    return True

                elif temp_state.is_free(
                        f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and not (
                temp_state.is_free(
                    f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}')):
                    raise Exception('Weird case of movement for agent/replanning/shit')

                else:
                    box_not_moved = False
                    box_row = agent_row
                    box_col = agent_col
                    agent_row = agent_row + action.agent_dir.d_row
                    agent_col = agent_col + action.agent_dir.d_col

            elif action.action_type is ActionType.Push:
                if (temp_state.is_free
                    (f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}')) and \
                        (temp_state.is_free
                            (f'{agent_row + action.agent_dir.d_row + action.box_dir.d_row},{agent_col + action.agent_dir.d_col + action.box_dir.d_col}')):

                    # Find box from
                    for k, v in temp_state.boxes.items():
                        if agent.current_box_id == v[0][2]:
                            k_temp = k
                            break
                        else:
                            k_temp = None

                    temp_replan = agent.search_replanner_heuristic(temp_state,
                                                                agent_to=f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}',
                                                                box_from=k_temp,
                                                                box_to=f'{agent_row + action.agent_dir.d_row + action.box_dir.d_row},{agent_col + action.agent_dir.d_col + action.box_dir.d_col}')
                    merge_agent_plans(agent, temp_replan, action_counter)
                    _replanned=True
                    return True

                elif (temp_state.is_free
                    (f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}')) and \
                        not (temp_state.is_free
                            (f'{agent_row + action.agent_dir.d_row + action.box_dir.d_row},{agent_col + action.agent_dir.d_col + action.box_dir.d_col}')):
                    raise Exception('Weird case of movement for agent/replanning/shit')

                else:
                    box_not_moved = False
                    agent_row = agent_row + action.agent_dir.d_row
                    agent_col = agent_col + action.agent_dir.d_col
                    box_row = agent_row + action.agent_dir.d_row + action.box_dir.d_row
                    box_col = agent_col + action.box_dir.d_col + action.box_dir.d_col

        # TODO: Implement if free location not found (goal location blocked, too far away)
        if not _replanned:
            print(temp_state, file=sys.stderr, flush=True)
            raise Exception('Goal location blocked or unreachable')
            return False
        # print(agent.plan, file=sys.stderr, flush=True)
        # self.color_goals = self.create_color_goals()

def merge_agent_plans(agent, temp_replan, action_counter):
    print("________", file=sys.stderr, flush=True)
    print(agent.plan, file=sys.stderr, flush=True)
    while action_counter > 0:
        agent.plan.popleft()
        action_counter -= 1
    # Append new plan
    while len(temp_replan) > 0:
        agent.plan.appendleft(temp_replan.pop())
    print(agent.plan, file=sys.stderr, flush=True)


