from abc import ABCMeta, abstractmethod
from typing import List
from agent import search_agent
from collections import defaultdict
from utils import cityblock_distance
import sys
from action import ActionType, Action
from state import State



'''
The goal of the assigner is from a given goal state


'''


class Replanner(metaclass=ABCMeta):

    def __init__(self, world_state: 'State', list_of_agents):
        self.world_state = State(world_state)
        self.agents = list_of_agents

    def replan_v1(self, illegal_movers, boxes_visible):

        _pop = [y for y, x in self.world_state.boxes.items() if x[2] not in boxes_visible]
        agent_dict=self.world_state.reverse_agent_dict()

        # Remove boxes not in boxes_visible
        for element in _pop:
            self.world_state.boxes.pop[_pop]

        for agent in self.agents:
            location = [int(x) for x in agent_dict[agent.agent_char][1].split(",")]
            agent_row = location[0]
            agent_col = location[1]
            box_not_moved = True
            action_counter = 0

            for action in agent.plan:
                action_counter += 1
                if action.action_type is ActionType.Move:
                    if box_not_moved:
                        if self.world_state.is_free(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'):
                            agent.search_replanner_heuristic(self.world_state, f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')
                            # Plan to this location from original
                            break
                        else:
                            agent_row = agent_row + action.agent_dir.d_row
                            agent_col = agent_col + action.agent_dir.d_col
                    else:
                        if self.world_state.is_free(
                                f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and (self.world_state.is_free(
                                f'{box_row},{box_col}')):

                            # Find box from
                            agent.search_replanner_heuristic(self.world_state,
                                                             agent_to=f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}',
                                                             box_from='2,2',
                                                             box_to=f'{box_row},{box_col}')

                            break
                        elif self.world_state.is_free(
                                f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and not (self.world_state.is_free(
                                f'{box_row},{box_col}')):
                            raise Exception('Weird case of movement for agent/replanning/shit')

                        else:
                            box_not_moved = False
                            agent_row = agent_row + action.agent_dir.d_row
                            agent_col = agent_col + action.agent_dir.d_col

                elif action.action_type is ActionType.Pull:
                    if (self.world_state.is_free
                        (f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}')) and \
                        (self.world_state.is_free
                            (f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')):
                        # Plan to this location from original
                        break

                    elif self.world_state.is_free(
                            f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}') and not (
                    self.world_state.is_free(
                        f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}')):
                        raise Exception('Weird case of movement for agent/replanning/shit')

                    else:
                        box_not_moved = False
                        box_row = agent_row
                        box_col = agent_col
                        agent_row = agent_row + action.agent_dir.d_row
                        agent_col = agent_col + action.agent_dir.d_col



                elif action.action_type is ActionType.Push:
                    if (self.world_state.is_free
                        (f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}')) and \
                            (self.world_state.is_free
                                (f'{agent_row + action.agent_dir.d_row + action.box_dir.d_row},{agent_col + action.agent_dir.d_col + action.box_dir.d_col}')):
                    # Plan to this location from original
                        break

                    elif (self.world_state.is_free
                        (f'{agent_row + action.agent_dir.d_row},{agent_col + action.agent_dir.d_col}')) and \
                            not (self.world_state.is_free
                                (f'{agent_row + action.agent_dir.d_row + action.box_dir.d_row},{agent_col + action.agent_dir.d_col + action.box_dir.d_col}')):
                        raise Exception('Weird case of movement for agent/replanning/shit')

                    else:
                        box_not_moved = False
                        agent_row = agent_row + action.agent_dir.d_row
                        agent_col = agent_col + action.agent_dir.d_col
                        box_row = agent_row + action.agent_dir.d_row + action.box_dir.d_row
                        box_col = agent_col + action.box_dir.d_col + action.box_dir.d_col

            # Generate states for all agents containing information agent_id: str, box_id: str, goal_agent: str, goal_box: str
            # store this value in the state class so it's passed down to the heuristic
            # The heuristic will take this value from the state and calculate the heuristic value
            # tasks.append([distinct state, agent])

        # Sync plans





        # self.color_goals = self.create_color_goals()