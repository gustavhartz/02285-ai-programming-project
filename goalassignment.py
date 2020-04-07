from abc import ABCMeta, abstractmethod
from typing import List
from agent import search_agent
from collections import defaultdict
from utils import cityblock_distance
import sys



'''
The goal of the assigner is from a given goal state


'''

class Assigner(metaclass=ABCMeta):

    @abstractmethod
    def assign_agents(self) -> None: raise NotImplementedError


class GoalAssigner(Assigner):

    def __init__(self, world_state: 'State', list_of_agents=None):
        self.world_state = world_state
        self.agents = list_of_agents
        self.assigned_tasks = []
        # self.color_goals = self.create_color_goals()

    def assign_agents(self):
        list_of_reassignments = []
        for agent in self.agents:
            if len(agent.plan) == 0:
                list_of_reassignments.append(agent)

    def create_color_goals(self):
        color_goals = {}

        for key, value in self.world_state.colors.items:
            if key in [str(x) for x in range(0,10)]:
                continue
            else:
                color_goals[value] = key
        return color_goals

    def create_tasks(self):
        '''
        Start of by creating an auction where boxes are bidding their manhatten distanve


        :return: list of tasks
        '''

        # Creates all potential boxes to be moved
        box_tasks = defaultdict(list)
        used_ids = []
        for key, value in self.world_state.boxes_goal.items():
            for element in value:
                for key_box, value_box in self.world_state.boxes.items():
                    if value_box[0][1] == key and key_box not in used_ids:
                        if not len(box_tasks[element]):
                            box_tasks[element] = [key, key_box,
                                                   cityblock_distance(element, key_box)]
                        else:
                            if box_tasks[element][2] > cityblock_distance(element, key_box):
                                box_tasks[element] = [key, key_box,
                                                           cityblock_distance(element, key_box)]
                used_ids.append(box_tasks[element][1])
        print(box_tasks, file=sys.stderr, flush=True)
















