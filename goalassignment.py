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
        self.box_tasks, self.agent_tasks = self.create_tasks()

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
        used_ids = set()
        for key, value in self.world_state.boxes_goal.items():
            for element in value:
                for key_box, value_box in self.world_state.boxes.items():
                    if (value_box[0][1] == key) and (key_box not in used_ids):
                        if not len(box_tasks[element]):
                            box_tasks[element] = [key, key_box,
                                                   cityblock_distance(element, key_box)]
                        else:
                            if box_tasks[element][2] > cityblock_distance(element, key_box):
                                box_tasks[element] = [key, key_box,
                                                           cityblock_distance(element, key_box)]
                used_ids.add(box_tasks[element][1])

        agent_tasks = defaultdict(list)
        used_ids = set()
        for key, value in self.world_state.agents_goal.items():
            for element in value:
                for k, v in self.world_state.agents.items():
                    if (v[0][1] == key) and (k not in used_ids):
                        if not len(agent_tasks[key]):
                            agent_tasks[key] = [element, k,
                                                    cityblock_distance(element, k)]
                        else:
                            if agent_tasks[key][2] > cityblock_distance(element, k):
                                agent_tasks[key] = [element, key_box,
                                                        cityblock_distance(element, k)]
                used_ids.add(agent_tasks[key][1])

        print(box_tasks, file=sys.stderr, flush=True)
        # print(agent_tasks, file=sys.stderr, flush=True)
        return box_tasks, agent_tasks

    def assign_tasks(self):

        used_ids = set()
        assignments = dict()
        for element in self.agents:
            if len(element.plan) == 0:
                min_dist = 10**5
                best_element = None
                list_of_potential_elements = self.world_state.colors_reverse[element.agent_color]
                list_of_potential_elements = [x for x in list_of_potential_elements if x != element.agent_char]
                for k, v in self.box_tasks.items():
                    if (self.world_state.goal_positions[k] in list_of_potential_elements) and k not in used_ids:
                        temp_dist = cityblock_distance(k,self.world_state.reverse_agent_dict[element.agent_char])
                        if min_dist > temp_dist:
                            best_element = k
                            min_dist = temp_dist

                used_ids.add(best_element)

                # Update tasks by removing element
                self.box_tasks.pop(best_element)
                assignments[best_element] = element

        self._delegate_tasks_box(assignments)

        # The agents that dosen't have a task should move to a goal location
        potential = list(set(self.agents) - set(assignments.values()))
        assignments_a = dict()
        for agent in potential:
            if agent.agent_char in self.agent_tasks:
                assignments_a[self.agent_tasks[agent.agent_char]] = agent

                # Update tasks by removing element
                self.agent_tasks.pop(agent.agent_char)

        self._delegate_tasks_agent(assignments_a)




    def _delegate_tasks_agent(self, assignments):
        for k, v in assignments.items():
            v.search_box(self.world_state, self.box_tasks[k][1], k)

    def _delegate_tasks_box(self, assignments):
        for k, v in assignments.items():
            v.search_position(self.world_state, k)




































