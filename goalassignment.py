from abc import ABCMeta, abstractmethod
from collections import defaultdict
from utils import cityblock_distance
import sys
from action import ActionType, Action
import config

'''
The goal of the assigner is from a given goal state

'''


class Assigner(metaclass=ABCMeta):

    @abstractmethod
    def assign_agents(self) -> None: raise NotImplementedError


class GoalAssigner(Assigner):

    def __init__(self, world_state: 'State', goal_dependencies: list, list_of_agents=None):
        self.world_state = world_state
        self.agents = list_of_agents
        self.assigned_tasks = []
        self.goal_dependencies = goal_dependencies
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


        # TODO: Create a update task function to handel dependencies


        # Creates all potential boxes to be moved
        box_tasks = defaultdict(list)
        used_ids = set()
        for key, value in self.world_state.boxes_goal.items():
            # Every goal_location
            for element in value:
                for box, value_box in self.world_state.boxes.items():
                    # If char is the same and box not assigned and same connected componet
                    if ((element, box) in self.world_state.dijkstras_map) \
                            and (value_box[0][2] not in used_ids) \
                            and (value_box[0][1] == key):

                        # default dict shit
                        if element not in box_tasks:
                            # box goal location, box_id
                            box_tasks[element] = [key, value_box[0][2]]
                            _temp_loc = box
                            _temp_dist = self.world_state.dijkstras_map[(element, _temp_loc)]
                        else:
                            _x = self.world_state.dijkstras_map[(element, box)]
                            if _temp_dist > _x:
                                box_tasks[element] = [key, value_box[0][2]]
                                _temp_loc = box
                                _temp_dist = _x

                used_ids.add(box_tasks[element][1])

        agent_tasks = self.world_state.agents_goal
        # TODO: print
        print(f'GOAL TASKS:\n agent : {agent_tasks} \n box : {box_tasks}', flush=True , file=sys.stderr)
        return box_tasks, agent_tasks

    def reassign_tasks(self):

        # Find goals currently satisfied
        solved_tasks = set()
        for task, values in self.box_tasks.items():
            if task in self.world_state.boxes.keys():
                # TODO: change values[3] to refer to correct element (box id)
                if self.world_state.boxes[task] == values[1]:
                    # Save the task_id (goal_location)
                    solved_tasks.add(task)

        # Find what jobs are currently getting executed
        current_execution = set()
        for agt in self.agents:
            if (agt.goal_job_id is not None) and (agt.goal_job_id not in solved_tasks):
                current_execution.add(agt.goal_job_id)

        # All tasks
        all_tasks = set(y for y in self.box_tasks.keys())

        # All potential without handeling dependencies in excecution ..
        potential_tasks = all_tasks.difference(current_execution)
        potential_tasks = potential_tasks.difference(solved_tasks)

        # All unsolved tasks with dependencies
        pending_tasks = set()
        for task in potential_tasks:
            if len(self.goal_dependencies[task])>0:
                for element in self.goal_dependencies[task]:
                    if element not in solved_tasks:
                        pending_tasks.add(task)
                        break

        # Genereate reversed box dict
        box_reversed = self._box_reversed()

        # ready to be assigned
        potential_tasks = potential_tasks.difference(pending_tasks)

        used_ids = set()
        assignments = dict()
        assignments_with_box = dict()
        for element in self.agents:
            if len(element.plan) == 0:
                # if the agent is performing a help task
                if (element.plan_category == config.solving_help_task) and (element.pending_task_bool):
                    # Update the world state
                    element.pending_task_dict['world_state'] = self.world_state
                    print("ENTER", file=sys.stderr, flush=True)
                    element.pending_task_func(**element.pending_task_dict)
                    element.pending_task_bool=False
                    continue
                
                # reset agent to be free
                if element.goal_job_id not in current_execution:
                    element._reset_from_help()
                elif element.goal_job_id == config.awaiting_help:
                    for _ele in self.agents:
                        # make _ele it's helper
                        if _ele.agent_char == element.helper_id[0]:
                            break
                    # Is it still solving the help task related to this agent
                    if _ele.helper_agt_requester_id==element.agent_char:
                        element.plan.appendleft(Action(ActionType.NoOp, None, None))

                    if element.helper_id[0]:
                        element.plan.appendleft(Action(ActionType.NoOp, None, None))
                        pass
                else:
                    if (element.goal_job_id is not None) and (element.goal_job_id not in solved_tasks):
                        print(box_reversed[element.current_box_id], file=sys.stderr, flush=True)
                        assignments_with_box[element.goal_job_id] = element
                        continue


                # We first get the goal_locations still needing boxes
                # Potential boxes

                min_dist = 10 ** 5
                best_element = None

                # location, (char, id_box)
                for k, v in self.box_tasks.items():

                    # A goal task that dosent need solving
                    if k not in potential_tasks:
                        continue

                    # Find char of this goal location and asses if we can move it
                    # make sure task is not assigned
                    # make sure connected_comp is the same

                    if (self.world_state.colors[v[0]] == element.agent_color) \
                            and (k not in used_ids)\
                            and (element.connected_component_id == box_reversed[v[1]][1]):
                        # TODO: Print
                        # Gives current distance from agent to box
                        temp_dist = cityblock_distance(self.world_state.reverse_boxes_dict()[v[1]][0],
                                                       self.world_state.reverse_agent_dict()[element.agent_char][1])
                        if min_dist > temp_dist:
                            # box_goal id, assigned box location, box_id
                            best_element = (k, box_reversed[v[1]][0], v[1])
                            min_dist = temp_dist

                used_ids.add(best_element)
                if best_element is None:
                    # no assignment for this agent
                    continue
                assignments[best_element] = element

        # If new box tasks found for agent - create format for delegate and assign
        if len(assignments_with_box)>0:
            self._delegate_task_with_box(assignments_with_box)

        if len(assignments) > 0:
            self._delegate_tasks_to_box(assignments)

        # The agents that dosen't have a task should move to a goal location
        potential = list(set(self.agents) - set(assignments.values()))

        # The agents that dosen't have a task should move to a goal location
        potential = list(set(potential) - set([x for x in self.agents if len(x.plan) > 0]))

        assignments_a = dict()
        for agent in potential:
            # Case where the agent has a move goal task, and has not solved it
            if (agent.agent_char in self.agent_tasks) and (self.agent_tasks[agent.agent_char][0] not in solved_tasks):
                assignments_a[self.agent_tasks[agent.agent_char][0]] = agent

            else:
                # No possible assignments found for agent
                # TODO: CORRECT STATE
                agent.plan_category = 1

                # TODO: Decide where the noop actions should be placed
                agent.plan.append(Action(ActionType.NoOp, None, None))

        self._delegate_tasks_agent(assignments_a)

    def _delegate_tasks_to_box(self, assignments):
        '''
        Maps the assignments to agents that should search for finding a path the the box it's assigned to
        :param assignments: dict with key (goal_loc, box_loc, box_id) and agent as value
        :return: nothing
        '''
        for k, v in assignments.items():
            v.plan_category = config.goal_assigner_box
            v.search_to_box(self.world_state, k[1], k[2])
            # Assign appropiate values
            v.goal_job_id = k[0]

    def _delegate_task_with_box(self, assignments):
        '''
        Maps the assignments to agents that should search for pushing a box to a goal location
        :param assignments: dict with key (element.goal_job_id, box_reversed[element.current_box_id]) and agent as value
        :return: nothing
        '''
        for k, v in assignments.items():
            v.plan_category = config.goal_assigner_box
            v.search_with_box(self.world_state)

    def _delegate_tasks_agent(self, assignments):
        for k, v in assignments.items():
            v.plan_category = config.goal_assigner_location
            v.search_position(self.world_state, k)

    def _box_reversed(self):
        x = dict()
        for key, value in self.world_state.boxes.items():
            x[value[0][2]] = [key, value[0][3]]
        return x