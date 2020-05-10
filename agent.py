from abc import ABCMeta, abstractmethod
import sys
import memory
from collections import deque
from state import State
from collections import defaultdict
from strategy import StrategyBestFirst,StrategyBFS
from heuristic import heuristic_func, heuristic
from utils import _get_agt_loc, _get_box_loc



# Super class that all agent classes inharit functions form
class Agent(metaclass=ABCMeta):

    @abstractmethod
    def search_box(self, world_state: 'State', box_from, box_to) -> 'Action': raise NotImplementedError

'''
Agents are given a goal by a central unit and plan how to achieve this goal themselves

agent_goal_task
(Box_name, (start_location, end_location)

'''

class search_agent(Agent):

    def __init__(self, agent_char: int, agent_color: str, connected_component_id: int, strategy, heuristic=None):
        super().__init__()
        self.agent_char = agent_char
        self.agent_color = agent_color
        self.connected_component_id = connected_component_id

        # Conflict only interacts with this one
        self.plan = deque()
        self.current_box_id = None
        self.plan_category = int

        # Used to track which jobs are currently assinged in goalassigner - 'goal_location'
        self.goal_job_id = None

        self.heuristic = heuristic
        self.strategy = strategy

    def __repr__(self):
        return 'search agent'

    def search_to_box(self, world_state: 'State', box_loc, box_id):
        if len(world_state.reverse_agent_dict()[self.agent_char]) != 2:
            raise Exception("No values for agent ")

        self.world_state = State(world_state)
        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)

        if self.strategy == StrategyBestFirst:
            strategy = self.strategy(heuristic.AStar(self.world_state, heuristic_func.h_goalassigner_to_box,
                                                         agent_char=self.agent_char,
                                                         box_loc=box_loc))
        else:
            strategy = self.strategy()
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}

        self.world_state.agents = defaultdict(list, removed_dict)

        self.current_box_id = box_id

        removed_dict = {k: v for k, v in self.world_state.boxes.items() if k == box_loc}

        self.world_state.boxes = defaultdict(list, removed_dict)

        strategy.add_to_frontier(state=self.world_state)

        iterations = 0
        while True:
            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

            if strategy.frontier_empty():
                print('Empty frontier', file=sys.stderr, flush=True)
                return None

            leaf = strategy.get_and_remove_leaf()

            # We are now adjecent to the box
            if strategy.heuristic.h(leaf) == 1:
                self._convert_plan_to_action_list(leaf.extract_plan())
                break

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)
            for child_state in leaf.get_children(self.agent_char):
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_with_box(self, world_state: 'State'):

        # Get current location of box trying to move
        box_from = _get_box_loc(self.world_state, self.current_box_id)

        if world_state.boxes[box_from][0][0] != self.agent_color:
            raise Exception("Agent cannot move this box")

        self.world_state = State(world_state)

        if self.strategy == StrategyBestFirst:
            strategy = self.strategy(heuristic.AStar(self.world_state, heuristic_func.h_goalassigner_with_box,
                                                         agent_char=self.agent_char,
                                                         goal_loc=self.goal_job_id,
                                                         box_id=self.current_box_id))
        else:
            strategy = self.strategy()
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        # Update state so the agent only is aware of itself
        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}
        self.world_state.agents = defaultdict(list, removed_dict)

        removed_dict = {k: v for k, v in self.world_state.boxes.items() if k == box_from}
        self.world_state.boxes = defaultdict(list, removed_dict)

        # make sure we only move this box
        self.world_state.sub_goal_box = self.current_box_id

        strategy.add_to_frontier(state=self.world_state)

        iterations = 0
        while True:
            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

            if strategy.frontier_empty():
                return None

            leaf = strategy.get_and_remove_leaf()

            if leaf.is_sub_goal_state_box(self.goal_job_id, self.current_box_id):
                self._convert_plan_to_action_list(leaf.extract_plan())
                break

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)

            for child_state in leaf.get_children(self.agent_char):  # The list of expanded states is shuffled randomly; see state.py.
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_position(self, world_state: 'State', agent_to):

        if len(world_state.reverse_agent_dict()[self.agent_char]) != 2:
            raise Exception("No values for agent ")

        self.world_state = State(world_state)
        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)

        # TODO: CHECK IF STRATEGY REFERS TO THE SAME OBJECT BEFORE IMPLEMENTING MULTI PROC

        if self.strategy == StrategyBestFirst:
            strategy = self.strategy(heuristic.AStar(self.world_state, heuristic_func.h_goalassigner_pos,
                                                         agent_char=self.agent_char,
                                                         agent_to=agent_to))
        else:
            strategy = self.strategy()
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()
        strategy = self.strategy
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        # finding our initial location and removing all other elements to increase speed and simplify world
        for key, value in self.world_state.agents.items():
            if value[0][1] == self.agent_char:
                location = key

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}

        self.world_state.agents = defaultdict(list, removed_dict)

        # Removing all e
        while len(self.world_state.boxes) > 0:
            self.world_state.boxes.popitem()
        strategy.add_to_frontier(state=self.world_state)

        iterations = 0
        while True:
            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

            if strategy.frontier_empty():
                print('Empty frontier', file=sys.stderr, flush=True)
                return None

            leaf = strategy.get_and_remove_leaf()

            if leaf.is_sub_goal_state_agent(agent_to, self.agent_char):
                self._convert_plan_to_action_list(leaf.extract_plan())
                break

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)
            for child_state in leaf.get_children(self.agent_char):
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_replanner_heuristic(self, world_state: 'State', agent_to, box_from=None, box_to=None):
        self.world_state = State(world_state)

        # TODO: Currently: quick fix to solve agent allowed moves problem - rewrite to allow for agents to move unassigned boxes
        self.world_state.sub_goal_box = self.world_state.boxes[box_from][0][2]

        # finding our initial location and removing all other elements to increase speed and simplify world
        for key, value in self.world_state.agents.items():
            if value[0][1] == self.agent_char:
                location = key

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}
        self.world_state.agents = defaultdict(list, removed_dict)

        if box_from is None:
            strategy = StrategyBestFirst(heuristic.AStar(self.world_state, heuristic_func.h_replanner_pos,
                                                         agent_to=agent_to,
                                                         agent_char=self.agent_char))
        else:
            strategy = StrategyBestFirst(heuristic.AStar(self.world_state, heuristic_func.h_replanner_pos,
                                                         agent_to=agent_to,
                                                         agent_char=self.agent_char,
                                                         box_id=self.world_state.boxes[box_from][0][2],
                                                         box_to=box_to))
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        strategy.add_to_frontier(state=self.world_state)

        iterations = 0
        while True:

            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

            if strategy.frontier_empty():
                # finished searchspace without sol
                return None

            leaf = strategy.get_and_remove_leaf()

            # h=0 is the same as goal
            # TODO: Update this to work with something else
            if strategy.heuristic.h(leaf) == 0:
                _temp_plan = leaf.extract_plan()
                return [x.action for x in _temp_plan]

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)
            for child_state in leaf.get_children(self.agent_char):
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_conflict_bfs_not_in_list(self, world_state: 'State', agent_collision_char, agent_collision_box,
                                        coordinates: list):
        '''
        This search method uses bfs to find the first location the agent can move to without being in the list
        of coordinate

        :param agent_collision_box: id of box involved in collision
        :param agent_collision_char: id of agent involved in collision
        :param world_state:
        :param coordinates:
        :return: None (update the plan of the agent to not interfer with the coordinates give
        '''


        self.world_state = State(world_state)

        removed_dict = {k: v for k, v in self.world_state.agents.items() if (v[0][1] == self.agent_char)
                        or (v[0][1] == agent_collision_char)}

        self.world_state.agents = defaultdict(list, removed_dict)

        removed_dict = {k: v for k, v in self.world_state.boxes.items() if (v[0][2] == self.current_box_id)
                        or (v[0][2] == agent_collision_box)}

        self.world_state.boxes = defaultdict(list, removed_dict)

        strategy = StrategyBFS()

        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        strategy.add_to_frontier(state=self.world_state)

        iterations = 0
        while True:
            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                return None

            if strategy.frontier_empty():
                ''' 
                # TODO: Could not find a location where agent is not "in the way" - return something that triggers
                the other collision object to move out of the way
                '''
                raise NotImplementedError()

            leaf = strategy.get_and_remove_leaf()
            agt_loc = _get_agt_loc(leaf, self.agent_char)
            box_loc = _get_box_loc(leaf, self.current_box_id)

            if agt_loc not in coordinates and box_loc not in coordinates:
                raise NotImplementedError('The agent has found a location where it '
                                          'can move to overwrite original plan and change state')
                self._convert_plan_to_action_list(leaf.extract_plan())
                break

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)

            for child_state in leaf.get_children(
                    self.agent_char):  # The list of expanded states is shuffled randomly; see state.py.
                # print("child box location value{}".format(child_state.boxes), file=sys.stderr, flush=True)
                # print("set value{}".format(x.__hash__()), file=sys.stderr, flush=True)
                # print("child value{}".format(child_state.__hash__()), file=sys.stderr, flush=True)
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def set_search_strategy(self, heuristic, strategy):
        self.heuristic = heuristic
        self.strategy = strategy

    def _convert_plan_to_action_list(self, list_states: [State, ...]):
        for state in list_states:
            self.plan.append(state.action)

    def update_old_subgoal(self):
        if len(self.plan) < 1:
            if hasattr(self, 'world_state'):
                self.world_state.sub_goal_box = None


    def __eq__(self, other):
        return self.agent_char == other.agent_char

    def __lt__(self, other):
        return self.agent_char < other.agent_char

    def __hash__(self):
        prime = 31
        _hash = 1
        _hash = _hash * prime + hash(self.agent_char)
        self._hash = _hash
        return self._hash
