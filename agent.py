from abc import ABCMeta, abstractmethod
import sys
import memory
from collections import deque
from state import State
from collections import defaultdict
from strategy import StrategyBestFirst
from heuristic import heuristic_func, heuristic


# Super class that all agent classes inharit functions form
class Agent(metaclass=ABCMeta):

    @abstractmethod
    def search_box(self) -> 'Action': raise NotImplementedError

'''
Agents are given a goal by a central unit and plan how to achieve this goal themselves

agent_goal_task
(Box_name, (start_location, end_location)

'''


class search_agent(Agent):

    def __init__(self, agent_char: int, agent_color: str, strategy, heuristic=None):
        super().__init__()
        self.agent_char = agent_char
        self.agent_color = agent_color

        # Conflict only interacts with this one
        self.plan = deque()

        self.heuristic = heuristic
        self.strategy = strategy()

    def __repr__(self):
        return 'search agent'

    def search_box(self, world_state: 'State', box_from, box_to):

        if world_state.boxes[box_from][0][0] != self.agent_color:
            raise Exception("Agent cannot move this box")

        self.world_state = State(world_state)
        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)
        strategy = self.strategy
        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        # remove
        for key, value in self.world_state.agents.items():
            if value[0][1] == self.agent_char:
                location = key

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}
        self.world_state.agents = defaultdict(list, removed_dict)

        for key, value in self.world_state.boxes.items():
            if key == box_from:
                box_id = value[0][2]

        removed_dict = {k: v for k, v in self.world_state.boxes.items() if k == box_from}
        self.world_state.boxes = defaultdict(list, removed_dict)

        self.world_state.sub_goal_box = box_id
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

            if leaf.is_sub_goal_state_box(box_to, box_id):
                self._convert_plan_to_action_list(leaf.extract_plan())
                break

            strategy.add_to_explored(leaf)
            x=strategy.explored.pop()
            strategy.explored.add(x)

            for child_state in leaf.get_children(self.agent_char):  # The list of expanded states is shuffled randomly; see state.py.
                # print("child box location value{}".format(child_state.boxes), file=sys.stderr, flush=True)
                # print("set value{}".format(x.__hash__()), file=sys.stderr, flush=True)
                # print("child value{}".format(child_state.__hash__()), file=sys.stderr, flush=True)
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_position(self, world_state: 'State', agent_to):

        if len(world_state.reverse_agent_dict()[self.agent_char]) != 2:
            raise Exception("No values for agent ")

        self.world_state = State(world_state)
        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)
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

        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)

        # finding our initial location and removing all other elements to increase speed and simplify world
        for key, value in self.world_state.agents.items():
            if value[0][1] == self.agent_char:
                location = key

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}
        self.world_state.agents = defaultdict(list, removed_dict)

        strategy = StrategyBestFirst(heuristic.AStar(self.world_state, heuristic_func.h_replanner_pos,
                                                     agent_to=agent_to,
                                                     agent_char=self.agent_char,
                                                     box_char=self.world_state.boxes[box_from][0][2],
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
            if strategy.heuristic.h(leaf) == 0:
                raise Exception('Found a solution needs implementation')
                return leaf.extract_plan()
                break

            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)
            for child_state in leaf.get_children(self.agent_char):
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def set_search_strategy(self, heuristic, strategy):
        self.heuristic = heuristic
        self.strategy = strategy

    def _convert_plan_to_action_list(self, list_states: [State, ...]):
        for state in list_states:
            self.plan.append(state.action)


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
