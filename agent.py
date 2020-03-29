from abc import ABCMeta, abstractmethod
import sys
import memory
from collections import deque
from state import State


# Super class that all agent classes inharit functions form
class Agent(metaclass=ABCMeta):

    @abstractmethod
    def act(self) -> 'Action': raise NotImplementedError

'''
Agents are given a goal by a central unit and plan how to achieve this goal themselves

agent_goal_task
(Box_name, (start_location, end_location)

'''
class search_agent(Agent):

    def __init__(self, agent_id: int, agent_color: chr, agent_goal_task: str, heuristic, strategy):
        super().__init__()
        self.agent_id = agent_id
        self.agent_color = agent_color
        self.agent_goal_task = agent_goal_task

        # Conflict only interacts with this one
        self.plan = deque()

        self.heuristic = heuristic
        self.strategy = strategy

    def __repr__(self):
        return 'search agent'

    def search(self, world_state: State):
        self.world_state = world_state
        print('Starting search with strategy {}.'.format(self.strategy), file=sys.stderr, flush=True)
        strategy = self.strategy
        strategy.add_to_frontier(self.initial_state)

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

            if leaf.is_goal_state():
                return leaf.extract_plan()

            strategy.add_to_explored(leaf)
            for child_state in leaf.get_children():  # The list of expanded states is shuffled randomly; see state.py.
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)

            iterations += 1




        # Calculate the execution

    def execute_action(self):
        return False
        # Return an action

    def set_search_strategy(self, heuristic, strategy):
        self.heuristic = heuristic
        self.strategy = strategy


