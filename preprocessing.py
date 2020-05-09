from strategy import StrategyBFS
from state import State
from collections import defaultdict
import memory
import sys


def create_dijkstras_map(state_: State):

    result_dict = defaultdict(int)

    for goal_location in state_.goal_positions.keys():
        preprocessing_current_state = State(state_)
        strategy = StrategyBFS()
        strategy.reset_strategy()
        preprocessing_current_state._dijkstras_location = goal_location
        strategy.add_to_frontier(preprocessing_current_state)

        iterations = 0
        while True:

            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                raise Exception('Maximum mem used')

            if strategy.frontier_empty():
                print('Done with dijkstras', file=sys.stderr, flush=True)
                break

            leaf = strategy.get_and_remove_leaf()

            # Add element to dict
            result_dict[(goal_location, leaf._dijkstras_location)] = leaf.g

            strategy.add_to_explored(leaf)

            for child_state in leaf.get_children_dijkstras():  # The list of expanded states is shuffled randomly; see state.py.
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state):
                    strategy.add_to_frontier(child_state)
            iterations += 1

    return result_dict