from abc import ABCMeta, abstractmethod
import math
import utils


class Heuristic(metaclass=ABCMeta):

    def __init__(self, initial_state: 'State'):
        None

    @abstractmethod
    def __repr__(self):
        raise NotImplementedError

    def h_max_two(self, state: 'State', coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str) -> 'int':

        # Straight Line distance
        return max(utils.cityblock_distance(coordinate_agent,goal_agent), utils.cityblock_distance(coordinate_box, goal_box))



class AStar(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)

    def f(self, state: 'State') -> 'int':
        # CALCULATE THE VALUES FROM LOCAL VARIABLE IN STATE coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str
        # return state.g + self.h_max_two(state, coordinate_agent: str, coordinate_box: str, goal_agent: str, goal_box: str)
        None

    def __repr__(self):
        return 'A* evaluation'


class WAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int'):
        super().__init__(initial_state)
        self.w = w

    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)

    def __repr__(self):
        return 'WA* ({}) evaluation'.format(self.w)


class Greedy(Heuristic):
    def __init__(self, initial_state: 'State'):
        super().__init__(initial_state)

    def f(self, state: 'State') -> 'int':
        return self.h(state)

    def __repr__(self):
        return 'Greedy evaluation'