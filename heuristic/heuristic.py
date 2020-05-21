from abc import ABCMeta, abstractmethod
import math
import utils
import sys


class Heuristic(metaclass=ABCMeta):

    def __init__(self, initial_state: 'State', heuristic_function, dist_function=utils.cityblock_distance, **kwargs):

        self.initial_state = initial_state
        self.heuristic_function = heuristic_function
        self.dist_function = dist_function
        self.data = kwargs


    @abstractmethod
    def __repr__(self):
        raise NotImplementedError

    @abstractmethod
    def f(self, state: 'State'):
        raise NotImplementedError

    # # add g value to make this the correct value
    # def h(self, state: 'State') -> 'int':
    #     return self.heuristic_function(self, state, self.dist_function)


class AStar(Heuristic):
    def __init__(self, initial_state: 'State', heuristic_function, **kwargs):
        super().__init__(initial_state, heuristic_function, **kwargs)
    #TODO: fix this A-star shit

    def h(self, state: 'State') -> 'int':
        return self.heuristic_function(self, state, self.dist_function)

    def f(self, state: 'State') -> 'int':
        return self.h(state) + state.g

    def __repr__(self):
        return 'A* evaluation'


class WAStar(Heuristic):
    def __init__(self, initial_state: 'State', w: 'int', heuristic_function, **kwargs):
        super().__init__(initial_state, heuristic_function, **kwargs)
        self.w = w

    def f(self, state: 'State') -> 'int':
        return state.g + self.w * self.h(state)

    def __repr__(self):
        return 'WA* ({}) evaluation'.format(self.w)


class Greedy(Heuristic):
    def __init__(self, initial_state: 'State', heuristic_function, **kwargs):
        super().__init__(initial_state, heuristic_function, **kwargs)

    def f(self, state: 'State') -> 'int':
        return self.h(state)

    def __repr__(self):
        return 'Greedy evaluation'