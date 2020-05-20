from abc import ABCMeta, abstractmethod
import sys
import memory
from collections import deque
from state import State
from collections import defaultdict
from strategy import StrategyBestFirst,StrategyBFS
from heuristic import heuristic_func, heuristic
from utils import _get_agt_loc, _get_box_loc
import config as _cfg
import utils
from action import Action, ActionType, Dir




# Super class that all agent classes inharit functions form
class Agent(metaclass=ABCMeta):
    None

'''
Agents are given a goal by a central unit and plan how to achieve this goal themselves

agent_goal_task
(Box_name, (start_location, end_location)

'''

class search_agent(Agent):

    def __init__(self, agent_char: int, agent_color: str, agent_internal_id: int, connected_component_id: int, strategy, heuristic=None):
        super().__init__()
        self.agent_char = agent_char
        self.agent_color = agent_color
        self.connected_component_id = connected_component_id
        self.agent_internal_id = agent_internal_id

        # Conflict only interacts with this one
        self.plan = deque()
        self.current_box_id = None
        self.plan_category = -1
        self.pending_help = False
        self.pending_task_bool = False
        self.pending_task_dict = {}
        self.pending_task_func = None
        self.helper_agt_requester_id = None
        # tuple(helper_agt.agent_char, helper_agt.agent_internal_id)
        self.helper_id = None
        self.pending_help_pending_plan = None


        #Self-help pipeline
        self.self_pending_task_funcs = []
        self.self_pending_task_dict = {}

        # Nested help variable
        self.nested_help = False


        # Used to track which jobs are currently assinged in goalassigner - 'goal_location'
        self.goal_job_id = None

        self.heuristic = heuristic
        self.strategy = strategy

    def __repr__(self):
        return 'search agent'

    def search_to_box(self, world_state: 'State', box_loc, box_id):
        if len(world_state.reverse_agent_dict()[self.agent_char]) < 1:
            raise Exception("No values for agent ")

        self.world_state = State(world_state)
        
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

    def search_with_box(self, world_state: 'State',boxes_visible:list):

        # Get current location of box trying to move
        box_from = _get_box_loc(world_state, self.current_box_id)
        print(f'Search with box: box_from: {box_from}, box_id = {self.current_box_id}')

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


        #TODO: boxes_visible added so we can tell agent if there are goal dependencies to worry about (they have to be visible)
        removed_dict = {k: v for k, v in self.world_state.boxes.items() if (k == box_from) or (k in boxes_visible)}
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

        if len(world_state.reverse_agent_dict()[self.agent_char]) < 2:
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

    def search_replanner_heuristic(self, world_state: 'State',blocked_locations: list, agent_to, box_from=None, box_to=None):
        self.world_state = State(world_state)

        print(f'agent_to {agent_to}',file=sys.stderr,flush=True)

        #Add blocked locations as walls, so the search does not include locations
        for loc in blocked_locations: 
            row,col = loc.split(",")
            self.world_state.walls[f'{row},{col}'] = True

        # TODO: Currently: quick fix to solve agent allowed moves problem - rewrite to allow for agents to move unassigned boxes
        if box_from is not None:
            self.world_state.sub_goal_box = self.world_state.boxes[box_from][0][2]

        # finding our initial location and removing all other elements to increase speed and simplify world
        for key, value in self.world_state.agents.items():
            if value[0][1] == self.agent_char:
                location = key

        removed_dict = {k: v for k, v in self.world_state.agents.items() if v[0][1] == self.agent_char}
        self.world_state.agents = defaultdict(list, removed_dict)

        if box_from is None:
            print(f'Enter box_from is none, agent_to {agent_to}',file=sys.stderr,flush=False)
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

        print(f'AAAAgent init wordstate: {self.world_state}',file=sys.stderr,flush=False)

        iterations = 0
        while True:

            if iterations == 1000:
                print(strategy.search_status(), file=sys.stderr, flush=True)
                iterations = 0
                

            if memory.get_usage() > memory.max_usage:
                print('Maximum memory usage exceeded.', file=sys.stderr, flush=True)
                for loc in blocked_locations: 
                    row,col = loc.split(",")
                    self.world_state.walls.pop(f'{row},{col}')
                return None

            if strategy.frontier_empty():
                # finished searchspace without sol
                for loc in blocked_locations: 
                    row,col = loc.split(",")
                    self.world_state.walls.pop(f'{row},{col}')
                return None

            leaf = strategy.get_and_remove_leaf()

            # h=0 is the same as goal
            # TODO: Update this to work with something else
            if strategy.heuristic.h(leaf) == 0:
                _temp_plan = leaf.extract_plan()
                for loc in blocked_locations: 
                    row,col = loc.split(",")
                    self.world_state.walls.pop(f'{row},{col}')
                return [x.action for x in _temp_plan]

            strategy.add_to_explored(leaf)

            #TODO: HVAD er det her med x
            #x = strategy.explored.pop()
            #strategy.explored.add(x)
            for child_state in leaf.get_children(self.agent_char):
                if not strategy.is_explored(child_state) and not strategy.in_frontier(child_state) and not (child_state.g  > _cfg.max_replanning_steps):
                    #print(f'XX h= {strategy.heuristic.h(leaf)}, g = {child_state.g} child_state {child_state}',file=sys.stderr,flush=True)
                    strategy.add_to_frontier(child_state)
            iterations += 1

    def search_conflict_bfs_not_in_list(self, world_state: 'State', agent_collision_internal_id, agent_collision_box, box_id,
                                        coordinates: list, move_action_allowed=True):
        '''
        This search method uses bfs to find the first location the agent can move to without being in the list
        of coordinate

        :param agent_collision_box: id of box involved in collision
        :param agent_collision_internal_id: id of agent involved in collision
        :param world_state: world_state with
        :param coordinates: list of coordinates that the agent is not allowed to be in
        :return: None (update the plan of the agent to not interfer with the coordinates give
        '''
        d = {'world_state': world_state,
                                                                            'agent_collision_internal_id': agent_collision_internal_id,
                                                                            'agent_collision_box': agent_collision_box,
                                                                            'box_id' : box_id,
                                                                            'coordinates': coordinates,
                                                                            'move_action_allowed': move_action_allowed
                                                                            }

        print(f'>>> agent char : {self.agent_char},  {d}',file=sys.stderr,flush=True)
        #

        self.world_state = State(world_state)
        if box_id is None:
            move_action_allowed = True
            print(f'>>>>Move=True, box_id=None', file=sys.stderr,flush=True)
        else:
            # fix for missing boxes in search
            self.current_box_id = box_id
        
        self.goal_job_id = None

        



        # test case
        if (box_id is not None) and (utils.cityblock_distance(
            utils._get_agt_loc(self.world_state,self.agent_char),
            utils._get_box_loc(self.world_state, box_id))>1):

            locc_agt = utils._get_agt_loc(self.world_state,self.agent_char)
            locc_box = utils._get_box_loc(self.world_state, box_id)

            print(f'locc_agt = {locc_agt}',file=sys.stderr,flush=True)
            print(f'locc_box = {locc_box}',file=sys.stderr,flush=True)

            print(f'?????????????? city b dist {utils.cityblock_distance(utils._get_agt_loc(self.world_state,self.agent_char),utils._get_box_loc(self.world_state, box_id))}',file=sys.stderr,flush=True)
            box_id=None
            move_action_allowed = True

            print(f'>>>>Move=True, 332', file=sys.stderr,flush=True)

        

        #If agt is asked to move out of someones plan, then include this someone in the planinng
        if agent_collision_internal_id is not None:
            self.world_state.redirecter_search = True

        removed_dict = {k: v for k, v in self.world_state.agents.items() if (v[0][1] == self.agent_char)
                        or (v[0][2] == agent_collision_internal_id)}

        self.world_state.agents = defaultdict(list, removed_dict)
        

        removed_dict = {k: v for k, v in self.world_state.boxes.items() if (v[0][2] == self.current_box_id)
                        or (v[0][2] == agent_collision_box)}

        self.world_state.boxes = defaultdict(list, removed_dict)

        strategy = StrategyBFS()

        # In case there has been a previous search we need to clear the elements in the strategy object
        strategy.reset_strategy()

        # Fix error with state not knowing which box is allowed to be moved
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
                ''' 
                # TODO: Could not find a location where agent is not "in the way" - return something that triggers
                the other collision object to move out of the way
                '''
                
                return False
                raise NotImplementedError()

            leaf = strategy.get_and_remove_leaf()

            agt_loc = utils._get_agt_loc(leaf, self.agent_char)

            if box_id is not None:
                box_loc = utils._get_box_loc(leaf, self.current_box_id)
            
            if (box_id is None) and (agt_loc not in coordinates):
                self._reset_plan()
                self._convert_plan_to_action_list(leaf.extract_plan())
                return True
                #break
            else:
                if agt_loc not in coordinates and box_loc not in coordinates:
                    self._reset_plan()
                    self._convert_plan_to_action_list(leaf.extract_plan())
                    return True


            strategy.add_to_explored(leaf)
            x = strategy.explored.pop()
            strategy.explored.add(x)

            for child_state in leaf.get_children(
                    self.agent_char, move_allowed=move_action_allowed):  # The list of expanded states is shuffled randomly;

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
    def _reset_plan(self):
        self.plan = deque()
    
    def _reset_from_help(self):
        self.current_box_id = None
        self.plan_category = _cfg.no_task
        self.pending_help = False
        self.pending_task_bool = False
        self.helper_agt_requester_id = None
        self.helper_id = None
        self.pending_help_pending_plan = None
        self.goal_job_id = None
        #
        self.pending_task_dict = {}
        self.pending_task_func = None
    
    def _resume_plan(self):
        if self.current_box_id is not None:
            self.plan_category = _cfg.goal_assigner_box
        else:
            self.pending_help_pending_plan = _cfg.goal_assigner_location
        self.pending_help = False
        self.pending_task_bool = False
        self.helper_agt_requester_id = None
        # tuple(helper_agt.agent_char, helper_agt.agent_internal_id)
        self.helper_id = None
        self.pending_help_pending_plan = None

    def get_next_action(self):
        if len(self.plan)<1:
            return Action(ActionType.NoOp, None, None)
        else:
            return self.plan.popleft()

    def agent_amnesia(self):
        
        self._reset_from_help()
        self._reset_plan()
        self.goal_job_id = None


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