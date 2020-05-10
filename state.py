import random
from collections import defaultdict
from action import ALL_ACTIONS, ActionType, MOVE_ACTIONS
import re
import sys
from copy import deepcopy as cp
from utils import _remove_element



class State:
    _RNG = random.Random(1)
    walls = defaultdict(bool)

    def __init__(self, copy: 'State' = None):
        '''
        If copy is None: Creates an empty State.
        If copy is not None: Creates a copy of the copy state.
        
        The lists walls, boxes, and goals are indexed from top-left of the level, row-major order (row, col).
               Col 0  Col 1  Col 2  Col 3
        Row 0: (0,0)  (0,1)  (0,2)  (0,3)  ...
        Row 1: (1,0)  (1,1)  (1,2)  (1,3)  ...
        Row 2: (2,0)  (2,1)  (2,2)  (2,3)  ...
        ...
        
        For example, self.walls is a list of size [MAX_ROW][MAX_COL] and
        self.walls[2][7] is True if there is a wall at row 2, column 7 in this state.
        
        Note: The state should be considered immutable after it has been hashed, e.g. added to a dictionary!
        '''
        self._hash = None
        if copy is None:
            # This defaultdict takes as input a string of coordinates an returns true if this posistion is a wall
            self.walls = defaultdict(bool)
            

            # this defaultdict takes as input a string of coordinates and returns a list of size 3 with color,char of the agents, and internal id
            self.agents = defaultdict(list)

            # this default_dict takes as input an agent char an returns a list of string of coordinates with len one
            self.agents_goal = defaultdict(list)

            # this default_dict takes as input the char of the box and returns a list of coordinates for that type of box goal locations
            self.boxes_goal = defaultdict(list)

            # this defaultdict takes as input a string of coordinates and returns a list of size 3 with color,char, id of the box
            self.boxes = defaultdict(list)

            # this default_dict takes as input a string of coordinates and returns a char corresponding to what element is
            # in that position
            self.goal_positions = defaultdict(str)

            # This is used for dijkstrals and should be none any other time than in preprocessing
            self._dijkstras_location = None

            # BFS from all goal_positions finding neighbouring states.
            # Input tuple(string_goal_position,string_coordinates). Returns int distance found.
            # Using dict to trigger positions not found to raise error
            self.dijkstras_map = dict()

            #
            self.colors = {}
            self.colors_reverse = defaultdict(list)

            self.parent = None
            self.action = None
            self.sub_goal_box = None

            self.g = 0

            self.search_init = True

            #Level Design variables
            self.tunnels = {}
            self.wells = {}
            self.mouths = defaultdict(list)
            self.tunnels_reverse = defaultdict(list)
            self.wells_reverse = defaultdict(list)


        else:
            self.walls = copy.walls
            self.goal_positions = copy.goal_positions
            self.parent = copy.parent
            self.action = copy.action
            self.agents = cp(copy.agents)
            self.boxes = cp(copy.boxes)
            self.search_init = False
            self.boxes_goal = copy.boxes_goal
            self.agents_goal = copy.agents_goal
            self.sub_goal_box = copy.sub_goal_box
            self.g = copy.g
            self.colors_reverse = copy.colors_reverse
            self._dijkstras_location = copy._dijkstras_location
            self.dijkstras_map = copy.dijkstras_map
            

            #Level design variables
            self.tunnels = copy.tunnels
            self.wells = copy.wells
            self.mouths = copy.mouths
            self.tunnels_reverse = copy.tunnels_reverse
            self.wells_reverse = copy.wells_reverse

    def is_initial_state(self) -> 'bool':
        return self.parent is None

    def is_in_goal_location(self, goal, pos_tuple) -> 'bool':
        if self.agents_goal[goal] == pos_tuple:
            return True
        else:
            return False

    def is_sub_goal_state_box(self, box_to: str, box_id: int) -> 'bool':
        if box_to in self.boxes and self.boxes[box_to][0][2] == box_id:
            return True
        else:
            return False

    def is_sub_goal_state_agent(self, agent_to: str, agent_id: int) -> 'bool':
        if agent_to in self.agents:
            if self.agents[agent_to][0][1] == agent_id:
                return True
            return False
        else:
            return False
    
    def box_at(self, position: str) -> 'bool':
        return position in self.boxes
    
    def extract_plan(self) -> '[State, ...]':
        plan = []
        state = self
        while not state.is_initial_state():
            plan.append(state)
            state = state.parent
        plan.reverse()
        return plan

    def is_free(self, new_position: str) -> 'bool':
        if self.search_init:
            return (new_position not in self.agents) and (new_position not in self.boxes) and (new_position not in self.walls)
        elif self._dijkstras_location is not None:
            return new_position not in self.walls
        else:
            # TODO: Solve the problem of moving into agent locations where the agent is "done"
            return (new_position not in self.walls) and (new_position not in self.boxes)

    def _update_agent_location(self,old_location: str, new_location: str):
        x = self.agents[old_location]
        self.agents.pop(old_location)
        self.agents[new_location] = x

    def _update_box_location(self, old_location: str, new_location: str):
        x = self.boxes[old_location]
        self.boxes.pop(old_location)
        self.boxes[new_location] = x

    def get_children(self, agentId) -> '[State, ...]':
        '''
        Returns a list of child states attained from applying every applicable action in the current state.
        The order of the actions is random.
        '''
        for key, value in self.agents.items():
            if value[0][1] == agentId:
                agent_location = key
        children = []
        old_agent_location = [int(x) for x in re.findall(r'\d+', agent_location)]
        old_agent_location_string = f'{old_agent_location[0]},{old_agent_location[1]}'
        for action in ALL_ACTIONS:
            # Determine if action is applicable.
            new_agent_position = [old_agent_location[0]+action.agent_dir.d_row,
                                  old_agent_location[1]+action.agent_dir.d_col]
            new_agent_location_string = f'{new_agent_position[0]},{new_agent_position[1]}'

            new_agent_row = old_agent_location[0] + action.agent_dir.d_row
            new_agent_col = old_agent_location[1] + action.agent_dir.d_col

            if action.action_type is ActionType.Move:
                if self.is_free(new_agent_location_string):
                    child = State(copy=self)
                    child._update_agent_location(old_agent_location_string, new_agent_location_string)
                    child.parent = self
                    child.action = action
                    child.g += 1
                    children.append(child)
            elif action.action_type is ActionType.Push:
                if self.box_at(new_agent_location_string):
                    if self.boxes[new_agent_location_string][0][2] == self.sub_goal_box:

                        new_box_location = f'{new_agent_position[0]+action.box_dir.d_row},{new_agent_position[1] + action.box_dir.d_col}'
                        if self.is_free(new_box_location):
                            child = State(copy=self)
                            child._update_agent_location(old_agent_location_string,new_agent_location_string)
                            child._update_box_location(new_agent_location_string,new_box_location)
                            child.parent = self
                            child.action = action
                            child.g += 1
                            children.append(child)
            elif action.action_type is ActionType.Pull:
                if self.is_free(new_agent_location_string):
                    old_box_location_string = f'{old_agent_location[0] + action.box_dir.d_row},{old_agent_location[1] + action.box_dir.d_col}'
                    if self.box_at(old_box_location_string):
                        if self.boxes[old_box_location_string][0][2] == self.sub_goal_box:
                            child = State(copy=self)
                            child._update_agent_location(old_agent_location_string, new_agent_location_string)
                            child._update_box_location(old_box_location_string, old_agent_location_string)
                            child.parent = self
                            child.action = action
                            child.g += 1
                            children.append(child)

        State._RNG.shuffle(children)
        return children

    def get_children_dijkstras(self) -> '[State, ...]':
        '''
        Gets the move child states of the given state by simulating an agent only allowed to do move actiosn
        :return: list of states (children)
        '''
        agent_location = self._dijkstras_location
        children = []
        old_agent_location = [int(x) for x in re.findall(r'\d+', agent_location)]
        for action in MOVE_ACTIONS:
            # Determine if action is applicable.
            new_agent_position = [old_agent_location[0] + action.agent_dir.d_row,
                                  old_agent_location[1] + action.agent_dir.d_col]
            new_agent_location_string = f'{new_agent_position[0]},{new_agent_position[1]}'

            # print(old_agent_location, file=sys.stderr, flush=True)
            # print(new_agent_location_string, file=sys.stderr, flush=True)
            # print(self.is_free(new_agent_location_string), file=sys.stderr, flush=True)


            if self.is_free(new_agent_location_string):
                child = State(copy=self)
                child._dijkstras_location = new_agent_location_string
                child.parent = self
                child.action = action
                child.g += 1
                children.append(child)

        State._RNG.shuffle(children)
        return children

    def reverse_agent_dict(self):
        temp = defaultdict(list)
        for key, value in self.agents.items():
            temp[value[0][1]] = [value[0][0], key, value[0][3]]
        return temp

    def reverse_boxes_dict(self):
        temp = defaultdict(list)
        for key, value in self.boxes.items():
            temp[value[0][2]] = [key, value[0][0], value[0][1]]
        return temp
    
    def reverse_agent_dict_internal(self):
        #TODO: HUSK DETNNE ÆNDRING OG SE OM DEN GIVER PROBLEMS senere hen
        temp = defaultdict(list)
        for key, value in self.agents.items():
            temp[value[0][2]] = [value[0][0], key]
        return temp

    def world_state_update(self, actions: list):
        agentDict = self.reverse_agent_dict_internal()
       
        for agentId, action in enumerate(actions):
            location = [int(x) for x in agentDict[agentId][1].split(",")]
            agent_row = location[0]
            agent_col = location[1]
        
            if action.action_type is ActionType.Move:
                #Move agent
                self.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(self.agents[f'{agent_row},{agent_col}'][0])
                
                #Remove agent from old location
                _remove_element(self.agents, f'{agent_row},{agent_col}')
            
            elif action.action_type is ActionType.Pull:
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''
               
                #Move agent
                self.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(self.agents[f'{agent_row},{agent_col}'][0])

                #Remove agent from old location
                _remove_element(self.agents, f'{agent_row},{agent_col}')

                #Move Box
                self.boxes[f'{agent_row},{agent_col}']\
                    .append(self.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'][0])

                #Remove box from old location
                _remove_element(self.boxes, f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}')

            elif action.action_type is ActionType.Push:
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''
                
                #Move Box
                self.boxes[f'{agent_row+action.agent_dir.d_row+action.box_dir.d_row},{agent_col+action.agent_dir.d_col+action.box_dir.d_col}']\
                    .append(self.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'][0])
                
                #Remove Box
                _remove_element(self.boxes, f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')

                #Move agent
                self.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(self.agents[f'{agent_row},{agent_col}'][0])
                
                #Remove agent from old location
                _remove_element(self.agents, f'{agent_row},{agent_col}')

    # SØG PÅ DENNE MÅDE if key in dict
    def world_is_goal_state(self):
        for loc, char in self.goal_positions.items():
            if char in "0123456789":
                if loc not in self.agents:
                    return False
                else: 
                    if self.agents[loc][0][1] != int(char):
                        return False
            else:
                if loc not in self.boxes:
                    return False
                else:
                    if self.boxes[loc][0][1] != char:
                        return False
        return True


    def __hash__(self):
        if self._hash is None:
            prime = 31
            _hash = 1
            temp=""
            for row, value in self.agents.items(): temp = temp + str(row) + value[0][0] + str(value[0][1])
            _hash = _hash * prime + hash(temp)
            temp = ""
            for row, value in self.boxes.items(): temp = temp + str(row) + value[0][0] + str(value[0][1]) + str(value[0][2])
            _hash = _hash * prime + hash(temp)
            temp = ""
            for row, value in self.goal_positions.items(): temp = temp + str(row) + str(value)
            _hash = _hash * prime + hash(temp)
            temp = ""
            for row, value in self.goal_positions.items(): temp = temp + str(row)
            _hash = _hash * prime + hash(temp)
            if self._dijkstras_location is not None:
                _hash = hash(self._dijkstras_location) * 11
            self._hash = _hash
        return self._hash
    
    def __eq__(self, other):
        if self is other: return True
        if not isinstance(other, State): return False
        if self.agents != other.agents: return False
        if self.boxes != other.boxes: return False
        if self.goal_positions != other.goal_positions: return False
        if self.walls != other.walls: return False
        return True

    # TO DO: Fix this to work on new repr
    def __repr__(self):
        lines = []
        lines.append(f"{self.boxes} boxes")
        lines.append(f"{self.agents} agents")
        return '\n'.join(lines)
    
    def __lt__(self, other):
        return False

