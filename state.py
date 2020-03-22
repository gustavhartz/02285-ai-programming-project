import random


class State:
    _RNG = random.Random(1)
    
    #default values, so will not work for levels bigger than 70x70.
    MAX_ROW = 70
    MAX_COL = 70
    
    #Save static level information in shared arrays
        #Data saved in 70x70 array but only loop through actual restrictions
    walls = [[False for _ in range(70)] for _ in range(70)]
    goals = [[None for _ in range(70)] for _ in range(70)]
    
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

        self.agents = []
        self.agents_goal = []
        self.boxes_goal = []

        # self.walls = [[False for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]
        self.boxes = [[None for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]
        # self.goals = [[None for _ in range(State.MAX_COL)] for _ in range(State.MAX_ROW)]


    
    def is_initial_state(self) -> 'bool':
        return self.parent is None
    
    def is_goal_state(self) -> 'bool':
        for row in range(self.MAX_ROW):
            for col in range(self.MAX_COL):
                goal = self.goals[row][col]
                box = self.boxes[row][col]
                if goal is not None and (box is None or goal != box.lower()):
                    return False
        return True
    
    def is_free(self, row: 'int', col: 'int') -> 'bool':
        return not self.walls[row][col] and self.boxes[row][col] is None
    
    def box_at(self, row: 'int', col: 'int') -> 'bool':
        return self.boxes[row][col] is not None
    
    def extract_plan(self) -> '[State, ...]':
        plan = []
        state = self
        while not state.is_initial_state():
            plan.append(state)
            state = state.parent
        plan.reverse()
        return plan
    
    def __hash__(self):
        if self._hash is None:
            prime = 31
            _hash = 1
            _hash = _hash * prime + self.agent_row
            _hash = _hash * prime + self.agent_col
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.boxes))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.goals))
            _hash = _hash * prime + hash(tuple(tuple(row) for row in self.walls))
            self._hash = _hash
        return self._hash
    
    def __eq__(self, other):
        if self is other: return True
        if not isinstance(other, State): return False
        if self.agent_row != other.agent_row: return False
        if self.agent_col != other.agent_col: return False
        if self.boxes != other.boxes: return False
        if self.goals != other.goals: return False
        if self.walls != other.walls: return False
        return True
    
    def __repr__(self):
        lines = []
        for row in range(State.MAX_ROW):
            line = []
            for col in range(State.MAX_COL):
                if self.boxes[row][col] is not None: line.append(self.boxes[row][col])
                elif self.goals[row][col] is not None: line.append(self.goals[row][col])
                elif self.walls[row][col] is not None: line.append('+')
                elif self.agent_row == row and self.agent_col == col: line.append('0')
                else: line.append(' ')
            lines.append(''.join(line))
        return '\n'.join(lines)
    
    def __lt__(self, other):
        return False

