
from state import State
from action import Action, ActionType, Dir
from collections import deque
from pulp import *
import sys
import utils
import config
import math
from replanner import Replanner

#Squarebrackets på alle kald til dicts


class ConflictManager:


    def __init__(self,agents:list):

        '''
        Percept the world with current state
        '''
        self.world_state = None
        self.blackboard = {}

        self._create_blackboard()



    '''
    For prereqs: Hvis en agent's prereqs ikke er opfyldt så skal den have NoOp, hvis ikke det er et illegalt move

    For effects: 

    '''


    def _create_blackboard(self):

        len_agt = len(self.world_state.agents)
        len_box = len(self.world_state.boxes)

        time_zero = [0]*(len_agt+len_box)


        for loc,agt in self.world_state.agents.items():
            row, col = loc.split(',')
            time_zero[agt[0][2]] = (int(row),int(col))
        for loc,box in self.world_state.boxes.items():
            row, col = loc.split(',')
            time_zero[box[0][2] + len_agt] = (int(row),int(col))
        
        #TODO: location variablen skal konstisten være str eller tuple 
        
        #Insert into the blacboard
        self.blackboard[0] = time_zero

            

    def update_blackboard(self,agents:list,time:int):

        ##########TODO:
        ### HUSK AT DENNE TAGER UDGANGSPUNKT I AT KUN EN/1 BOKS 
        ### KAN VÆRE INKLUDERET I EN PLAN SAMTIDIG
        ##########



        '''
        This method shall be called inside the agent-controll loop, so first time = 1
        time = Global time step
        
        '''
        len_agents = len(agents)

        min_plan_length = math.inf

        moved_boxes = [False]*len(self.world_state.boxes)

        for agt_id,agent in enumerate(agents):
            T = time

            if len(agent.plan) < min_plan_length:
                min_plan_length = len(agent.plan)

            if T not in self.blackboard:
                self.blackboard[T] = [0] * (len(self.world_state.agents)+len(self.world_state.boxes))

            box_id = None 

            counter = 0
            while True:
                #Run through all elements of the plan
                try:
                    action = agent.plan[counter]
                except:
                    break

                if action.action_type is ActionType.NoOp:
                    self.blackboard[T][agt_id] = self.blackboard[T-1][agt_id] 
                    
                elif action.action_type is ActionType.Move:
                    agt_row, agt_col = self.blackboard[T-1][agt_id]
                    
                    new_agt_row = agt_row + action.agent_dir.d_row
                    new_agt_col = agt_col + action.agent_dir.d_col

                    self.blackboard[T][agt_id] = (new_agt_row,new_agt_col)
                    
                elif action.action_type is ActionType.Push:
                    agt_row, agt_col = self.blackboard[T-1][agt_id]
                    new_agt_row = agt_row + action.agent_dir.d_row
                    new_agt_col = agt_col + action.agent_dir.d_col

                    box_row = new_agt_row 
                    box_col = new_agt_col 
                    
                    new_box_row = box_row + action.box_dir.d_row
                    new_box_col = box_col + action.box_dir.d_col

                    if box_id == None:
                        box_id =  self.world_state.boxes[f'{box_row},{box_col}'][0][2]


                    self.blackboard[T][agt_id] = (new_agt_row,new_agt_col)
                    self.blackboard[T][box_id+len_agents] = (new_box_row,new_box_col)

                    moved_boxes[box_id] = True

                elif action.action_type is ActionType.Pull:
                    agt_row, agt_col = self.blackboard[T-1][agt_id]
                    new_agt_row = agt_row + action.agent_dir.d_row
                    new_agt_col = agt_col + action.agent_dir.d_col

                    box_row = agt_row +  action.box_dir.d_row
                    box_col = agt_col +  action.box_dir.d_col
                    
                    new_box_row = agt_row 
                    new_box_col = agt_col

                    if box_id == None:
                        box_id = self.world_state.boxes[f'{box_row},{box_col}'][0][2]
                    
                    self.blackboard[T][agt_id] = (new_agt_row,new_agt_col)
                    self.blackboard[T][box_id+len_agents] = (new_box_row,new_box_col)

                    moved_boxes[box_id] = True
                
                counter+=1
                T+=1
        ##
        T = time
        for b_id,moved in enumerate(moved_boxes):
            if not moved:
                for i in range(T,T+min_plan_length):
                    print(i)
                    self.blackboard[i][b_id+len_agents] = self.blackboard[i-1][b_id+len_agents]

    
    def delete_from_blackboard(self,time):
        #Must always be run after sending actions to the server 
        self.blackboard.pop(time)

    def prereq_check(self,agents: list, agentDict):
        #temp_state_create skal være kørt forinden dette, så vi kan bruge 
        #World_state SKAL være opdateret i dette trin

        illegal_movers = []     
        
        for agent in agents:
            action = agent.plan[0]
            location = [int(x) for x in agentDict[agent.agent_char][1].split(",")]
            agent_row = location[0]
            agent_col = location[1]

            #Fix for NoOp
            if action.action_type is ActionType.NoOp:
                new_agent_position = [agent_row,agent_col]
            else:
                new_agent_position = [agent_row+action.agent_dir.d_row,
                                  agent_col+action.agent_dir.d_col]

            new_agent_location_string = f'{new_agent_position[0]},{new_agent_position[1]}'

            ############################################################
            #KUN illegalt move hvis boksen ikke er inkluderet i en plan#
            #TODO: Passer dette stadig
            assigned_boxes = [agent.world_state.sub_goal_box for agent in agents]
            __assigned_boxes_agent = [[agent.world_state.sub_goal_box, agent.agent_char] for agent in agents]
            __box_visible_replanning = [x for x in self.world_state.boxes.values() if x[2] not in assigned_boxes]
            ############################################################


            if action.action_type is ActionType.Move:
                if not self.world_state.is_free(new_agent_location_string):
                    if new_agent_location_string in self.world_state.agents:
                        #Giv NoOp, da agenten nok flytter sig snart
                        agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                    else:
                        #Hvis en boks, tjek om stationær:
                        box_id = self.world_state.boxes[new_agent_location_string][0][2]

                        if box_id in assigned_boxes:
                            # Find agent pushing box's location
                            for item in __assigned_boxes_agent:
                                if item[0] == box_id:
                                    __assigned_agent = item[1]
                            if utils.cityblock_distance(new_agent_location_string, agentDict[__assigned_agent][1]) > config.illegal_move_threshold:
                                # INITIALIZE REPLAN (FIND ALTERNATIVE ROUTE AROUND OBJECT)
                                __box_visible_replanning.append(box_id)
                                illegal_movers.append(agent.agent_char)
                                None
                            else:
                                agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                        else:
                            #Hvis box er stationær skal der replannes. Input et NoOp, så action ikke fejler bagefter
                            illegal_movers.append(agent.agent_char)
                            # agent.plan.appendleft(Action(ActionType.NoOp, None, None))

            elif action.action_type is ActionType.Pull:
                if not self.world_state.is_free(new_agent_location_string):
                    if new_agent_location_string in self.world_state.agents:
                        #Giv NoOp, da agenten nok flytter sig snart
                        agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                    else:
                        #Hvis en boks, tjek om stationær:
                        box_id = self.world_state.boxes[new_agent_location_string][0][2]

                        if box_id in assigned_boxes:
                            # Find agent pushing box's location
                            for item in __assigned_boxes_agent:
                                if item[0] == box_id:
                                    __assigned_agent = item[1]
                            if utils.cityblock_distance(new_agent_location_string,
                                                        agentDict[__assigned_agent][1]) > config.illegal_move_threshold:
                                # INITIALIZE REPLAN (FIND ALTERNATIVE ROUTE AROUND OBJECT)
                                __box_visible_replanning.append(box_id)
                                illegal_movers.append(agent.agent_char)
                                None
                            else:
                                agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                        else:
                            #Hvis box er stationær skal der replannes. Input et NoOp, så action ikke fejler bagefter
                            illegal_movers.append(agent.agent_char)
                            # agent.plan.appendleft(Action(ActionType.NoOp, None, None))
            elif action.action_type is ActionType.Push:
                box_loc_string = f'{agent_row+action.agent_dir.d_row+action.box_dir.d_row},{agent_col+action.agent_dir.d_col+action.box_dir.d_col}'

                if not self.world_state.is_free(box_loc_string):
                    if box_loc_string in self.world_state.agents:
                        agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                    else:
                        box_id = self.world_state.boxes[box_loc_string][0][2]

                        if box_id in assigned_boxes:
                            # Find agent pushing box's location
                            for item in __assigned_boxes_agent:
                                if item[0] == box_id:
                                    __assigned_agent = item[1]
                            if utils.cityblock_distance(box_loc_string,
                                                        agentDict[__assigned_agent][1]) > config.illegal_move_threshold:
                                # INITIALIZE REPLAN (FIND ALTERNATIVE ROUTE AROUND OBJECT)
                                __box_visible_replanning.append(box_id)
                                illegal_movers.append(agent.agent_char)
                                None
                            else:
                                agent.plan.appendleft(Action(ActionType.NoOp, None, None))
                        else:
                            #Hvis box er stationær skal der replannes. Input et NoOp, så action ikke fejler bagefter
                           illegal_movers.append(agent.agent_char)
                           # agent.plan.appendleft(Action(ActionType.NoOp, None, None))

        # Create replanning object and handle illegal moves
        Replanner(self.world_state, agents)
        Replanner.replan_v1(illegal_movers, __box_visible_replanning)
        return illegal_movers


    #Efter temp_state er blevet konstrueret, skal konflikterne findes. 
    # Når konflikterne er fundet skal vi finde en måde at spore hvile agenter der konflikter 
    # Til sidst finde en måde at løse det på.


    def temp_state_create(self, agents: list, agentDict):
        #Kontrollere foreløbigt et enkelt state
        #antage at world_state bliver opdateret i agent control loop, så vi hver gang i denne metode har det korrekte billede af miljøet. 
        
        temp_state = State(copy=self.world_state)

        box_agent_match = {}

        for agent in agents:
            action = agent.plan[0] # Et enkelt state
            location = [int(x) for x in agentDict[agent.agent_char][1].split(",")]
            agent_row = location[0]
            agent_col = location[1]
            if action.action_type is ActionType.Move:
                # print(temp_state.agents, file=sys.stderr, flush=True)
                #Move agent
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])
                # print(temp_state.agents, file=sys.stderr, flush=True)
                
                #Remove agent from old location
                self._remove_element(temp_state.agents, f'{agent_row},{agent_col}')
                # print(temp_state.agents, file=sys.stderr, flush=True)
            
            elif action.action_type is ActionType.Pull:
                box_id = temp_state.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'][0][2]
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''

                box_agent_match[box_id] = agent.agent_char
                #Move agent
                # print(temp_state.agents, file=sys.stderr, flush=True)
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])
                # print(temp_state.agents, file=sys.stderr, flush=True)
                #Remove agent from old location
                self._remove_element(temp_state.agents, f'{agent_row},{agent_col}')

                #Move Box
                temp_state.boxes[f'{agent_row},{agent_col}']\
                    .append(temp_state.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'][0])

                #Remove box from old location
                self._remove_element(temp_state.boxes,
                                     f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}')
        
            elif action.action_type is ActionType.Push:
                box_id = temp_state.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'][0][2]
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''
                box_agent_match[box_id] = agent.agent_char


                #Move Box
                temp_state.boxes[f'{agent_row+action.agent_dir.d_row+action.box_dir.d_row},{agent_col+action.agent_dir.d_col+action.box_dir.d_col}']\
                    .append(temp_state.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'][0])
                
                #Remove Box
                self._remove_element(temp_state.boxes,
                                     f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')
                #Move agent
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])
                #Remove agent from old location
                self._remove_element(temp_state.agents, f'{agent_row},{agent_col}')

        return [temp_state, box_agent_match]


    def _remove_element(self, list_elements, position):
        if len(list_elements[position]) > 1:
            list_elements[position].pop(0)
        else:
            list_elements.pop(position)
        
   
    def check_collisions(self, agents:list, agentDict):
        [temp_state,box_agent_match]  = self.temp_state_create(agents,agentDict)

        agent_collisions = []
        
        #Check Agents locations against other agents
        for loc, agt in temp_state.agents.items():
            if len(agt) > 1:
                agent_collisions.append(list(map(lambda x: x[1],agt)))

        #Check boxes against boxes
        for loc, box in temp_state.boxes.items():
            if len(box) > 1:
                #Sætter en box kollision i samme liste som en agent kollision (POTENTIEL ÆNDRING)
                temp = []
                for b in box:
                    temp.append(box_agent_match[b[2]])    
                agent_collisions.append(temp)

        #Check agents and boxes:
        for loc, agt in temp_state.agents.items():   
            temp = list(map(lambda x : x[1],agt))
            
            if len(temp_state.boxes[loc]) > 0:             
                for box in temp_state.boxes[loc]:
                    temp.append(box_agent_match[box[2]])
                agent_collisions.append(temp)
                
        return agent_collisions


    def fix_collisions(self, agents):

        agentDict = self.world_state.reverse_agent_dict()
        
        agent_illegal_moves = self.prereq_check(agents,agentDict)

        agent_collisions = self.check_collisions(agents,agentDict)
       
        if len(agent_collisions) == 0:
            joint_actions = [agent.plan.popleft() for agent in agents]
            return joint_actions, agent_illegal_moves

        '''I første omgang regner vi med ingen illegale moves,så denne vektor pille vi ikke ved. Det bliver altså så et spørgsmål 
        om at give NoOps tilfældigt til agenterne så deres collisions løses, og så returnere deres joint actions
        
        Løser NoOp assignment-problemet som et LP problem med pakken pulp'''

        #Conflicts = [[1,2],[4,5],[2,3,4]]    
        #agent_chars = ['a0','a1','a2','a3','a4','a5']

        agent_chars = []
        for agent in agents:
            agent_chars.append('a'+str(agent.agent_char))
        
        flat = []
        for elem in agent_collisions:
            for ind in elem:
                flat.append(ind)

        flatto = set(flat)

        conflict_agents = []

        for agt in agents:
            if agt.agent_char in flatto:
                conflict_agents.append(agt.agent_char)

        Lp_prob = LpProblem('Problem', LpMaximize)

        #define variables
        for agt in conflict_agents:
            agent_chars[agt] = LpVariable(agent_chars[agt], lowBound = 0,cat='Integer')

        #cost function
        Lp_prob += lpSum([1 * agent_chars[agt] for agt in conflict_agents])

        #constraints
        for elem in agent_collisions:
            Lp_prob += lpSum([1*agent_chars[agt] for agt in elem]) <= 1

        for agt in conflict_agents:
            Lp_prob += agt >= 0
        Lp_prob.solve()

        #save results to dictionary for quick indexing
        results = {}
        for v in Lp_prob.variables():
            results[int(v.name[-1:])] = v.varValue

        #Ready to push NoOps into plans for agents:
        for agent in agents:
            if agent.agent_char in results:
                if results[agent.agent_char] == 0:
                    agent.plan.appendleft(Action(ActionType.NoOp, None, None))
        
        joint_actions = [agent.plan.popleft() for agent in agents]
        return joint_actions, agent_illegal_moves
                    
                    

        










        
        






        


    




        


            
        
    









            





    
        

    





