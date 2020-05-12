
from state import State
from action import Action, ActionType, Dir
from collections import deque, defaultdict
from pulp import *
import sys
import utils
import config
import math
from replanner import Replanner
import agent
import utils
import config as _cfg

from math import inf

#Squarebrackets på alle kald til dicts


class ConflictManager:

    def __init__(self, agents:list):

        '''
        Percept the world with current state
        '''
        self.world_state = None
        
        
        self.stationary_agents = []
        self.stationary_boxes = []
        self.shortest_plan_agt = None #Agent with shortest plan

        self.replanner = Replanner()


        #self._create_blackboard()
       


    def create_blackboard(self,agents:list):

        #ASSUME WORLDSTATE IS UPDATED HERE 

        
        len_agents = len(agents)
        stationary_boxes = [True]*len(self.world_state.boxes)

        blackboard = [[],[]]

        #Set current corrdinates
        for loc,agt in self.world_state.agents.items():
            row,col = loc.split(',')

            blackboard[0][agt[0][2]] = f'{row},{col}'

        for loc,box in self.world_state.boxes.items():
            row,col = loc.split(',')

            blackboard[0][box[0][2]+len_agents] = f'{row},{col}'

        
        #Set coordinates after 1 action
        for agt in agents: 
            agt_id = agt[0][2]

            try:
                action = agt.plan[0]
            except:
                blackboard[1][agt_id] = blackboard[0][agt_id]
                continue
    
            if action.action_type is ActionType.NoOp:
                    blackboard[1][agt_id] = blackboard[0][agt_id] 
                    
            elif action.action_type is ActionType.Move:
                agt_row, agt_col = [int(x) for x in blackboard[0][agt_id].split(',')]
                
                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                blackboard[1][agt_id] =f'{new_agt_row},{new_agt_col}'
                
            elif action.action_type is ActionType.Push:
                agt_row, agt_col = [int(x) for x in blackboard[0][agt_id].split(',')]
                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                box_row = new_agt_row 
                box_col = new_agt_col 
                
                new_box_row = box_row + action.box_dir.d_row
                new_box_col = box_col + action.box_dir.d_col

                box_id =  self.world_state.boxes[f'{box_row},{box_col}'][0][2]


                blackboard[1][agt_id] = f'{new_agt_row},{new_agt_col}'
                blackboard[1][box_id+len_agents] = f'{new_box_row},{new_box_col}'

                stationary_boxes[box_id] = False

            elif action.action_type is ActionType.Pull:
                agt_row, agt_col = [int(x) for x in blackboard[0][agt_id].split(',')]
                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                box_row = agt_row +  action.box_dir.d_row
                box_col = agt_col +  action.box_dir.d_col
                
                new_box_row = agt_row 
                new_box_col = agt_col

                
                box_id = self.world_state.boxes[f'{box_row},{box_col}'][0][2]
                
                blackboard[1][agt_id] = f'{new_agt_row},{new_agt_col}'
                blackboard[1][box_id+len_agents] = f'{new_box_row},{new_box_col}'

                stationary_boxes[box_id] = False

            #Update coordinates for stationary boxes
            for idx, box in enumerate(stationary_boxes):
                if stationary:
                    blackboard[1][idx+len_agents] = blackboard[0][idx+len_agents] 
        return blackboard
        

    def _blackboard_conflictSolver(self, agents:list):

        blackboard = self.create_blackboard(agents)

        len_agents = len(agents)
        
        #Check prereqs
        for idx, obj in enumerate(blackboard[1]):
            
            prereq_state = defaultdict(list)

            box_id = None

            #Define idx agent
            if idx < len_agents:
                agt = agents[idx]
            else:
                box_id = idx - len_agents
                agt = [agt for agt in agents if agt.sub_goal_box == box_id][0]


            prereq_state[loc].append(idx)
            for p_idx,p_loc in enumerate(blackboard[0]):
                if p_idx != idx:
                    prereq_state[p_loc].append(p_idx)

            for _,v in prereq_state.items():
                #If multiple indexes hash to same value, then we have a conflict
                if len(v) > 1:
                    for v_id in v:
                        if v_id != idx:

                            #Extract locations of interest (from string)
                            row_0_v_id,col_0_v_id  = [int(x) for x in blackboard[0][v_id].split(',')]
                            row_1_v_id,col_1_v_id  = [int(x) for x in blackboard[1][v_id].split(',')]
                            row_0_idx,col_0_idx = [int(x) for x in blackboard[0][idx].split(',')]
                            row_1_idx,col_1_idx = [int(x) for x in blackboard[1][idx].split(',')]


                            
                            #Check if stationary (have not moved from T-1  to T)
                            if blackboard[0][v_id] == blackboard[1][v_id]:

                                #Check WELL. 
                                if f'{row_0_v_id},{col_0_v_id}' in self.world_state.wells:
                                    
                                    if v_id < len_agents:
                                        #Stationary oject is an agent.

                                        #Ask agent to move out of idx_agt plan
                                        idx_plan = self._calculate_plan_coords(agt,blackboard[0][idx])
                                        possible = agents[v_id].search_conflict_bfs_not_in_list(self.world_state,idx, agt.current_box_id, None, \
                                            coordinates = idx_plan)

                                        if possible:
                                            agents[v_id].plan_category = _cfg.solving_help_task
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            
                                        else:
                                            #Ask v_id to plan a path out of well
                                            agents[v_id].search_conflict_bfs_not_in_list(self.world_state,None,None,None, \
                                                coordinated = self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]])

                                            #Solve precondition conflict
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                            #Then ask outer (idx) agent to move out of v_id's plan:
                                            #If idx agent hav a box, then only search with push and pull movements
                                            agt.search_conflict_bfs_not_in_list(self.world_state,v_id,None,agt.current_box_id, \
                                                self._calculate_plan_coords(agents[v_id],blackboard[0][v_id]), move_action_allowed = False)

                                            agents[v_id].plan_category=_cfg.solving_help_task
                                            agt.plan_category= _cfg.awaiting_help
                                            agt.helper_id = (agents[v_id].agent_char, agents[v_id].agent_internal_id)


                                    else:
                                        #idx hits a box, ask for help to move this box:
                                        helper_agt = self._determine_helper_agent(blackboard[0][v_id],blackboard,agents)
                                        
                                        #If no agent was found, just wait until someone becomes available
                                        if helper_agt is None:
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                        else:
                                            #One agent has bee found as helper.

                                            #Helper agent now computes route to box, and then how to get the box out of the well
                                            helper_agt.plan_category = _cfg.solving_help_task
                                            helper_agt._reset_plan()
                                            #Search to location where our current agent/box is located
                                            helper_agt.search_to_box(self.world_state, blackboard[0][idx],box_id)
                                            
                                            #Search out of well with the box 
                                            helper_agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = self.world_state.boxes[blackboard[0][v_id]][2], \
                                                                    coordinates = self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]],\
                                                                        move_action_allowed = False)


                                            #Current agent now has to plan how to get out from well
                                            agt.pending_help = True
                                            agt.helper_id = (helper_agt.agent_char, helper_agt.agent_internal_id)
                                            agt.plan_category = _cfg.awaiting_help   
                                            if box_id is not None:
                                                agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = box_id, \
                                                                    coordinates = self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]], \
                                                                        move_action_allowed = False)
                                            
                                            else:
                                                #If agt have not box, then search alone
                                                agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = None, \
                                                                    coordinates = self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]])
                                            
                                            #TODO: Skal måske tilføjes igen:
                                            '''
                                            #Push as many NoOps as the helper_agt will take to solve the current help-plan
                                            for _ in range(len(helper_agt.plan)):
                                                agt.plan.append(Action(ActionType.NoOp, None, None))
                                            '''    

                                #Check TUNNEL
                                elif f'{row_0_v_id},{col_0_v_id}' in self.world_state.tunnels:
                                    #TODO: Nu er dette det samme som sker i en brønd. Overvej om der er bedre måder at udnytte vores preprocessing på
                                    if v_id < len_agents:
                                        #Stationary oject is an agent.

                                        #Ask agent to move out of idx_agt plan
                                        idx_plan = self._calculate_plan_coords(agt,blackboard[0][idx])
                                        possible = agents[v_id].search_conflict_bfs_not_in_list(self.world_state,idx, agt.current_box_id, None, \
                                            coordinates = idx_plan)

                                        if possible:
                                            agents[v_id].plan_category = _cfg.solving_help_task
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            
                                        else:
                                            #Ask v_id to plan a path out of well
                                            agents[v_id].search_conflict_bfs_not_in_list(self.world_state,None,None,None, \
                                                coordinated = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]][0]])

                                            #Solve precondition conflict
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                            #Then ask outer (idx) agent to move out of v_id's plan:
                                            #If idx agent hav a box, then only search with push and pull movements
                                            agt.search_conflict_bfs_not_in_list(self.world_state,v_id,None,agt.current_box_id, \
                                                self._calculate_plan_coords(agents[v_id],blackboard[0][v_id]), move_action_allowed = False)

                                            agents[v_id].plan_category=_cfg.solving_help_task
                                            agt.plan_category= _cfg.awaiting_help
                                            agt.helper_id = (agents[v_id].agent_char, agents[v_id].agent_internal_id)

                                    else:
                                        #idx hits a box, ask for help to move this box:
                                        helper_agt = self._determine_helper_agent(blackboard[0][v_id],blackboard,agents)
                                        
                                        #If no agent was found, just wait until someone becomes available
                                        if helper_agt is None:
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                        else:
                                            #One agent has bee found as helper.

                                            #Helper agent now computes route to box, and then how to get the box out of the well
                                            helper_agt.plan_category = _cfg.solving_help_task
                                            helper_agt._reset_plan()
                                            #Search to location where our current agent/box is located
                                            helper_agt.search_to_box(self.world_state, blackboard[0][idx],box_id)
                                            
                                            #Search out of well with the box 
                                            helper_agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = self.world_state.boxes[blackboard[0][v_id]][2], \
                                                                    coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]][0]],\
                                                                        move_action_allowed = False)


                                            #Current agent now has to plan how to get out from well
                                            agt.pending_help = True
                                            agt.helper_id = (helper_agt.agent_char, helper_agt.agent_internal_id)
                                            agt.plan_category = _cfg.awaiting_help   
                                            if box_id is not None:
                                                agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = box_id, \
                                                                    coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]][0]], \
                                                                        move_action_allowed = False)
                                            
                                            else:
                                                #If agt have not box, then search alone
                                                agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = None, \
                                                            agent_collision_box = None, \
                                                                box_id = None, \
                                                                    coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]][0]])
                                 

                                #If in open space
                                else:
                                    #Try to move around. If no plan around object was found in X steps, we ask for the object to be moved. 
                                    #TODO: Vær sikker på at det virker så vi både kan sende en agent med og uden en boks
                                    able_to_move = self.replanner.replan_v1(self.world_state,[agt],box_id,[blackboard[0][v_id]])
                                    
                                    if not able_to_move:
                                        if v_id < len_agents:

                                            if box_id is not None:
                                                #Ask for agent to move out of idx's plan:
                                                agents[v_id].search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                            agent_collision_internal_id = idx, \
                                                                agent_collision_box = box_id, \
                                                                    box_id = self.world_state.boxes[blackboard[0][v_id]][2], \
                                                                        coordinates = self._calculate_plan_coords(agt,blackboard[0][idx]))
                                        else:
                                            #Ask to have box removed from idx's plan






                                    
                                    




                                    if idx < len_agents:




                                    '''
                                    if idx < len_agents:
                                        agents[idx].REPLAN AROUND

                                    else:
                                        box_id = idx-len_agents
                                        agt  = [agt for agt in agents if agt.sub_goal_box == box_id][0]
                                        agt.REPLAN AROUND
                                    '''
                            else:
                                #NOT STATIONARY

                                '''
                                TODO: USE JOB Importance to determine who to move 
                                '''

                                #Check if opposite directions of movement:
                                if f'{row_0_idx},{col_0_idx}' == f'{row_1_v_id},{col_1_v_id}':
                                    '''
                                    EVENTUELT: 
                                        if in tunnel/well: 
                                            løs anderledes
                                    
                                    1 søger væk fra nr2's plan
                                    nr2 fortsætter sin gamle plan
                                    
                                    '''

                                else:
                                    '''
                                    agt.plan.push(NoOp)
                                    '''

                                #if blackboard[1][idx] not in Blackboard[2]:
                                #    '''
                                #    agt.plan.push(NoOp)
                                #    agt.plan.push(NoOp) 
                                #    '''
                                #else:
                                #    #Bude ikke kunne ske?
                                #    '''
                                #    agt.REPLAN UDENOM
                                #    '''
                               
        #Update blackboard after new actions are pushed that fixes prereqs
        blackboard = self.blackboard_update(agents)


        #Check actual collisions
        current_state = defaultdict(list)
        for idx, obj in enumerate(blackboard[1]):
            current_state[blackboard[1][idx]].append(idx)
        
        for loc,v in current_state.items():
            if len(v) > 1:
                '''
                For collisions we prioritize the agent with the most important job
                '''

                #In region of interest?
                loc_row,loc_col = blackboard[1][loc].split(',')

                if f'{loc_row},{loc_col}' in self.world_state.wells:
                    pass
                elif f'{loc_row},{loc_col}' in self.world_state.tunnel:
                    pass
                elif f'{loc_row},{loc_col}' in self.world_state.juntions:
                    pass
                else:
                    #In open space 
                    pass
                    



                
    def _calculate_plan_coords(self,agent,current_loc):

        #TODO: Test denne her
        coordinate_list = []
        row,col = [int(x) for x in current_loc.split(',')]

        for action in agt.plan:
            
        
            if action.action_type is ActionType.NoOp:
                    coordinate_list.append(f'{row},{col}')  
                    
            elif action.action_type is ActionType.Move:
                agt_row, agt_col = row, col

                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                coordinate_list.append(f'{new_agt_row},{new_agt_col}')
                
            elif action.action_type is ActionType.Push:
                agt_row, agt_col = row, col
                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                box_row = new_agt_row 
                box_col = new_agt_col 
                
                new_box_row = box_row + action.box_dir.d_row
                new_box_col = box_col + action.box_dir.d_col



                coordinate_list.append(f'{new_agt_row},{new_agt_col}')
                coordinate_list.append(f'{new_box_row},{new_box_col}')

                
            elif action.action_type is ActionType.Pull:
                agt_row, agt_col = row, col
                new_agt_row = agt_row + action.agent_dir.d_row
                new_agt_col = agt_col + action.agent_dir.d_col

                box_row = agt_row +  action.box_dir.d_row
                box_col = agt_col +  action.box_dir.d_col
                
                new_box_row = agt_row 
                new_box_col = agt_col

                
                coordinate_list.append(f'{new_agt_row},{new_agt_col}')
                coordinate_list.append(f'{new_box_row},{new_box_col}')
            #Update agent current location
            row,col = new_agt_row,new_agt_col
        return(set(coordinate_list))



    def _determine_helper_agent(self,box_loc,blackboard,agents):

        min_dist = inf
        helper_agt = None
        box_color = self.world_state.boxes[box_loc][0]
        box_loc_id=self.world_state.boxes[box_loc][0][2]
        
        # Temp creation to identify if an agents has this box assigned 
        x=[(agt.current_box_id, agt) for agt in agents if agents.current_box_id==box_loc_id]
        # Case where box is assigned
        if len(x)>0:
            return x[0][1]
        
        
        else:
            # TODO: Overvej om det giver mening helper_agt kan være agenten selv som beder om hjælp 
            #Box was not assigned to  
            for agt in [agt for agt in agents if agt.agent_color==box_color]:

                dist = utils.cityblock_distance(box_loc,blackboard[0][agt.agent_internal_id])

                # only get help from an agent that's not assigned
                if  (dist < min_dist) and (agt.plan_category!=6):
                    min_dist = dist
                    helter_agt = agt  
            return helper_agt


