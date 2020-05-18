
from state import State
from action import Action, ActionType, Dir
from collections import deque, defaultdict
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

        blackboard = [[0]*(len(self.world_state.agents)+len(self.world_state.boxes)),[0]*(len(self.world_state.agents)+len(self.world_state.boxes))]

        #Set current corrdinates
        for loc,agt in self.world_state.agents.items():
            row,col = loc.split(',')
            blackboard[0][agt[0][2]] = f'{row},{col}'

        for loc,box in self.world_state.boxes.items():
            row,col = loc.split(',')

            blackboard[0][box[0][2]+len_agents] = f'{row},{col}'

        #Set coordinates after 1 action
        for agt in agents: 
            agt_id = agt.agent_internal_id

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
            for idx, stationary in enumerate(stationary_boxes):
                if stationary:
                    blackboard[1][idx+len_agents] = blackboard[0][idx+len_agents] 
        return blackboard
        

    def blackboard_conflictSolver(self, agents:list):

        blackboard = self.create_blackboard(agents)
        
        len_agents = len(agents)
        
        #Check prereqs

        #List to skips idx's if their prereq conflict has already been solve
        skippo = []
        
        for idx, obj in enumerate(blackboard[1]):
            if idx in skippo:
                continue
            
            prereq_state = defaultdict(list)

            box_id = None

            #Define idx agent
            if idx < len_agents:
                agt = agents[idx]
            else:
                box_id = idx - len_agents
                _agt_list = [agt for agt in agents if agt.current_box_id == box_id]
                if len(_agt_list)>0:
                    print(f'idx: {idx}, _agt_list{_agt_list[0].agent_char}',file=sys.stderr,flush=True)
                    agt = _agt_list[0]
                else:
                    continue
                    # TODO: Require helperagents to have subgoal box value


            prereq_state[obj].append(idx)
            for p_idx,p_loc in enumerate(blackboard[0]):
                if p_idx != idx:
                    prereq_state[p_loc].append(p_idx)

            print(f'idx: {idx}, preqeq state{prereq_state}',file=sys.stderr,flush=True)
            for _,v in prereq_state.items():
                #If multiple indexes hash to same value, then we have a conflict
                if len(v) > 1:
                    
                    
                    skip = False
                    #If idx is box, make sure we don't fail on a pull move:
                    if idx >= len_agents:
                        for v_check in v:
                            if v_check  == agt.agent_internal_id:
                                skip = True
                    else:
                        #Fix push move for agents
                        for v_check in v:
                            if v_check-len_agents == agt.current_box_id:
                            #if v_check == agt.current_box_id-len_agents:    
                                print(f'skipping in idx {idx},vcheck {v_check}',file=sys.stderr,flush=True)
                                skip = True
                    

                    if skip:
                        continue
                    for v_id in v:
                        if v_id != idx:

                            #Extract locations of interest (from string)
                            row_0_v_id,col_0_v_id  = [int(x) for x in blackboard[0][v_id].split(',')]
                            row_1_v_id,col_1_v_id  = [int(x) for x in blackboard[1][v_id].split(',')]
                            row_0_idx,col_0_idx = [int(x) for x in blackboard[0][idx].split(',')]
                            #row_1_idx,col_1_idx = [int(x) for x in blackboard[1][idx].split(',')]


                            
                            #Check if stationary (have not moved from T-1  to T)
                            if blackboard[0][v_id] == blackboard[1][v_id]:

                                #Check WELL. 
                                if f'{row_0_v_id},{col_0_v_id}' in self.world_state.wells:
                                    
                                    if v_id < len_agents:
                                        #Stationary oject is an agent.

                                        #Ask agent to move out of idx_agt plan
                                        idx_plan = self._calculate_plan_coords(agt,blackboard[0][idx])
                                        possible = agents[v_id].search_conflict_bfs_not_in_list(self.world_state,idx, agt.current_box_id, agents[v_id].current_box_id, \
                                            coordinates = idx_plan)

                                        if possible:
                                            #***
                                            agents[v_id].plan_category = _cfg.solving_help_task
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))
                                            
                                        else:
                                            #Ask v_id to plan a path out of well
                                            agents[v_id].search_conflict_bfs_not_in_list(self.world_state,None,None,agents[v_id].current_box_id, \
                                                coordinates = self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]])

                                            #Solve precondition conflict
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                            #Then ask outer (idx) agent to move out of v_id's plan:
                                            #If idx agent hav a box, then only search with push and pull movements
                                            agt.search_conflict_bfs_not_in_list(self.world_state,v_id,agents[v_id].current_box_id,agt.current_box_id, \
                                                self._calculate_plan_coords(agents[v_id],blackboard[0][v_id]), move_action_allowed = False)

                                            agents[v_id].plan_category=_cfg.solving_help_task
                                            agt.plan_category= _cfg.awaiting_help
                                            agt.helper_id = (agents[v_id].agent_char, agents[v_id].agent_internal_id)
                                            
                                            #***
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))


                                    else:
                                        #idx hits a box, ask for help to move this box:
                                        helper_agt = self._determine_helper_agent(blackboard[0][v_id],blackboard,agents)
                                        
                                        #If no agent was found, just wait until someone becomes available
                                        if helper_agt is None:
                                            self.agent_amnesia(agt)
                                            #TODO: not final solution
                                            #agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                        else:
                                            #One agent has bee found as helper.
                                            
                                            #Helper agent now computes route to box, and then how to get the box out of the well
                                            helper_agt.plan_category = _cfg.solving_help_task
                                            helper_agt._reset_plan()
                                            #Search to location where our current agent/box is located
                                            helper_agt.search_to_box(self.world_state, blackboard[0][v_id],v_id-len_agents)
                                            helper_agt.plan
                                            
                                            #PUSH to agent: Search out of well with the box 
                                            helper_agt.goal_job_id = None
                                            helper_agt.pending_task_bool = True
                                            helper_agt.helper_agt_requester_id=agt.agent_char
                                            helper_agt.pending_task_func = helper_agt.search_conflict_bfs_not_in_list
                                            helper_agt.pending_task_dict = {'world_state': self.world_state,
                                                                            'agent_collision_internal_id': None,
                                                                            'agent_collision_box': None,
                                                                            'box_id' : self.world_state.boxes[blackboard[0][v_id]][0][2],
                                                                            'coordinates': self.world_state.wells_reverse[self.world_state.wells[blackboard[0][v_id]][0]],
                                                                            'move_action_allowed': False
                                                                            }
                                            #***
                                            helper_agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            print(f'************Well, box_id : {self.world_state.boxes[blackboard[0][v_id]][0][2]}',file=sys.stderr,flush=True)
                                            if helper_agt.agent_char != agt.agent_char:

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
                                                #***
                                                agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                                
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
                                        possible = agents[v_id].search_conflict_bfs_not_in_list(self.world_state,idx, agt.current_box_id, agents[v_id].current_box_id, \
                                            coordinates = idx_plan)

                                        if possible:
                                            #***
                                            agents[v_id].plan_category = _cfg.solving_help_task
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))
                                            
                                        else:
                                            #Ask v_id to plan a path out of tunnel
                                            agents[v_id].search_conflict_bfs_not_in_list(self.world_state,None,None,agents[v_id].current_box_id, \
                                                coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]][0]])

                                            #Solve precondition conflict
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                            #Then ask outer (idx) agent to move out of v_id's plan:
                                            #If idx agent hav a box, then only search with push and pull movements
                                            agt.search_conflict_bfs_not_in_list(self.world_state,v_id,agents[v_id].current_box_id,agt.current_box_id, \
                                                self._calculate_plan_coords(agents[v_id],blackboard[0][v_id]), move_action_allowed = False)

                                            #***
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agents[v_id].plan_category=_cfg.solving_help_task
                                            agt.plan_category= _cfg.awaiting_help
                                            agt.helper_id = (agents[v_id].agent_char, agents[v_id].agent_internal_id)

                                    else:
                                        #idx hits a box, ask for help to move this box:
                                        helper_agt = self._determine_helper_agent(blackboard[0][v_id],blackboard,agents)
                                        
                                        #If no agent was found, just wait until someone becomes available
                                        if helper_agt is None:
                                            self.agent_amnesia(agt)

                                            #TODO: not final
                                            #agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                        else:
                                            #One agent has been found as helper.

                                            #Helper agent now computes route to box, and then how to get the box out of the well
                                            helper_agt.plan_category = _cfg.solving_help_task
                                            helper_agt._reset_plan()
                                            #Search to location where our current agent/box is located
                                            helper_agt.search_to_box(self.world_state, blackboard[0][v_id],v_id-len_agents)

                                            #***
                                            helper_agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            
                                            #Search out of tunnel with the box 
                                            #TODO: PUSH TO AGENT
                                            helper_agt.goal_job_id = None
                                            helper_agt.pending_task_bool = True
                                            helper_agt.helper_agt_requester_id=agt.agent_char
                                            helper_agt.pending_task_func = helper_agt.search_conflict_bfs_not_in_list
                                            helper_agt.pending_task_dict = {'world_state': self.world_state,
                                                                            'agent_collision_internal_id': None,
                                                                            'agent_collision_box': None,
                                                                            'box_id': self.world_state.boxes[
                                                                                blackboard[0][v_id]][0][2],
                                                                            'coordinates':
                                                                                self.world_state.tunnels_reverse[
                                                                                    self.world_state.tunnels[
                                                                                        blackboard[0][v_id]]],
                                                                            'move_action_allowed': False
                                                                            }


                                            #Current agent now has to plan how to get out from well
                                            if helper_agt.agent_char != agt.agent_char:
                                                agt.pending_help = True
                                                agt.helper_id = (helper_agt.agent_char, helper_agt.agent_internal_id)
                                                agt.plan_category = _cfg.awaiting_help   
                                                if box_id is not None:
                                                    agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                            agent_collision_internal_id = None, \
                                                                agent_collision_box = None, \
                                                                    box_id = box_id, \
                                                                        coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]]], \
                                                                            move_action_allowed = False)
                                                
                                                else:
                                                    #If agt have not box, then search alone
                                                    agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                            agent_collision_internal_id = None, \
                                                                agent_collision_box = None, \
                                                                    box_id = None, \
                                                                        coordinates = self.world_state.tunnels_reverse[self.world_state.tunnels[blackboard[0][v_id]]])
                                                #***
                                                agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                    

                                #If in open space
                                else:
                                    #Try to move around. If no plan around object was found in X steps, we ask for the object to be moved. 
                                    #TODO: Vær sikker på at det virker så vi både kan sende en agent med og uden en boks

                                    #Find out if v-id os box or agent, and if collision agent has a box
                                    if v_id < len_agents:
                                        if agents[v_id].current_box_id is not None:
                                            blocked = [blackboard[0][v_id],blackboard[0][agents[v_id].current_box_id + len_agents]]
                                        else:
                                            blocked = [blackboard[0][v_id]]
                                    else:
                                        b_agt_list = [agt for agt in agents if agt.current_box_id == v_id-len_agents]
                                        if len(b_agt_list) > 0:
                                            b_agt_loc = blackboard[0][b_agt_list[0].agent_internal_id]
                                            blocked = [blackboard[0][v_id],b_agt_loc]
                                        else:
                                            blocked = [blackboard[0][v_id]]



                                    able_to_move = self.replanner.replan_v1(self.world_state,agt ,box_id,blocked)
                                    
                                    # TODO: Change states and categories to the appropiate values
                                    if not able_to_move:
                                        if v_id < len_agents:
                                            # Ask col agent to move out of plan
                                            bool_val = agents[v_id].search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = agt.agent_internal_id, \
                                                            agent_collision_box = box_id, \
                                                                box_id = agents[v_id].current_box_id, \
                                                                    coordinates = self._calculate_plan_coords(agt,blackboard[0][agt.agent_internal_id]))
                                            if not bool_val:
                                                raise NotImplementedError(f'Deadlock between agents, idx: {idx} and v_id: {v_id}')
                                                # Agent cannot move out of plan
                                            else:
                                                #If agents was able to move our of idx's plan, give 1 NoOp so we dont collide 
                                                #***
                                                agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                                agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                                 

                                        else:
                                            # Ask to have box removed from idx's plan if not assigned
                                            #idx hits a box, ask for help to move this box:
                                            
                                            helper_agt = self._determine_helper_agent(blackboard[0][v_id],blackboard,agents)

                                            
                                            
                                            #If no agent was found, just wait until someone becomes available
                                            if helper_agt is None:
                                                self.agent_amnesia(agt)

                                                #TODO: not final
                                                #agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            else:
                                                #One agent has been found as helper.

                                                #Helper agent now computes route to box, and then how to get the box out of the well
                                                helper_agt.plan_category = _cfg.solving_help_task

                                                plan_coords = self._calculate_plan_coords(agt,blackboard[0][agt.agent_internal_id])
                                                
                                                helper_agt._reset_plan()

                                                #TODO: søg på et andet punkt tæt på boksen
                                                #Search to location where our current agent/box is located
                                                helper_agt.search_to_box(self.world_state, blackboard[0][v_id],v_id-len_agents)
                                                #***
                                                helper_agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                                
                                                #Push next job to agent: Search out of well with the box
                                                helper_agt.goal_job_id = None
                                                helper_agt.pending_task_bool = True
                                                helper_agt.helper_agt_requester_id=agt.agent_char
                                                helper_agt.pending_task_func = helper_agt.search_conflict_bfs_not_in_list
                                                helper_agt.pending_task_dict = {'world_state': self.world_state,
                                                                                'agent_collision_internal_id': None,
                                                                                'agent_collision_box': None,
                                                                                'box_id': self.world_state.boxes[blackboard[0][v_id]][0][2],
                                                                                'coordinates': plan_coords,
                                                                                'move_action_allowed': False
                                                                                }
                                                
                                                

                                                #Current agent now gets NoOps in main until helping task is solved
                                                if helper_agt.agent_char != agt.agent_char:
                                                    #***
                                                    agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                                    agt.pending_help_pending_plan = True
                                                    agt.helper_id = (helper_agt.agent_char, helper_agt.agent_internal_id)
                                                    agt.plan_category = _cfg.awaiting_help   
                                                                                   
                            else:
                                #NOT STATIONARY
                                #Check if opposite directions of movement:
                                if f'{row_0_idx},{col_0_idx}' == f'{row_1_v_id},{col_1_v_id}':
                                    

                                    if v_id < len_agents:

                                        #If we collide with agent, this conflict is aldready solved so skip solving the conflcit with this agt
                                        skippo.append(v_id)
                                        
                                        #Two agents - find out who has the highest priority 
                                        if agt.plan_category >= agents[v_id].plan_category:
                                            
                                            if idx < len_agents:
                                                if agents[idx].current_box_id is not None:
                                                    blocked = [blackboard[0][idx],blackboard[0][agents[idx].current_box_id + len_agents]]
                                                else:
                                                    blocked = [blackboard[0][idx]]
                                            else:
                                                b_agt_list = [agt for agt in agents if agt.current_box_id == idx-len_agents]
                                                if len(b_agt_list)>0:
                                                    b_agt_loc = blackboard[0][_agt_list[0].agent_internal_id]

                                                    blocked = [blackboard[0][v_id],b_agt_loc]
                                                else:
                                                    blocked = [blackboard[0][v_id]]


                                            able_to_move = self.replanner.replan_v1(self.world_state,agents[v_id],None,blocked)

                                            if not able_to_move:
                                                bool_val = agents[v_id].search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = agt.agent_internal_id, \
                                                            agent_collision_box = box_id, \
                                                                box_id = agents[v_id].agent_internal_id, \
                                                                    coordinates = self._calculate_plan_coords(agt,blackboard[0][agt.agent_internal_id]),
                                                                    move_action_allowed = False)
                                                agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))

                                                if not bool_val:
                                                    raise NotImplementedError(f'Agent {agents[v_id].agent_char} cannot move around or get away from {idx}')

                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                        else:

                                            
                                            if agents[v_id].current_box_id is not None:
                                                blocked = [blackboard[0][v_id],blackboard[0][agents[v_id].current_box_id + len_agents]]
                                            else:
                                                blocked = [blackboard[0][v_id]]
                                            
                                            able_to_move = self.replanner.replan_v1(self.world_state,agt,None,blocked)

                                            if not able_to_move:
                                                bool_val = agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = agents[v_id].agent_internal_id, \
                                                            agent_collision_box = agents[v_id].current_box_id, \
                                                                box_id = agt.current_box_id, \
                                                                    coordinates = self._calculate_plan_coords(agents[v_id],blackboard[0][v_id]))
                                                agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                                if not bool_val:
                                                    raise NotImplementedError(f'Agent {agents[v_id].agent_char} cannot move around or get away from {idx}')

                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agents[v_id].plan.appendleft(Action(ActionType.NoOp, None, None))
                                    
                                    else:
                                        

                                        #If moving box, find relevant agent
                                        box_agt = [agt for agt in agents if agt.current_box_id == v_id][0]

                                        #Skip this agtens prereq collision in next idx iteration
                                        skippo.append(box_agt.agent_internal_id)

                                        #Two agents - find out who has the highest priority 
                                        if agt.plan_category >= box_agt.plan_category:

                                            if idx < len_agents:
                                                if agents[idx].current_box_id is not None:
                                                    blocked = [blackboard[0][idx],blackboard[0][agents[idx].current_box_id + len_agents]]
                                                else:
                                                    blocked = [blackboard[0][idx]]
                                            else:
                                                b_agt_list = [agt for agt in agents if agt.current_box_id == idx-len_agents]
                                                if len(b_agt_list)>0:
                                                    b_agt_loc = blackboard[0][_agt_list[0].agent_internal_id]

                                                    blocked = [blackboard[0][v_id],b_agt_loc]
                                                else:
                                                    blocked = [blackboard[0][v_id]]
                                            


                                            able_to_move = self.replanner.replan_v1(self.world_state,box_agt,None,blocked)

                                            if not able_to_move:
                                                bool_val = box_agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = agt.agent_internal_id, \
                                                            agent_collision_box = agt.current_box_id, \
                                                                box_id = agents[v_id].current_box_id, \
                                                                    coordinates = self._calculate_plan_coords(agt,blackboard[0][agt.agent_internal_id]))
                                                box_agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                                if not bool_val:
                                                    raise NotImplementedError(f'Agent {box_agt.agentagent_char} cannot move around or get away from {idx}')

                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                        else:
                                            
                                            
                                            blocked = [blackboard[0][v_id],blackboard[0][box_agt.agent_internal_id]]

                        


                                            able_to_move = self.replanner.replan_v1(self.world_state,agt,None,blocked)

                                            if not able_to_move:
                                                bool_val = agt.search_conflict_bfs_not_in_list(world_state = self.world_state, \
                                                        agent_collision_internal_id = box_agt.agent_internal_id, \
                                                            agent_collision_box = v_id, \
                                                                box_id = agt.current_box_id, \
                                                                    coordinates = self._calculate_plan_coords(box_agt,blackboard[0][box_agt.agent_internal_id]))
                                                agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                                if not bool_val:
                                                    raise NotImplementedError(f'Agent {box_agt.agentagent_char} cannot move around or get away from {idx}')

                                            box_agt.plan.appendleft(Action(ActionType.NoOp, None, None))
                                            box_agt.plan.appendleft(Action(ActionType.NoOp, None, None))

                                else:
                                    #Wait for prereq location to be free
                                    agt.plan.appendleft(Action(ActionType.NoOp, None, None))
        print(f'++++++ Done wiht iter for idx:',file=sys.stderr,flush=True)                   
        #Update blackboard after new actions are pushed that fixes prereqs
        blackboard = self.create_blackboard(agents)


        #Check actual collisions
        current_state = defaultdict(list)
        for idx, obj in enumerate(blackboard[1]):
            current_state[blackboard[1][idx]].append(idx)
        
        print(f'Conflict_manager effects {current_state}',file=sys.stderr,flush=True)
        for loc ,v in current_state.items():
            if len(v) > 1:

                '''
                For collisions we prioritize the agent with the most important job
                '''
                #Find all agents from idx's:

                agt_list = []
                for idc in v:
                    if idc < len_agents:
                        agt_list.append(agents[idc])
                    else:
                        _agt_list = [agt for agt in agents if agt.current_box_id == box_id]
                        if len(_agt_list) > 0:
                            box_agt = _agt_list[0]
                        else:
                            continue
                        agt_list.append(box_agt)
                
                prez = -1
                prez_category = -10
                for p_id, ag in enumerate(agt_list):
                    if ag.plan_category > prez_category:
                        prez_category = ag.plan_category
                        prez = p_id


                for idc, ag in enumerate(agt_list):
                    if idc != prez:
                        ag.plan.appendleft(Action(ActionType.NoOp, None, None))


                
                #if (f'{loc_row},{loc_col}' in self.world_state.wells) or (f'{loc_row},{loc_col}' in self.world_state.tunnel):
                #else:
                    #In open space 
                       
    def agent_amnesia(self,agent):
        print(f'AMNESIA, char =  {agent.agent_char}',file=sys.stderr,flush=True)
        agent._reset_from_help()
        agent._reset_plan()
        agent.goal_job_id = None
                
    def _calculate_plan_coords(self,agent,current_loc):

        #TODO: Test denne her
        coordinate_list = []
        row,col = [int(x) for x in current_loc.split(',')]
        coordinate_list.append(f'{row},{col}')
        for action in agent.plan:
            
            if action.action_type is ActionType.NoOp:
                coordinate_list.append(f'{row},{col}')
                new_agt_row = row
                new_agt_col = col  
                    
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
        print(f'coordlist: {set(coordinate_list)}',file=sys.stderr,flush=True)
        return(set(coordinate_list))



    def _determine_helper_agent(self,box_loc,blackboard,agents):

        min_dist = inf
        helper_agt = None
        box_color = self.world_state.boxes[box_loc][0][0]
        box_loc_id=self.world_state.boxes[box_loc][0][2]
        box_component =self.world_state.boxes[box_loc][0][3]
        
        # Temp creation to identify if an agents has this box assigned 
        x=[(agt.current_box_id, agt) for agt in agents if agt.current_box_id==box_loc_id]

        print(f':::::box_lox_id = {box_loc_id}, x = {x}',file=sys.stderr,flush=True)
        
        # Case where box is assigned
        if len(x)>0:
            
            return x[0][1]
        
        
        else:
            
            # TODO: Overvej om det giver mening helper_agt kan være agenten selv som beder om hjælp 
            #Box was not assigned to  
            for agt in [agt for agt in agents if (agt.agent_color==box_color) and (agt.connected_component_id == box_component)]:
                print(f'-------agtchar : {agt.agent_char}, agt_connected component : {agt.connected_component_id}, box_component {box_component}',file=sys.stderr,flush=True)
                dist = utils.cityblock_distance(box_loc,blackboard[0][agt.agent_internal_id])
                

                # only get help from an agent that's not assigned
                if  (dist < min_dist) and (agt.plan_category!=_cfg.solving_help_task):
                    min_dist = dist
                    helper_agt = agt  
            helper_agt.current_box_id = box_loc_id
            print(f'XXXXXXXXXXXX Helper_agt_id {helper_agt.current_box_id}',file=sys.stderr,flush=True)
            return helper_agt


