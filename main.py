import argparse
import re
import sys
import memory
import config
from conflictManager import ConflictManager
from searchclient import SearchClient
from agent import search_agent
from strategy import StrategyBFS, StrategyDFS, StrategyBestFirst
from goalassignment import *
from state import State
from copy import deepcopy as cp
from preprocessing import create_dijkstras_map
from action import MOVE_ACTIONS
from random import shuffle

def main():


    # Read server messages from stdin.
    server_messages = sys.stdin

    # Use stderr to print to console through server.
    print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr, flush=True)

    # Initialize level by sending client name
    ClientName= f"{config.client_name}\r".encode("ascii")
    sys.stdout.buffer.write(ClientName)
    sys.stdout.flush()

    # TO DO: Figure out if we have an multiagent or single agent level
    single_agent=False

    if single_agent:
        raise Exception("Not implemented")

    # Read level and create the initial state of the problem.
    client = SearchClient(server_messages)

    '''
    Testing assigner 
    '''
    preprocessing_current_state = State(client.initial_state)

    '''
    Create Dijkstras mapping for all goal location
    
    For all goal_locations do
        Create BFS From location and 
    
    '''
    current_state = State(client.initial_state)
    current_state.dijkstras_map = create_dijkstras_map(preprocessing_current_state)

    print(current_state.agents,file=sys.stderr,flush=True)

    # Create list of agents
    list_agents = []
    del_agents = []
    for k, v in current_state.reverse_agent_dict().items():
        print(f'{k},{v}',file=sys.stderr, flush=True)
        if k not in client.del_agents_ids:
            # Char int, color, connectedcomp, strategy
            list_agents.append(search_agent(k, v[0], v[3], v[2], StrategyBestFirst))
        else:
            del_agents.append(search_agent(k, v[0], v[3], v[2], StrategyBestFirst))
    
    del_locs = []
    for del_id in client.del_agents_ids:
        for loc, agt in current_state.agents.items():
            if agt[0][1] == del_id:
                del_locs.append(loc)

    for loc in del_locs:
        del current_state.agents[loc]
    
    list_agents_full=list_agents+del_agents
    list_agents_full.sort()
    list_agents.sort()

    #Creating internal id's
    a_inter_id = 0
    for agt in list_agents:
        agt.agent_internal_id = a_inter_id
        a_inter_id+=1
    
    for k,v in current_state.agents.items():
        for agt in list_agents:
            if v[0][1]==agt.agent_char:
                current_state.agents[k][0][2] = agt.agent_internal_id

    

    goal_assigner = GoalAssigner(current_state, goal_dependencies=client.goal_dependencies, list_of_agents=list_agents)
    goal_assigner.reassign_tasks()

    for agt in list_agents:
        print(f'agt_char = {agt.agent_char}, inter_id= {agt.agent_internal_id}, box_assign = {agt.current_box_id}',file=sys.stderr,flush=True)

    conflict_manager = ConflictManager(list_agents)
    
    #Fixing conflict in initial plans
    conflict_manager.world_state = current_state
    conflict_manager.blackboard_conflictSolver(list_agents)

    
    #Agent randomness execution variables
    agent_prev_category = [-1]*len(list_agents_full)
    agent_noop_counter = [0]*len(list_agents_full)
    include_categories = [config.goal_assigner_box,config.goal_assigner_location,config.self_helping]
    

    # Whileloop
    counter = 0
    while True:
        '''
        Process of control loop

        1. Identify if any agents is receiving help and should No-Op this iter
        2. Reassign the agents not having a task
            This also handels if an agent is doing a twofold assignemt: move to box -> move box to goal
        3. Solve the conflicts in the prereq and effects of the agents actions
        4. Extract actions and merge irellevant agents
        5. Check if our offline planning is succesfull - if not then shit
        6. Check if we have reached the goal sate
        7. Update the world states in the different elements

        '''
        print(f'-----------------------{counter}-----------------------',file=sys.stderr,flush=True)



        # print(f"WHILE ENTER {counter}\n", file=sys.stderr, flush=True)
        # print(f"{current_state.boxes} boxes", file=sys.stderr, flush=True)
        # print(f"{current_state.agents} agents", file=sys.stderr, flush=True)

        
        # TODO: Check subgoal for agents - does it need to be updated

        # Keeps it from getting out of hand while testing
        if counter == config.while_counter:
            break

        
        ''' Deprecated
        # All agents helping
        
        _helper_agents = [x for x in list_agents if x.plan_category==config.solving_help_task] 
        
        #All agents reciving help and have a plan left in the bag
        _agents_reciving_help = [x for x in list_agents if (x.plan_category == config.awaiting_help) and (x.pending_help_pending_plan)]

        # If in helping state noop untill tasks solved
        for x in _agents_reciving_help:
            _awaiting_done=True
            for y in _helper_agents:
                
                if  (y.helper_agt_requester_id == x.agent_char) and ((len(y.plan) > 0) or y.pending_task_bool):
                    x.plan.appendleft(Action(ActionType.NoOp, None, None))
                    _awaiting_done=False
                    print('eeeeeeeenter', file=sys.stderr,flush=True)
                    break
            if _awaiting_done:
                print('enter', file=sys.stderr,flush=True)
                #x._resume_plan()
                ConflictManager.agent_amnesia(x)
        '''

         # Solve the new colflicts
        for e in list_agents:
            if len(e.plan) > 0:
                # print(f'{e.agent_char} plan:{e.plan}', flush=True, file=sys.stderr)
                print(f'{e.plan[0]} {e.agent_char} before conflict length: {len(e.plan)} category:{e.plan_category}', file=sys.stderr, flush=True)
            else:
                print(f'NoPlan for {e.agent_char} before conflict length: {len(e.plan)} category:{e.plan_category}', file=sys.stderr, flush=True)

        # Give task to unassigned agents
        goal_assigner.reassign_tasks()
        
        for e in list_agents:
            if len(e.plan) > 0:
                # print(f'{e.agent_char} plan:{e.plan}', flush=True, file=sys.stderr)
                print(f'>>{e.plan[0]} {e.agent_char} before conflict length: {len(e.plan)} category:{e.plan_category}', file=sys.stderr, flush=True)
            else:
                print(f'>>NoPlan for {e.agent_char} before conflict length: {len(e.plan)} category:{e.plan_category}', file=sys.stderr, flush=True)
       


        print(f'############## pre BOX_IDS {[(agt.agent_char,agt.current_box_id) for agt in list_agents]}',file=sys.stderr,flush=True)

        conflict_manager.blackboard_conflictSolver(list_agents)
        print(f'############## post BOX_IDS {[(agt.agent_char,agt.current_box_id) for agt in list_agents]}',file=sys.stderr,flush=True)

        print(f'!-!-!-! {[(x.agent_char,x.helper_agt_requester_id, x.helper_id) for x in list_agents]}',file=sys.stderr,flush=True)


        # Improve speed - find the agetns that are currently executing help tasks
        _helper_agents_ = [y for y in list_agents if y.helper_agt_requester_id is not None]

        # Helper agents awaiting help
        # TODO: Check up on nested help correctness 
        for x in list_agents:
            _awaiting_done=True
            if x.plan_category==config.solving_help_task and x.nested_help:
                for y in _helper_agents_:
                    if x.helper_id[0]==y.helper_agt_requester_id and x.agent_char!=y.agent_char:
                        x.plan.appendleft(Action(ActionType.NoOp, None, None))
                        _awaiting_done=False
                if _awaiting_done:
                    x.nested_help=False

        for e in list_agents:
            if len(e.plan) > 0:
                print(f'{e.plan[0]} {e.agent_char} after conflict ', file=sys.stderr, flush=True)
            else:
                print(f'NoPlan for {e.agent_char} after conflict', file=sys.stderr, flush=True)

        print(f'- world_state agents before actions:  {current_state.agents}',file=sys.stderr,flush=True)
        print(f'- world_state boxes before actions:  {current_state.boxes}',file=sys.stderr,flush=True)
    

        #TODO: Sync here with deleted agents and get their latest actions

        # pop latest action from the full sorted list of actions
        # This functions gives a noop if it does not have a plan
        list_of_actions = [x.get_next_action() for x in list_agents_full]

        
        if config.initiate_random_agents:
            #Increase counters if agent have been stalling  
            for idx, act in enumerate(list_of_actions):
                if act.action_type == ActionType.NoOp: 
                    if (list_agents_full[idx].plan_category in include_categories) and (list_agents_full[idx].plan_category == agent_prev_category[idx]):
                        agent_noop_counter[idx]+=1
                else:
                    agent_noop_counter[idx] = 0


            if max(agent_noop_counter)>config.agent_max_stall:
                #Create possible future state only if any value is actually above threshold
                temp_state = State(current_state)
                temp_state.world_state_update(list_of_actions,client.del_agents_ids)
                agentDict = current_state.reverse_agent_dict()
                
                #list of temp coordinates that are blocked along the way 
                temp_blocked = []

                #Fix random actions
                for idx, num in enumerate(agent_noop_counter):
                    
                    if num > config.agent_max_stall:
                        #If agent have been stalling too long, push random action
                        applicable_action = None
                        found_action = False

                        shuffle(MOVE_ACTIONS)
                        
                        for action in MOVE_ACTIONS:
                            #get location
                            agt_row, agt_col = [int(x) for x in agentDict[idx][1].split(',')]

                            new_agt_row = agt_row + action.agent_dir.d_row
                            new_agt_col = agt_col + action.agent_dir.d_col

                            loc_string = f'{new_agt_row},{new_agt_col}'

                            if current_state.random_is_free(loc_string) \
                                and temp_state.random_is_free(loc_string) \
                                    and (loc_string not in temp_blocked):
                                applicable_action = action
                                found_action = True
                                
                                break
                        if found_action:
                            #Ask agent to forget everything, but and then push the applicable action
                            agent_noop_counter[idx] = 0
                            list_agents_full[idx].agent_amnesia()
                            
                            list_of_actions[idx] = applicable_action

                            #"Apply" action
                            temp_blocked.append(loc_string)


        # push to server -> list of actions
        my_string = ';'.join(list(str(x) for x in list_of_actions))
        print(my_string, flush=True)

        # getting the data from the client to asses validity of actions
        response = server_messages.readline().rstrip()
        print(response, file=sys.stderr, flush=True)
        for resp in response.split(";"):
            if resp == 'false':
                raise Exception("[FATAL ERROR] received false response from server")

        current_state.world_state_update(list_of_actions,client.del_agents_ids)

        print(f'- World_satate after actions: {current_state.agents}',file=sys.stderr,flush=True)
        print(f'- world_state boxes after actions:  {current_state.boxes}',file=sys.stderr,flush=True)


        if current_state.world_is_goal_state():
            print("Done", file=sys.stderr, flush=True)
            raise Exception('SHOULD BE DONE - but something went wrong')
            break
        
        # Update the states of goalassigner and conflictmanager
        conflict_manager.world_state = State(current_state)
        GoalAssigner.world_state = State(current_state)

        #Randomizer updating - keeping track in next iteration of the type of category 
        for ct, agt in enumerate(list_agents_full):
            agent_prev_category[ct] = agt.plan_category




        print("\n", file=sys.stderr, flush=True)
        counter += 1




if __name__ == '__main__':
    # Program arguments.
    parser = argparse.ArgumentParser(description='Simple client reading in levels from server for multi agent system')
    parser.add_argument('--max-memory', metavar='<MB>', type=float, default=2048.0,
                        help='The maximum memory usage allowed in MB (soft limit, default 2048).')

    args = parser.parse_args()

    # Set max memory usage allowed (soft limit).
    memory.max_usage = args.max_memory

    # Run client.
    main()