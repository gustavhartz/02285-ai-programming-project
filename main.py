import argparse
import re
import sys
import memory
import config
from searchclient import SearchClient
from agent import search_agent
from strategy import StrategyBFS, StrategyDFS
from goalassignment import *
from conflictManager import ConflictManager
from state import State
from copy import deepcopy as cp


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
    Testing the agent class moving box
    '''

    # client.initial_state
    #
    # agent = search_agent(0, 'red', StrategyBFS)
    # #
    # for key, value in client.initial_state.boxes.items():
    #     box_start = key
    # for key, value in client.initial_state.boxes_goal.items():
    #     box_end = value[0]
    #
    # agent.search_box(client.initial_state, box_start, box_end)
    #
    # print(agent.plan, file=sys.stderr, flush=True)
    # for action in agent.plan:
    #     print(str(action)[1:len(str(action))-1], flush=True)

    '''
    Testing agent class placement
    '''

    # client.initial_state
    #
    # agent = search_agent(0, 'red', StrategyBFS)
    #
    # location = client.initial_state.agents_goal[0]
    #
    # # Location return a list
    # agent.search_position(client.initial_state, location[0])
    #
    # print(agent.plan, file=sys.stderr, flush=True)
    #
    # print(agent.plan, file=sys.stderr, flush=True)
    # for action in agent.plan:
    #     print(str(action)[1:len(str(action))-1], flush=True)

    '''
    Testing with two agents
    '''

    #
    # agent1 = search_agent(1, 'blue', StrategyBFS)
    #
    # agent0 = search_agent(0, 'red', StrategyBFS)
    #
    #
    # # Location return a list
    # location = client.initial_state.agents_goal[0]
    # # Find plan for solving task
    # agent0.search_position(client.initial_state, location[0])
    #
    # for key, value in client.initial_state.boxes.items():
    #     box_start = key
    # for key, value in client.initial_state.boxes_goal.items():
    #     box_end = value[0]
    #
    # agent1.search_box(client.initial_state, box_start, box_end)
    #
    # print(agent0.plan, file=sys.stderr, flush=True)
    # print(agent1.plan, file=sys.stderr, flush=True)
    #
    #
    # while len(agent1.plan)>0 or len(agent0.plan)>0:
    #     string_action=""
    #     if len(agent0.plan)>0:
    #         test = agent0.plan.popleft()
    #         string_action = string_action+str(test)[1:len(str(test))-1]+";"
    #     else:
    #         string_action = string_action + "NoOp"
    #
    #     if len(agent1.plan)>0:
    #         test = agent1.plan.popleft()
    #         string_action = string_action+str(test)[1:len(str(test))-1]
    #     else:
    #         string_action = string_action + "NoOp"
    #
    #     print(string_action, flush=True)

    '''
    Testing assigner 
    '''
    current_state = State(client.initial_state)

    # Create list of agents
    list_agents = []
    for k, v in current_state.reverse_agent_dict().items():
        list_agents.append(search_agent(k, v[0], StrategyBFS))

    list_agents.sort()
    x = GoalAssigner(current_state, list_agents)
    x.assign_tasks()
    conflict_manager = ConflictManager()
    conflict_manager.world_state = current_state


    # Whileloop
    counter=0
    while True:
        print(f"WHILE ENTER {counter}\n", file=sys.stderr, flush=True)
        print(f"{current_state.boxes} boxes", file=sys.stderr, flush=True)
        print(f"{current_state.agents} agents", file=sys.stderr, flush=True)

        for e in list_agents:
            print(f'{e.plan[0]} {e.agent_char} before', file=sys.stderr, flush=True)


        list_of_actions = conflict_manager.fix_collisions(list_agents)

        # push to server -> list of actions
        my_string = ';'.join(list(str(x) for x in list_of_actions))

        current_state.world_state_update(list_of_actions)

        print(f"{current_state.boxes} boxes", file=sys.stderr, flush=True)
        print(f"{current_state.agents} agents", file=sys.stderr, flush=True)



        if current_state.world_is_goal_state():
            print("Done", file=sys.stderr, flush=True)
            break


        # update of state in classes
        x.assign_tasks()

        conflict_manager.world_state = cp(current_state)
        GoalAssigner.world_state = cp(current_state)

        x.assign_tasks()
        print("\n", file=sys.stderr, flush=True)
        counter+=1




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