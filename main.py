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
    counter = 0
    while True:
        print(f"WHILE ENTER {counter}\n", file=sys.stderr, flush=True)
        # print(f"{current_state.boxes} boxes", file=sys.stderr, flush=True)
        # print(f"{current_state.agents} agents", file=sys.stderr, flush=True)

        for e in list_agents:
            print(f'{e.plan[0]} {e.agent_char} before', file=sys.stderr, flush=True)


        list_of_actions = conflict_manager.fix_collisions(list_agents)

        # push to server -> list of actions
        my_string = ';'.join(list(str(x) for x in list_of_actions))
        print(my_string, flush=True)

        # getting the data from the client to asses validity of actions
        response = server_messages.readline().rstrip()
        print(response, file=sys.stderr, flush=True)

        current_state.world_state_update(list_of_actions)

        if current_state.world_is_goal_state():
            print("Done", file=sys.stderr, flush=True)
            break

        conflict_manager.world_state = cp(current_state)
        GoalAssigner.world_state = cp(current_state)

        x.assign_tasks()

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