# naming the client
import sys
from math import inf

import random

client_name = "Deep_Plan"
illegal_move_threshold = 10**10

sys.setrecursionlimit(1180)

agent_list_str=['0','1','2','3','4','5','6','7','8','9']

max_replanning_depth=5

max_replanning_steps=max_replanning_depth*2

while_counter = 100

# This is the maximum amount of nodes allowed to explored
max_search_depth = 2000

#Random agents variables
initiate_random_agents = True
agent_max_stall = 5

#FOR OUR OWN LEVEL, seed = 88
seed = 899 # best atm
seed = 17






no_task=1
goal_assigner_location=2
goal_assigner_box=3
awaiting_help = 4
self_helping = 5
solving_help_task = 6
