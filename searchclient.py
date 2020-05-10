import argparse
import re
import sys
import memory
import config
from collections import defaultdict, deque
from itertools import chain

from state import State


class SearchClient:
    def __init__(self, server_messages):
        self.initial_state = None
        self.max_row = -1
        self.max_col = -1
        self.goal_dependencies = {}
        

        try:
            # Read lines for level.
            # TODO: Edit 70x70 init to work with 2**15 x 2**15 maps as described in problem formulation
            self.initial_state = State()
            line = server_messages.readline()

            # Get meta data
            while True:

                if line == "#domain\n":
                    line = server_messages.readline()
                    self.initial_state.domain = line

                elif line == "#levelname\n":
                    line = server_messages.readline()
                    self.initial_state.levelname = line

                elif line == "#colors\n":
                    # self.initial_state.colors = {}
                    # self.initial_state.colors = defaultdict(list)
                    line = server_messages.readline()
                    while "#" not in [c for c in line]:
                        current_color = line.split(':')[0]
                        for x in line.split(':')[1].replace(' ','').strip('\n').split(','):
                            self.initial_state.colors[x] = current_color
                            if not self.initial_state.colors_reverse[current_color]:
                                self.initial_state.colors_reverse[current_color] = [x]
                            else:
                                y = self.initial_state.colors_reverse[current_color]
                                y.append(x)
                                self.initial_state.colors_reverse[current_color] = y
                        line = server_messages.readline()
                    break
                line = server_messages.readline()
                


            # Loading in the level
            if line != "#initial\n":
                raise Exception("Problem in parsing")
            line = server_messages.readline()
            row = 0
            box_id = 0
            connected_component_num = -1
            while line != "#goal\n":
                self.max_row +=1
                for col, char in enumerate(line):
                    if char != '\n':
                        if col > self.max_col:
                            self.max_col = col

                    if char == '+':
                        self.initial_state.walls[f'{row},{col}'] = True
                    elif char in "0123456789":
                        internal_agt_id = 0
                        self.initial_state.agents[f'{row},{col}'] = [[self.initial_state.colors[char], int(char),internal_agt_id,connected_component_num]]
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                        self.initial_state.boxes[f'{row},{col}'] = [[self.initial_state.colors[char], char, box_id,connected_component_num]]
                        box_id += 1
                    elif char == ' ':
                        # Free cell.
                        pass
                    elif char == '\n':
                        # End of line
                        break
                    else:
                        print(f'Error, read invalid level character: {char,line} at {row,col}', file=sys.stderr, flush=True)
                        sys.exit(1)
                row += 1
                line = server_messages.readline()
                

            # Getting the goal setting
            row = 0
            while line != "#end\n":
                line = server_messages.readline()
                for col, char in enumerate(line):
                    if char == '+':
                        # nothing to do
                        pass
                    elif char in "0123456789":
                        self.initial_state.agents_goal[int(char)] = [f'{row},{col}']
                        self.initial_state.goal_positions[f'{row},{col}'] = char
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                        x = self.initial_state.boxes_goal[char]
                        x.append(f'{row},{col}')
                        self.initial_state.boxes_goal[char] = x
                        self.initial_state.goal_positions[f'{row},{col}'] = char
                    elif char == ' ':
                        # Free cell.
                        pass
                    elif char == '\n':
                        # End of line
                        break
                    elif line == "#end\n":
                        break

                    else:
                        print(f'Error, read invalid level character: {char, line} at {row, col}', file=sys.stderr,
                              flush=True)
                        sys.exit(1)
                row += 1

            # TODO: TESTING - dosent work currently
            # self.levelDesigner()

            print(f'Done with loading data', file=sys.stderr, flush=True)

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            sys.exit(1)


    def levelDesigner(self):

        connection_graph = defaultdict(list)
        for i in range(self.max_row+1):
            for j in range(self.max_col+1):
                if f'{i},{j}' not in self.initial_state.walls:
                    
                    if i != 0:
                        if f'{i-1},{j}' not in self.initial_state.walls:
                           connection_graph[f'{i},{j}'].append(f'{i-1},{j}')

                    if i != self.max_row:
                        if f'{i+1},{j}' not in self.initial_state.walls:
                            connection_graph[f'{i},{j}'].append(f'{i+1},{j}')

                    if j != 0:
                        if f'{i},{j-1}' not in self.initial_state.walls:
                           connection_graph[f'{i},{j}'].append(f'{i},{j-1}')

                    if j != self.max_col:
                        if f'{i},{j+1}' not in self.initial_state.walls:
                            connection_graph[f'{i},{j}'].append(f'{i},{j+1}')
        
        
        returned = set()
        connected_components = []
        
        #Create list of list of connected components 
        for node in connection_graph:
            if node not in returned:
                cncmp = self.bfs_connected_component(connection_graph,node)
                connected_components.append(cncmp)

                for cm in cncmp:
                    returned.add(cm)
            

        
        for elem in connected_components:
            print(elem, file=sys.stderr, flush=True)


        #Go over all nodes in the connected_components graph and see if any should be filtered out.
        #Filtering out is based on the subgraph containing any relevant box- or  agentgoals
        to_remove = []
        relevant_level = []
        for subgraph in connected_components:
            remove = True
            for node in subgraph:
                if node in self.initial_state.goal_positions:
                    remove = False  
            if remove:
                to_remove.append(subgraph)
            else:
                relevant_level.append(subgraph)
        
        #Flatten list of coordinats to remove
        all_remove = list(chain.from_iterable(to_remove))
        
        #print('to_remove', file=sys.stderr, flush=True)
        #print(to_remove, file=sys.stderr, flush=True)
        #print('relevant_level',file=sys.stderr, flush=True)
        #print(relevant_level,file=sys.stderr, flush=True)



        #Iterate over agents and boxes in initial state to remove any located in the irrelevant parts of the level
        
        '''
        Slet agenten - husk hvem der slettes så vi kan sende NoOps i main
        
        '''
        #TODO: HUSK at smide ud til main.py hvilke agenter der er blevet slettet, så vi kan sende en NoOp automatisk 

        del_agents = [key for key in self.initial_state.agents.keys() if key in all_remove]
        for loc in del_agents:
            del self.initial_state.agents[loc]
        
        a_inter_id = 0
        for _,agt in self.initial_state.agents.items():
            agt[0][2] = a_inter_id
            a_inter_id+=1

        #Re-indexing of our box_id's 
        del_boxes = [key for key in self.initial_state.boxes.keys() if key in all_remove]
        
        for loc in del_boxes:
            del self.initial_state.boxes[loc]
        
        b_id= 0
        for _,box in self.initial_state.boxes.items():
            box[0][2] = b_id
            b_id+=1    


        #Assign connected component to agents:
        for loc, agt in self.initial_state.agents:
            for idx,cncmp in enumerate(connected_components):
                if loc in cncmp:
                    agt[3] = idx
        
        #Assign connected component to boxes:
        for loc, box in self.initial_state.boxes:
            for idx,cncmp in enumerate(connected_components):
                if loc in cncmp:
                    box[3] = idx
        
        


        #move_directions

        #dir_moves = [[1,0],[-1,0],[0,1],[0,-1]]
        #dir_all = [[1,1],[1,0],[1,-1],[0,-1],[-1,-1]‚[-1,0],[-1,1],[0,1]]

        for subgraph in relevant_level:
            for node in subgraph:
                
                num_connections = len(connection_graph[node])
                
                well_id = 0
                if num_connections == 1:
                    self.initial_state.wells[node] = [well_id,0]
                    well_id+=1

                elif num_connections == 2:
                    #TODO: Et hjørne bliver nu til en tunnel - opvervej om det er korrekt 

                    self.initial_state.tunnels[node] = -1

                

        #Transform tunnels into wells where necessary
        #TODO: Overvej om dette rekursive kald er vejen frem (store levels)

        #Define a list of dependencies in wells
        dependencies_wrong_order = []

        for loc, well in self.initial_state.wells.items():
            listo = []
            #makeWell(self, graph, coordinate,cost,goal_priority_list,well_id):
            self.makeWell(connection_graph,loc,well[0],listo,well[1])
            
            if len(listo)> 0:
                dependencies_wrong_order.append(listo)



        #Go through all tunnels, and assign them id's
        tunnel_id = well_id+1

        returned = set()
        #Create list of list of connected components 
        for loc in self.initial_state.tunnels:
            if node not in returned:
                tunnel = self.bfs_tunnel(connection_graph,loc,tunnel_id)
                tunnel_id+=1
                for tun in tunnel:
                    returned.add(tun)

        
        #Iterate over goals, add them as individuals if they have no dependencies in the wells
        for loc in self.initial_state.goal_positions:
            not_in = True
            for dependency in dependencies_wrong_order:
                if loc in dependency:
                    not_in = False
            
            if not_in:
                dependencies_wrong_order.append([loc])



        #Reverse the order and point to a dictionary, so a dependency that were [1,2,3] (1->2->3) becomes
        #{3: [2,1], 2: [1], 1: []}
        for element in dependencies_wrong_order:
            for i in reversed(range(len(element))):
                self.goal_dependencies[element[i]]=[element[j] for j in reversed(range(i))]


        #Reverse lists of wells and tunnels (including the mouths)

        for loc, well in self.initial_state.wells.items():
            self.initial_state.wells_reverse[well[0]].append[loc]

        for loc, tunnel in self.initial_state.tunnels.items():
            self.initial_state.tunnels_reverse[tunnel].append[loc]

        
        for loc, mouth_id in self.initial_state.mouths.items():
            for idx in mouth_id:
                if idx in self.initial_state.wells_reverse:
                    self.initial_state.wells_reverse[idx].append(loc)
                elif idx in self.initial_state.tunnels_reverse:
                    self.initial_state.tunnels_reverse[idx].append(loc)
                else:
                    raise(f'ID mismatch when including mouths in the tunnels/wells, on id: {idx}')


        





        '''
        Eventuelt implementer at en tunnel er et objekt: Indgang, udgang, længde osv
        Definer logik for junctions
        
        '''

    def bfs_connected_component(self, graph, start):


        # keep track of all visited nodes
        explored = []
        # keep track of nodes to be checked
        queue = deque()
        queue.append(start)
    
        # keep looping until there are nodes still to be checked
        while queue:
            node = queue.popleft()
            if node not in explored:
                # add node to list of checked nodes
                explored.append(node)
                neighbours = graph[node]
                # add neighbours of node to queue
                for neighbour in neighbours:
                    queue.append(neighbour)
 
        return explored

    

    def makeWell(self, graph,coordinate,cost,goal_priority_list,well_id):
            
            if coordinate in self.initial_state.goal_positions:
                goal_priority_list.append(coordinate)

            #Recursively find and identify wells
            connects = graph[coordinate]
            for con in connects:
                if con in self.initial_state.tunnels:
                    if con != coordinate:

                        self.initial_state.wells[con] = [well_id,cost+1]
                        del self.initial_state.tunnels[con]
                        
                        self.makeWell(graph,con,cost+1,goal_priority_list,well_id)
                elif (con not in self.initial_state.tunnels) and (con not in self.initial_state.wells):
                        self.initial_state.mouths[con].append(well_id)
            

    def bfs_tunnels(self,graph,start,tunnel_id):
        # keep track of all visited nodes
        explored = []
        # keep track of nodes to be checked
        queue = deque()
        queue.append(start)
    
        # keep looping until there are nodes still to be checked
        while queue:
            node = queue.popleft()
            if node not in explored:
                # add node to list of checked nodes
                explored.append(node)
                neighbours = graph[node]
                # add neighbours of node to queue
                for neighbour in neighbours:
                    if neighbour in self.initial_state.tunnels:
                        queue.append(neighbour)
                        self.initial_state.tunnels[neighbour] = tunnel_id 
                    else:
                        #Neighbour not a tunnel - add to mouths instead    
                        self.initial_state.mouths[neighbour].append(tunnel_id)
 
        return explored


                    

                        

        
         




        







