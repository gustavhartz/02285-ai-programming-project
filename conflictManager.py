
from state import State
from action import Action, ActionType, Dir
from collections import deque
from pulp import *

#Squarebrackets på alle kald til dicts


class ConflictManager:


    def __init__(self):

        '''
        Percept the world with current state
        '''
        self.world_state = None


    #Efter temp_state er blevet konstrueret, skal konflikterne findes. 
    # Når konflikterne er fundet skal vi finde en måde at spore hvile agenter der konflikter 
    # Til sidst finde en måde at løse det på.

    def temp_state_create(self, agents: list):
        #Kontrollere foreløbigt et enkelt state
        #antage at world_state bliver opdateret i agent control loop, så     vi hver gang i denne metode har det korrekte billede af miljøet. 
        
        temp_state = State(self.world_state)

        agentDict = temp_state.reverse_agent_dict()

        box_agent_match = {}

        for agent in agents:
            action = agent.plan[0] # Et enkelt state
            agent_row = agentDict[agent.agent_char][1][0]
            agent_col = agentDict[agent.agent_char][1][1]

            if action.action_type is ActionType.Move:
                #Move agent
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])
                
                #Remove agent from old location
                temp_state.agents[f'{agent_row},{agent_col}'].pop(0)
            
            elif action.action_type is ActionType.Pull:
                box_id = temp_state.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'][0][2]
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''
                box_agent_match[box_id] = agent.char
                #Move agent
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])

                #Remove agent from old location
                temp_state.agents[f'{agent_row},{agent_col}'].pop(0)

                #Move Box
                temp_state.boxes[f'{agent_row},{agent_col}']\
                    .append(temp_state.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'][0])

                #Remove box from old location
                temp_state.boxes[f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'].pop(0)
        
            elif action.action_type is ActionType.Push:
                box_id = temp_state.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'][0][2]
                '''
                Antager her at en box ikke kan flyttes af flere agenter på samme tid
                '''
                box_agent_match[box_id] = agent.char


                #Move Box
                temp_state.boxes[f'{agent_row+action.agent_dir.d_row+action.box_dir.d_row},{agent_col+action.agent_dir.d_col+action.box_dir.d_col}']\
                    .append(temp_state.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'][0])
                
                #Remove Box
                temp_state.boxes[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'].pop(0)

                #Move agent
                temp_state.agents[f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}']\
                    .append(temp_state.agents[f'{agent_row},{agent_col}'][0])
                
                #Remove agent from old location
                temp_state.agents[f'{agent_row},{agent_col}'].pop(0)
        return [temp_state,box_agent_match]
        
   
    def check_collisions(self, agents):
        [temp_state,box_agent_match]  = self.temp_state_create(agents)

        agent_collisions = []
        agent_illegal_moves = []
        
        #Check Agents locations against other agents
        for loc, agt in temp_state.agents.items():
            if len(agt) > 1:
                agent_collisions.append(list(map(lambda x: x[1],agt)))

        #Check boxes against boxes
        for loc, box in temp_state.boxes.items():
            if len(box) > 1:
                #Sætter en box kollision i samme liste som en agent kollision (POTENTIEL ÆNDRING)
                temp = []
                stationary_box = False
                for b in box:
                    #If box exists in box_agent_match, the box has been moved by an agent, and then a NoOp should follow (ÆNDRING PÅ SIGT)
                    try:
                        temp.append(box_agent_match[b[2]])
                    #If box has not been moved, then all agents that moved boxes to this location should make other moves. 
                    except:
                        stationary_box = True
                if stationary_box:
                    agent_illegal_moves.append(temp)
                else:
                    agent_collisions.append(temp)

          
        #Check agents and boxes:
        for loc, agt in temp_state.agents.items():
            temp = list(map(lambda x : x[1],agt))
            
            if len(temp_state.boxes[loc]) > 0:
                stationary_box = False
                for box in temp_state.boxes[loc]:
                    #If box has been moved to loc, include the responsible agent in the conflict tuple
                    try:
                        temp.append(box_agent_match[box[2]])
                    #If box has not been moved, then all agents that moved boxes to this location should make other moves. 
                    except:
                        stationary_box = True
                        
                if stationary_box:
                    agent_illegal_moves.append(temp)
                else:
                    agent_collisions.append(temp)
        return [agent_collisions, agent_illegal_moves]

    def fix_collisions(self,agents):

        [agent_collisions, agent_illegal_moves] = self.check_collisions(agents)

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
                    agent.plan.appendleft(ActionType.NoOp)

        return [agent.plan.popleft() for agent in agents]
                    
                    

        










        
        






        


    




        


            
        
    









            





    
        

    





