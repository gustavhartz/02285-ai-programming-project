
from state import State
from action import Action, ActionType, Dir
from collections import deque


class ConflictManager:


    def __init__(self):

        '''
        Percept the world with current state
        '''
        self.world_state = None


#Efter temp_state er blevet konstrueret, skal konflikterne findes. 
# Når konflikterne er fundet skal vi finde en måde at spore hvile agenter der konflikter 
# Til sidst finde en måde at løse det på.

def temp_state_create(agents):
    #Kontrollere foreløbigt et enkelt state
    #antage at world_state bliver opdateret i agent control loop, så vi hver gang i denne metode har det korrekte billede af miljøet. 
    
    temp_state = State(self.world_state)

    agentDict = temp_state.reverse_agent_dict()

    for agent in agents:
        action = agent.plan[0] # Et enkelt state
        agent_row = agentDict[agent.agent_char][1][0]
        agent_col = agentDict[agent.agent_char][1][1]

        if action.action_type is ActionType.Move:
            #Move agent
            temp_state.agents(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')\
                .append(temp_state.agents(f'{agent_row},{agent_col}'))
            
            #Remove agent from old location
            temp_state.agents(f'{agent_row},{agent_col}').pop(0)
        
        elif action.action_type is Actiontype.Pull:
            #Move agent
            temp_state.agents(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')\
                 .append(temp_state.agents(f'{agent_row},{agent_col}'))

            #Remove agent from old location
            temp_state.agents(f'{agent_row},{agent_col}').pop(0)

            #Move Box
            temp_state.boxes(f'{agent_row},{agent_col}')\
                .append(temp_state.boxes(f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}'))

            #Remove box from old location
            temp_state.boxes(f'{agent_row+action.box_dir.d_row},{agent_col+action.box_dir.d_col}').pop(0)
    
        elif action.action_type is Actiontype.Push:
            #Move Box
            temp_state.boxes(f'{agent_row+action.agent_dir.d_row+action.box_dir.d_row},{agent_col+action.agent_dir.d_col+action.box_dir.d_col}')\
                .append(temp_state.boxes(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}'))
            
            #Remove Box
            temp_state.boxes(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}').pop(0)

            #Move agent
            temp_state.agents(f'{agent_row+action.agent_dir.d_row},{agent_col+action.agent_dir.d_col}')\
                 .append(temp_state.agents(f'{agent_row},{agent_col}'))
            
            #Remove agent from old location
            temp_state.agents(f'{agent_row},{agent_col}').pop(0)
            
        
    









            





    
        

    





