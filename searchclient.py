import argparse
import re
import sys
import memory
import config

from state import State


class SearchClient:
    def __init__(self, server_messages):
        self.initial_state = None

        try:
            # Read lines for level.
            # TO DO: Edit 70x70 init to work with 2**15 x 2**15 maps as described in problem formulation
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
                    self.initial_state.colors = {}
                    line = server_messages.readline()
                    while "#" not in [c for c in line]:
                        current_color = line.split(':')[0]
                        for x in line.split(':')[1].replace(' ','').strip('\n').split(','):
                            self.initial_state.colors[x] = current_color
                        line = server_messages.readline()
                    break
                line = server_messages.readline()


            # Loading in the level
            if line != "#initial\n":
                raise Exception("Problem in parsing")
            line = server_messages.readline()
            row = 0
            box_id = 0
            while line != "#goal\n":
                for col, char in enumerate(line):
                    if char == '+':
                        self.initial_state.walls[f'{row},{col}'] = True
                    elif char in "0123456789":
                        self.initial_state.agents[f'{row},{col}'] = [[self.initial_state.colors[char], int(char)]]
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                        self.initial_state.boxes[f'{row},{col}'] = [[self.initial_state.colors[char], char, box_id]]
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
            print(f'Done with loading data', file=sys.stderr, flush=True)

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            sys.exit(1)

