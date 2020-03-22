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
                    colors = []
                    line = server_messages.readline()
                    while "#" not in [c for c in line]:
                        colors.append(line)
                        line = server_messages.readline()
                    break
                line = server_messages.readline()

            # Loading in the level
            if line != "#initial\n":
                raise Exception("Problem in parsing")
            line = server_messages.readline()
            row = 0
            while line != "#goal\n":
                for col, char in enumerate(line):
                    if char == '+':
                        self.initial_state.walls[row][col] = True
                    elif char in "0123456789":
                        self.initial_state.agents.append((char, [row, col]))
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                        self.initial_state.boxes[row][col] = char
                    elif char in "abcdefghijklmnopqrstuvwxyz":
                        self.initial_state.goals[row][col] = char
                    elif char == ' ':
                        # Free cell.
                        pass
                    elif char == '\n':
                        # End of line
                        break
                    else:
                        print(f'Error, read invalid level character: {char} at {row,col}', file=sys.stderr, flush=True)
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
                        self.initial_state.agents_goal.append((char, [row, col]))
                    elif char in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
                        self.initial_state.boxes_goal.append((char,[row, col]))
                    elif char in "abcdefghijklmnopqrstuvwxyz":
                        pass
                    elif char == ' ':
                        # Free cell.
                        pass
                    elif char == '\n':
                        # End of line
                        break
                    else:
                        print(f'Error, read invalid level character: {char} at {row, col}', file=sys.stderr,
                              flush=True)
                        sys.exit(1)
                row += 1
                line = server_messages.readline()
            print(f'Done with loading data', file=sys.stderr, flush=True)

        except Exception as ex:
            print('Error parsing level: {}.'.format(repr(ex)), file=sys.stderr, flush=True)
            sys.exit(1)


def main():
    # Read server messages from stdin.
    server_messages = sys.stdin

    # Use stderr to print to console through server.
    print('SearchClient initializing. I am sending this using the error output stream.', file=sys.stderr, flush=True)

    # Initialize level by sending client name
    ClientName= f"{config.client_name}\r".encode("ascii")
    sys.stdout.buffer.write(ClientName)
    sys.stdout.flush()

    # Read level and create the initial state of the problem.
    client = SearchClient(server_messages)




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


