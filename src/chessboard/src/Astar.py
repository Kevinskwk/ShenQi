class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


class Astar():

    def __init__(self, grid):
        self.W = 9
        self.H = 10
        self.grid = grid

    def get_adjacent(self, pos, occupied=False):
        res = []
        for adj in [(0, -1), (0, 1), (-1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1), (1, 0)]:
            # Get new position
            new_pos = (pos[0] + adj[0], pos[1] + adj[1])
            # Make sure within range and walkable terrain
            if new_pos[0] < self.W and new_pos[0] >= 0 and \
                new_pos[1] < self.H and new_pos[1] >= 0 and \
                self.grid[new_pos[0] + new_pos[1]*self.W] == occupied:
                res.append(new_pos)
        return res

    def astar(self, start, end, eat=False):
        """Returns a list of tuples as a path from the given start to the given end in the given grid"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if ((not eat) and current_node == end_node) or (eat and current_node.position[0] == 0):
                path = [(-1, current_node.position[1])] if eat else []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for new_position in self.get_adjacent(current_node.position):

                # Create new node
                new_node = Node(current_node, new_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:
                flag = False
                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        flag = True
                        break
                if flag:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                if eat:
                    child.h = child.position[0] ** 2
                else:
                    child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        flag = True
                        break
                if flag:
                    continue

                # Add the child to the open list
                open_list.append(child)

    def find_path(self, grid, start, end):
        paths = []
        self.grid = grid.copy()
        eat = self.grid[end[1]*self.W + end[0]]
        move_back = None

        if eat:
            print("eating")
            path = self.astar(end, None, eat=True)
            if path is None:
                print("processing exceptional case")
                blockings = []
                # get first iteration of blockings
                check_tile = list(end)
                while len(blockings) == 0:
                    blockings = self.get_adjacent(check_tile, occupied=True)
                    check_tile[0] -= 1
                # iterating through all blockings
                # This is actually an Astar inside a BFS
                i = 0
                while i < len(blockings):
                    blocking = blockings[i]
                    self.grid[blocking[1]*self.W + blocking[0]] = False
                    for vacancy in self.get_adjacent(blocking):
                        self.grid[vacancy[1]*self.W + vacancy[0]] = True
                        new_path = self.astar(end, None, eat=True)
                        if new_path is not None: # found a path
                            paths.append([blocking, vacancy])
                            paths.append(new_path)
                            move_back = [vacancy, blocking]
                            break
                        self.grid[vacancy[1]*self.W + vacancy[0]] = False
                    if len(paths) != 0:
                        break

                    # append new blockings
                    self.grid[blocking[1]*self.W + blocking[0]] = True
                    for new_blocking in self.get_adjacent(blocking, occupied=True):
                        if new_blocking not in blockings:
                            blockings.append(new_blocking)
                    i += 1
                # leave the opening there as the next move might need as well
            else:
                paths.append(path)

        self.grid[end[1]*self.W + end[0]] = False
        path = self.astar(start, end)
        if path is None:
            # special blocked case for cannon
            blockings = []
            if start[0] == end[0]: # same column
                increment = 1 if start[1] < end[1] else -1
                for i in range(start[1]+increment, end[1], increment):
                    if self.grid[i*self.W + end[0]]: # found the blocking piece
                        blockings.append((end[0], i))
                        break

            elif start[1] == end[1]: # same row
                increment = 1 if start[0] < end[0] else -1
                for i in range(start[0]+increment, end[0], increment):
                    if self.grid[end[1]*self.W + i]: # found the blocking piece
                        blockings.append((i, end[1]))
                        break
            if len(blockings) == 0: # SHOULD NOT HAPPEN
                print("ILLEGAL MOVE OF CANNON/BLOCKING OF NON-CANNON!")
                return paths

            # iterating through all blockings
            # This is actually an Astar inside a BFS
            i = 0
            while i < len(blockings):
                blocking = blockings[i]
                self.grid[blocking[1]*self.W + blocking[0]] = False
                for vacancy in self.get_adjacent(blocking):
                    self.grid[vacancy[1]*self.W + vacancy[0]] = True
                    new_path = self.astar(start, end)
                    if new_path is not None: # found a path
                        paths.append([blocking, vacancy])
                        paths.append(new_path)
                        paths.append([vacancy, blocking])
                        break
                    self.grid[vacancy[1]*self.W + vacancy[0]] = False
                if len(paths) != 0:
                    break

                # append new blockings
                self.grid[blocking[1]*self.W + blocking[0]] = True
                for new_blocking in self.get_adjacent(blocking, occupied=True):
                    if new_blocking not in blockings:
                        blockings.append(new_blocking)
                i += 1

        else:
            paths.append(path)

        if move_back is not None:
            paths.append(move_back)
        return paths

def main():

    grid = [True , True , True , True , True , True , True , True , True ,
            False, False, False, False, False, False, False, False, False,
            False, True , False, False, False, False, False, True , False,
            True , False, True , False, True , False, True , False, True ,
            False, False, False, False, False, False, False, False, False,
            False, False, False, False, False, False, False, False, False,
            True , True, True , True, True , True, True , True, True ,
            False, True , False, False, False, False, False, True , True,
            False, False, False, False, False, False, False, True, True,
            True , True , True , True , True , True , True , True , False ]

    start = (8, 7)
    end = (8, 9)
    astar = Astar(grid)
    eat = False

    paths = astar.find_path(grid, start, end, eat)
    print(paths)


if __name__ == '__main__':
    main()