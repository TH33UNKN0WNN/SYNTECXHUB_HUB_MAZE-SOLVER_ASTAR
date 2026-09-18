"""
Maze Solver using A* Search
----------------------------
Represents a maze as a grid of nodes, finds the shortest path from
start to goal using A* search with a configurable heuristic
(Manhattan or Euclidean), handles unreachable goals gracefully,
and can visualize the search/path in the console.
"""

import heapq
import math


class Node:
    """Represents a single cell in the maze grid."""

    def __init__(self, row, col, walkable=True):
        self.row = row
        self.col = col
        self.walkable = walkable

        # A* bookkeeping
        self.g = float("inf")   # cost from start to this node
        self.h = 0               # heuristic estimate to goal
        self.f = float("inf")   # g + h
        self.parent = None

    def __lt__(self, other):
        # Needed for heapq tie-breaking when f-scores are equal
        return self.f < other.f

    def __repr__(self):
        return f"Node({self.row}, {self.col})"


class Maze:
    """Holds the grid and runs A* search over it."""

    def __init__(self, grid):
        """
        grid: list of lists where 0 = walkable, 1 = wall
        """
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.grid = [
            [Node(r, c, walkable=(grid[r][c] == 0)) for c in range(self.cols)]
            for r in range(self.rows)
        ]

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def get_neighbors(self, node):
        # 4-directional movement (up, down, left, right)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for dr, dc in directions:
            r, c = node.row + dr, node.col + dc
            if self.in_bounds(r, c) and self.grid[r][c].walkable:
                neighbors.append(self.grid[r][c])
        return neighbors

    @staticmethod
    def manhattan(a, b):
        return abs(a.row - b.row) + abs(a.col - b.col)

    @staticmethod
    def euclidean(a, b):
        return math.sqrt((a.row - b.row) ** 2 + (a.col - b.col) ** 2)

    def solve(self, start, goal, heuristic="manhattan"):
        """
        start, goal: (row, col) tuples
        heuristic: "manhattan" or "euclidean"
        Returns: (path, visited_order) where path is a list of (row, col)
                 tuples from start to goal, or None if unreachable.
        """
        start_node = self.grid[start[0]][start[1]]
        goal_node = self.grid[goal[0]][goal[1]]

        if not start_node.walkable or not goal_node.walkable:
            return None, []

        h_func = self.manhattan if heuristic == "manhattan" else self.euclidean

        # Reset bookkeeping in case solve() is called multiple times
        for row in self.grid:
            for node in row:
                node.g = float("inf")
                node.f = float("inf")
                node.parent = None

        start_node.g = 0
        start_node.h = h_func(start_node, goal_node)
        start_node.f = start_node.h

        open_set = []
        heapq.heappush(open_set, (start_node.f, id(start_node), start_node))
        open_set_hash = {start_node}
        closed_set = set()
        visited_order = []

        while open_set:
            _, _, current = heapq.heappop(open_set)
            open_set_hash.discard(current)

            if current in closed_set:
                continue
            closed_set.add(current)
            visited_order.append((current.row, current.col))

            if current == goal_node:
                return self._reconstruct_path(current), visited_order

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g = current.g + 1  # uniform step cost
                if tentative_g < neighbor.g:
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = h_func(neighbor, goal_node)
                    neighbor.f = neighbor.g + neighbor.h
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (neighbor.f, id(neighbor), neighbor))
                        open_set_hash.add(neighbor)

        # Goal is unreachable
        return None, visited_order

    @staticmethod
    def _reconstruct_path(node):
        path = []
        while node is not None:
            path.append((node.row, node.col))
            node = node.parent
        path.reverse()
        return path

    def print_maze(self, path=None, visited=None, start=None, goal=None):
        """Console visualization of the maze, path, and visited cells."""
        path_set = set(path) if path else set()
        visited_set = set(visited) if visited else set()

        for r in range(self.rows):
            row_str = ""
            for c in range(self.cols):
                pos = (r, c)
                if pos == start:
                    row_str += " S "
                elif pos == goal:
                    row_str += " G "
                elif not self.grid[r][c].walkable:
                    row_str += " # "
                elif pos in path_set:
                    row_str += " * "
                elif pos in visited_set:
                    row_str += " . "
                else:
                    row_str += "   "
            print(row_str)


def demo():
    # 0 = open path, 1 = wall
    grid = [
        [0, 0, 0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0, 1, 0],
        [0, 0, 0, 1, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0],
    ]

    maze = Maze(grid)
    start = (0, 0)
    goal = (6, 6)

    print("Maze layout ( # = wall, S = start, G = goal ):")
    maze.print_maze(start=start, goal=goal)

    path, visited = maze.solve(start, goal, heuristic="manhattan")

    print("\nSearch result:")
    if path:
        print(f"Path found! Length: {len(path)} steps")
        print(f"Nodes explored: {len(visited)}")
        print("Path:", path)
        print("\nVisualization ( * = path, . = explored, # = wall ):")
        maze.print_maze(path=path, visited=visited, start=start, goal=goal)
    else:
        print("No path found — goal is unreachable from start.")
        print(f"Nodes explored before giving up: {len(visited)}")
        print("\nVisualization of explored area ( . = explored, # = wall ):")
        maze.print_maze(visited=visited, start=start, goal=goal)


def demo_unreachable():
    # A maze where the goal is walled off completely
    grid = [
        [0, 0, 0, 1],
        [0, 1, 0, 1],
        [0, 1, 0, 1],
        [0, 0, 0, 1],
    ]
    maze = Maze(grid)
    start = (0, 0)
    goal = (0, 3)

    print("\n\n--- Unreachable goal demo ---")
    maze.print_maze(start=start, goal=goal)
    path, visited = maze.solve(start, goal)
    if path is None:
        print("\nCorrectly detected: goal is unreachable.")
    else:
        print("\nUnexpected: a path was found.")


if __name__ == "__main__":
    demo()
    demo_unreachable()
