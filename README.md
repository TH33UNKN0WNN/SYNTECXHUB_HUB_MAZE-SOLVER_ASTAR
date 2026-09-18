# Syntecxhub_Maze_Solver_AStar

A Python implementation of a Maze Solver using **A\* Search**, built for the Syntecxhub AI Internship (Project 1).

## Features
- Represents the maze as a grid of `Node` objects (start, goal, walls).
- Implements A* search with a configurable heuristic — **Manhattan** or **Euclidean** distance.
- Returns the shortest path from start to goal.
- Gracefully handles **unreachable** goals (returns `None` instead of crashing).
- Includes a simple **console visualization** of the explored nodes and final path.

## How to Run
```bash
python3 maze_solver.py
```

This runs two demos:
1. A solvable maze — prints the maze, the path found, and a visualization.
2. An unreachable-goal maze — demonstrates correct handling of no-solution cases.

## How It Works
- `Maze` class builds a grid of `Node`s from a 2D list (`0` = walkable, `1` = wall).
- `Maze.solve(start, goal, heuristic)` runs A*:
  - `g` = cost from start to the current node
  - `h` = heuristic estimate to the goal (Manhattan or Euclidean)
  - `f = g + h` — nodes are expanded in order of lowest `f` using a min-heap (`heapq`)
- When the goal is popped from the open set, the path is reconstructed by following `parent` pointers back to the start.
- If the open set empties without reaching the goal, the goal is unreachable.

## Using Your Own Maze
```python
from maze_solver import Maze

grid = [
    [0, 0, 0],
    [1, 1, 0],
    [0, 0, 0],
]
maze = Maze(grid)
path, visited = maze.solve(start=(0, 0), goal=(2, 2), heuristic="manhattan")
print(path)
```

## Submission Checklist
- [ ] Push this code to a GitHub repo named `Syntecxhub_Maze_Solver_AStar`
- [ ] Share internship status on LinkedIn, tagging @Syntecxhub
- [ ] Submit via the official Submission Form
