# CSC-2400-MazeSolvingProject
## Project Title : Maze Solving
Maze Solving using BFS, DFS, A*, Bidirectionals search, and Ant colony optimization(ACO)
### Project description : 
The maze problem involves finding a path from a starting position to a destination within a maze. This project solves the maze pathfinding problem using multiple algorithms. The goal is to find a path from the start cell to the goal cell while avaoiding walls(blocked cells).
Algorithms that are implemented :
- Breadth First Search (BFS) : This algorithm explores all nearby paths level by level and guarantees the shortest path in an unweighted maze.
- Depth First Search (DFS) : This algorithm explores one path deeply before backtracking, which may not always give the shortest path.
- A* or Dijkstra :This algorithm uses path cost and heuristic estimation to quickly find the shortest and most efficient path.
- Ant Colony Optimization (ACO) -It is a probabilistic, nature-inspired metaheuristic algorithm used to solve complex computational problems—such as routing and scheduling—by finding optimal paths through graphs. It simulates the foraging behavior of real ants, where "artificial ants" deposit pheromones on shorter, successful paths, guiding other agents to reinforce the best solutions

### Datasets Used : 

{ Maze data creation and dataset:
https://www.kaggle.com/datasets/mexwell/maze-dataset/data?select=perfect_maze } 
 



### Team Members : 
- Mani Botla (https://github.com/bmaniswarupa-lab)
- Anissa Sollman (https://github.com/anissabelle)
- Nick Liverett (https://github.com/nicholasliverett)

### How to Compile and Run the code
- Requirements to compile and run :
  * C++ Compiler
  * VS Code
- Steps to Run :
```bash
g++ main.cpp -o maze_runner.exe

./maze_runner.exe
# Either specify a file or directory to run the algorithms in and select options in the menu
# In the case of specifing a directory, it will run all the algorithms on all the files.
```




### Expected Output : 
The program should display: 
- BFS Path
- DFS Path
- A* Path
- ACO Result
********Example Output********

### Project Objectives: 
- Compare pathfinding algorithms
- Measure shortest path efficiency
- Analyze execution time
- Compare memory usage
- Study optimization methods in maze solving

### GenAI Usage: 


