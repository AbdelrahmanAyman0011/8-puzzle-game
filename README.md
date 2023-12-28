# 8-Puzzle-Problem

## Table of Contents

1. [Introduction](#introduction)
2. [Assumptions](#assumptions)
3. [Data Structure Used](#data-structure-used)
4. [Algorithms](#algorithms)
5. [Results & Output](#results--output)

---

## Introduction <a name="introduction"></a>

The 8-Puzzle Problem is a classic challenge in artificial intelligence and search algorithms. Its objective is to rearrange a 3x3 grid of numbered tiles from an initial state to a goal state using the fewest moves possible. This report explores the implementation and performance analysis of three search algorithms: Depth-First Search (DFS), Breadth-First Search (BFS), and A\*.

---

## Assumptions <a name="assumptions"></a>

1. The initial state consists of 9 unique numbers from 0 to 8, where 0 represents the empty tile.
2. The goal state is a predefined arrangement where numbers ascend from left to right, top to bottom.

---

## Data Structure Used <a name="data-structure-used"></a>

### Puzzle State Representation

- **Matrix**: The puzzle state is represented as a 3x3 matrix, each element signifying a tile.
- **Structs Used**: "PuzzleState" encapsulates the 2D vector representing the puzzle state, and "Node" incorporates the puzzle state, parent node, and action direction.

### Node Expansion & Exploration

- **DFS & BFS**: Expansion involves iterating through possible moves (left, right, up, down), storing valid moves in a frontier container, and checking if it's the goal or the next parent node.
- **A\***: Expansion involves evaluating child nodes by valid moves (Up, Down, Right, Left), using a priority queue to select nodes with the lowest heuristic (Manhattan or Euclidean distance) for expansion.

### Set of the Explored

- **Storage**: Node states are converted to strings using a "toString" function and stored in an unordered set of states, preventing revisiting explored nodes.

### Search Algorithm

- A common class "Search Algorithm" houses shared methods like "isGoalState," "toString," and "printPuzzle" for accessing by each algorithm class.

---

## Algorithms <a name="algorithms"></a>

1. **Depth-First Search (DFS)**: Explores the search space deeply before backtracking, using a stack to store nodes and an unordered set for visited nodes.
2. **Breadth-First Search (BFS)**: Explores the search space level by level, utilizing a queue and an unordered set for visited nodes.
3. **A\***: An informed search using a combination of node cost and heuristic estimate, employing a priority queue based on the lowest f = g + h value.

---

## Results & Output <a name="results--output"></a>

### BFS

- **Path to Goal**: Displayed for each iteration.
- **Cost of Path**: Total moves from initial to goal state.
- **Nodes Expanded**: Total nodes expanded during the search.
- **Search Depth**: Depth of the solution found.
- **Running Time**: Total time for solution.

### DFS

- **Cost of Path**: Total moves from initial to goal state.
- **Nodes Expanded**: Total nodes expanded during the search.
- **Max Depth**: Depth of the solution found.
- **Running Time**: Total time for solution.

### A\*

- **Depth of The Path**: Depth of the solution found.
- **Number of Nodes Visited**: Nodes visited in the path.
- **Number of Nodes Expanded**: Total nodes expanded.
- **Nodes Expanded**: Total nodes expanded during the search.
- **Running Time**: Total time for solution.

### Sample Run

#### BFS

1. **Enter 9 numbers from 0 to 8 for the initial state of the puzzle:** 1 2 5 3 4 0 6 7 8
   **Initial State:**

2. **Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):** 1
   **BFS:**

   - **Goal State Found:**

     - **Iteration 1: Direction UP**

       ```
       1 2 0
       3 4 5
       6 7 8
       ```

     - **Iteration 2: Direction LEFT**

       ```
       1 0 2
       3 4 5
       6 7 8
       ```

     - **Iteration 3: Direction LEFT**

       ```
       0 1 2
       3 4 5
       6 7 8
       ```

#### A\* Algorithm

**Manhattan Heuristic:**

1. **Enter 9 numbers from 0 to 8 for the initial state of the puzzle:** 1 2 5 3 4 0 6 7 8
   **Initial State:**

2. **Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):** 3
   **Solve by Euclidean (1) or by Manhattan (2):** 2
   **A\* using Manhattan:**

   - **Direction UP**

     ```
     1 2 0
     3 4 5
     6 7 8
     ```

   - **Direction Left**

     ```
     1 0 2
     3 4 5
     6 7 8
     ```

   - **Direction Left**

     ```
     0 1 2
     3 4 5
     6 7 8
     ```

   - **GOAL IS REACHED**

     - **Depth of the path:** 3
     - **Number of nodes visited:** 4
     - **Number of nodes expanded:** 7
     - **Running Time:** 5 milliseconds

**Euclidean Heuristic:**

1. **Enter 9 numbers from 0 to 8 for the initial state of the puzzle:** 1 2 5 3 4 0 6 7 8
   **Initial State:**

2. **Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):** 3
   **Solve by Euclidean (1) or by Manhattan (2):** 1
   **A\* using Euclidean:**

   - **Direction UP**

     ```
     1 2 0
     3 4 5
     6 7 8
     ```

   - **Direction Left**

     ```
     1 0 2
     3 4 5
     6 7 8
     ```

   - **Direction Left**

     ```
     0 1 2
     3 4 5
     6 7 8
     ```

   - **GOAL IS REACHED**

     - **Depth of the path:** 3
     - **Number of nodes visited:** 4
     - **Number of nodes expanded:** 7
     - **Running Time:** 4 milliseconds
