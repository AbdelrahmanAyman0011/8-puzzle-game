**8-Puzzle-Problem**

**Developed by:**

**Ahmed Sabry Ibrahim Abdelrahman Ayman Saad Youssef Waleed Mostafa**

**Table of Contents:**

- **Introduction**
- **Assumptions**
- **Data Structure Used**
- **Algorithms**
- **Results & Output**

**Introduction**

The 8-Puzzle Problem is a classic problem in artificail intelligence and=d search algorithms. The goal is to re arrange a 3x3 grid of numbered tiles from an initial state to a goal state using the minimum number of moves. This report presents the implementation and performance analysis of three search algorithms: Depth-First Search (DFS), Breadth-First Search (BFS), and A\* .

**Assumptions**

- The initial state of the puzzle is entered by the user, and it consists of 9 unique numbers from 0 to 8, where 0 represents the empty tile.
- The goal state is a predefined arrangement where numbers are in ascending order from left to right, top to bottom.

**Data Structure Used**

**Puzzle State Representation**

The puzzle state is represented as a 3x3 matrix, where each element represents a tile in the puzzle.

The “PuzzleState” struct is utilized to encapsulate the puzzle state as a 2D Vector, the “zero index” position.

The “Node” struct is utilized to encapsulate the puzzle state as an instance of a “PuzzleState” class, the “parent” node, the action direction.

**Node Expansion & Exploration**

In DFS & BFS To expand the nodes we used a certain method, in which we declared two vectors that store the values of the possible index moves and then iterated on every possible move such as left, right, up, down.. if it is a valid move we add it to our frontier container and use it, checking if it is our goal or it can be our next parent node.

In A\* When we call the function “solveA” we have our current node, in a while loop we check if the state of the goal is not our goal, then we start expanding the childes of these node by: first checking if every move such as Up, Down, Right and Left is a valid move or not by checking the position of the zero value “empty tile” that is stored in the variables x & y, if the move is valid we add it to our frontier of the type priority queue and then choosing the node with the least F and then add it to the visited list, at the end we update our current node to the chosen node of the least f using manhattan or euclidean distance (choosen by the user).

**Set of the Explored**

To store every node state that we already used, we convert our state to a string using a “toString” function and then store it in an unordered set of states in a “string” shape, which will allow us to easily access and search on the set to prevent visiting any explored node again.

**Search Algorithm**

A class called “Search Algorithm” is used to determine some common methods the algorithms need such as “isGoalState” function to compare our current state to the goal, “toString” to convert the state to a string, “printPuzzle” to print the puzzle state in each iteration, each algorithm class can access this class.

**Algorithms**

**Depth-First Search (DFS)**

DFS “uninformed search” algorithm explores the search![](Aspose.Words.564d53c8-1fc2-4889-984d-40f106ee05e5.001.jpeg) space by traversing as far as possible along each branch before backtracking, this is mainly implemented using a stack to store the frontier nodes, an unordered set to store.

**Breadth-First Search (BFS)**

BFS “uninformed search” algorithm explores the search space level by level, expanding all nodes at the current depth before moving on to the next depth, this is mainly implemented using a queue that contains a pair of puzzle state and voctor of tuples that contains “directions, step number, state of the corresponding step”, and an unordered set to store the already visited nodes.

**A\***

A\* is an “informed search” algorithm that uses a combination of the cost to reach a node (g) and the heuristic estimate of the remaining cost to the goal (h). The priority queue is employed to select nodes with the lowest f = g + h value for expansion, this is implemented using a priority queue of the lowest f.

**Results**

**BFS**

- Path to Goal: Displayed for each iteration.
- Cost of Path: The total number of moves from the initial state to the goal state.
- Nodes Expanded: The total number of nodes expanded during the search.
- Search Depth: The depth of the solution found.
- Running Time: The total time taken to find the solution.

**DFS**

- Cost of Path: The total number of moves from the initial state to the goal state.
- Nodes Expanded: The total number of nodes expanded during the search.
- Max Depth: The depth of the solution found.
- Running Time: The total time taken to find the solution.

**A\***

- Depth of The Path: The depth of the solution found.
- Number of Nodes Visited: The number of nodes that actually visited in our path.
- Number of Nodes Expanded: The total number of nodes expanded during the search.
- Nodes Expanded: The total number of nodes expanded during the search.
- Running Time: The total time taken to find the solution.

**Sample Run**

**BFS**

Enter 9 numbers from 0 to 8 for the initial state of the puzzle: 1 2 5 3 4 0 6 7 8

Initial State:

Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):1

BFS:

Goal State Found:

Iteration 1:

Direction: up

1 2 0

3 4 5

6 7 8

Iteration 2: Direction: left 1 0 2

3 4 5

6 7 8

Iteration 3: Direction: left 0 1 2

3 4 5

6 7 8

Path to Goal: 2 steps Cost of Path: 2 Nodes Expanded: 15

Search Depth: 2

Running Time: 0 milliseconds

**DFS**

Enter 9 numbers from 0 to 8 for the initial state of the puzzle: 1 2 5 3 4 0 6 7 8

Initial State:

Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):2

DFS:

Goal State Found:

Direction: left

1 2 5

3 0 4

6 7 8

Direction: left 1 2 5

0 3 4

6 7 8

Direction: up 0 2 5

1 3 4

6 7 8

Direction: right 2 0 5

1 3 4

6 7 8

Direction: right 2 5 0

1 3 4

6 7 8

Direction: down

2 5 4 1 3 0 6 7 8

Direction: left 2 5 4

1 0 3

6 7 8

Direction: left 2 5 4

0 1 3

6 7 8

Direction: up 0 5 4

2 1 3

6 7 8

Direction: right 5 0 4

2 1 3

6 7 8

Direction: right 5 4 0

2 1 3

6 7 8

Direction: down 5 4 3

2 1 0

6 7 8

Direction: left 5 4 3

2 0 1

8 8 8

Direction: left 5 4 3

0 2 1

6 7 8

Direction: up 0 4 3

5 2 1

6 7 8

Direction: right 4 0 3

5 2 1

6 7 8

Direction: right 4 3 0

5 2 1

6 7 8

Direction: down 4 3 1

5 2 0

6 7 8

Direction: left 4 3 1

5 0 2

6 7 8

Direction: left 4 3 1

0 5 2

6 7 8


Direction: up 0 3 1

4 5 2

6 7 8

Direction: right 3 0 1

4 5 2

6 7 8

Direction: right 3 1 0

4 5 2

6 7 8

Direction: down 3 1 2

4 5 0

6 7 8

Direction: left 3 1 2

4 0 5

6 7 8

Direction: left 3 1 2

0 4 5

6 7 8

Direction: up 0 1 2

3 4 5

6 7 8

Cost of Path: 3 Nodes Expanded: 27

Max Depth: 3

Running Time: 0 milliseconds

**A\***

**Manhattan:**

Enter 9 numbers from 0 to 8 for the initial state of the puzzle: 1 2 5 3 4 0 6 7 8

Initial State:

Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):3

solve by Euclidian (1) or by manhattan(2):2

Direction UP

1 2 0

3 4 5

6 7 8 Direction Left

1 0 2

3 4 5

6 7 8 Direction Left

0 1 2

3 4 5

6 7 8

GOAL IS REACHED

depth of the path: 3

number of nodes visited: 4 number of nodes expanded: 7 Running Time: 5 milliseconds

**Euclidean:**

Enter 9 numbers from 0 to 8 for the initial state of the puzzle: 1 2 5 3 4 0 6 7 8

Initial State:

Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A\*):3 solve by Euclidian (1) or by manhattan(2):1

Direction UP

1 2 0

3 4 5

6 7 8 Direction Left

1 0 2

3 4 5

6 7 8 Direction Left

0 1 2

3 4 5

6 7 8

GOAL IS REACHED

depth of the path: 3

number of nodes visited: 4 number of nodes expanded: 7 Running Time: 4 milliseconds
