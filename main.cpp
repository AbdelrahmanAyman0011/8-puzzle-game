#include <bits/stdc++.h>

using namespace std;
using namespace chrono;

// Structure to represent the state of the puzzle
struct PuzzleState {// 2d vector  and accessed on zero
    vector<vector<int>> board;
    int zeroRow, zeroCol;
};

// Node2 structure to hold puzzle states for A* search
struct Node2 {
    PuzzleState state;  // Represents a state in the puzzle
    Node2 * parent;       // Pointer to the parent node
    string action;      // Action taken to reach this node from its parent
    int cost;           // Cost associated with reaching this node

    bool operator<(const Node2 & other) const {
        return cost > other.cost; // Comparison for priority queue in A* search
    }
};

class AStar {
public:
    // Heuristic: Calculates Manhattan distance to estimate distance to goal state
    int calculateManhattanHeuristic(const PuzzleState& state) {
        int distance = 0; //Initializes the variable distance to store the total Manhattan distance.
        // Loop through the puzzle board to compute Manhattan distance for each tile
        for (int i = 0; i < state.board.size(); ++i) {
            for (int j = 0; j < state.board[i].size(); ++j) {
                int value = state.board[i][j];
                if (value != 0) { // ---------- to get his idx ----------- //
                    int goalRow = (value - 1) / 3; // Assuming goal state is {{1, 2, 3}, {4, 5, 6}, {7, 8, 0}}
                    int goalCol = (value - 1) % 3;
                    distance += abs(i - goalRow) + abs(j - goalCol);
                }/* For goalRow: (5 - 1) / 3 = 4 / 3 = 1, which means the tile with value 5 should ideally be in the second row (index 1) in the goal state.
For goalCol: (5 - 1) % 3 = 4 % 3 = 1, which means the tile with value 5 should ideally be in the second column (index 1) in the goal state.*/
            }
        }
        return distance;
    }

    // Heuristic: Calculates Euclidean distance to estimate distance to goal state
    double calculateEuclideanHeuristic(const PuzzleState& state) {
        double distance = 0.0;
        // Loop through the puzzle board to compute Euclidean distance for each tile
        for (int i = 0; i < state.board.size(); ++i) {
            for (int j = 0; j < state.board[i].size(); ++j) {
                int value = state.board[i][j];
                if (value != 0) {
                    int goalRow = (value - 1) / 3; // Assuming goal state is {{1, 2, 3}, {4, 5, 6}, {7, 8, 0}}
                    int goalCol = (value - 1) % 3;
                    distance += sqrt(pow(i - goalRow, 2) + pow(j - goalCol, 2));
                }
            }
        }
        return distance;
    }

    // A* search algorithm
    void aStar(const PuzzleState& initialState, bool useEuclidean) {
        auto start = high_resolution_clock::now();

        priority_queue<Node2> frontier;     // Priority queue to store nodes for exploration
        unordered_set<string> explored;    // Set to track explored states
        Node2 startNode{initialState, nullptr, "", 0}; // Start node with initial state and cost 0
        // Set the initial cost using the selected heuristic
        startNode.cost = useEuclidean ? calculateEuclideanHeuristic(initialState) : calculateManhattanHeuristic(initialState);
        frontier.push(startNode); // Push start node to the priority queue
        explored.insert(toString(initialState)); // Mark initial state as explored

        // Directions for moving the zero tile (left, right, up, down)
        vector<string> directions = {"left", "right", "up", "down"};
        vector<int> dr = {0, 0, -1, 1}; // Changes in row for each direction
        vector<int> dc = {-1, 1, 0, 0}; // Changes in column for each direction

        int iteration = 0; // Counter for iterations in A* search

        // Main A* search loop
        while (!frontier.empty()) {
            Node2 currentNode = frontier.top(); // Get the node with the lowest cost
            frontier.pop();

            cout << "Iteration: " << iteration++ << endl;

            // Check if the current state is the goal state
            if (isGoalState(currentNode.state)) {
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(stop - start);

                cout << "Goal State Found:\n";
                recordResult(currentNode.state, {}); // Record the result if needed
                return;
            }

            // Explore possible moves (left, right, up, down)
            for (int i = 0; i < 4; ++i) {
                int newRow = currentNode.state.zeroRow + dr[i]; // Calculate new row for the zero tile
                int newCol = currentNode.state.zeroCol + dc[i]; // Calculate new column for the zero tile

                // Check if the new position is within the puzzle boundaries
                if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {
                    PuzzleState nextState = currentNode.state; // Create a new state for exploration
                    // Swap the zero tile with the new position
                    swap(nextState.board[currentNode.state.zeroRow][currentNode.state.zeroCol],
                         nextState.board[newRow][newCol]);
                    nextState.zeroRow = newRow; // Update the zero tile's new row
                    nextState.zeroCol = newCol; // Update the zero tile's new column

                    string nextStateString = toString(nextState); // Convert the state to a string representation

                    // Check if the state has not been explored
                    if (explored.find(nextStateString) == explored.end()) {
                        Node2 nextNode{nextState, &currentNode, directions[i], 0}; // Create the next node
                        // Set the cost using the selected heuristic
                        nextNode.cost = currentNode.cost - (useEuclidean ? calculateEuclideanHeuristic(currentNode.state) : calculateManhattanHeuristic(currentNode.state)) + 1 + (useEuclidean ? calculateEuclideanHeuristic(nextState) : calculateManhattanHeuristic(nextState));
                        frontier.push(nextNode); // Add the next node to the priority queue
                        explored.insert(nextStateString); // Mark the state as explored

                        cout << "Direction: " << directions[i] << endl;
                        printPuzzle(nextState); // Print the puzzle state for visualization
                    }
                }
            }
        }

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);

        cout << "Goal state not reachable.\n";
        cout << "Running Time: " << duration.count() << " milliseconds\n";
    }


    void recordResult(const PuzzleState &currentState, const vector<tuple<string, int, PuzzleState>> &steps) {
        // Record the result if needed
    }

    bool isGoalState(const PuzzleState &state) {
        const vector<vector<int>> goal = {{1, 2, 3}, {4, 5, 6}, {7, 8, 0}};
        return state.board == goal;
    }

    string toString(const PuzzleState &state) {
        string result = "";
        for (const auto &row : state.board) {
            for (int num : row) {
                result += to_string(num);
            }
        }
        return result;
    }

    void printPuzzle(const PuzzleState &state) {
        for (const auto &row: state.board) {
            for (int num: row) {
                cout << num << " ";
            }
            cout << endl;
        }
        cout << endl;
    }
};

//---------------------------------------------------------------------------------

// Structure to represent the state of the puzzle


struct Node {
    PuzzleState state;
    Node *parent; // This member is a pointer to the parent node. It refers to the node that led to the current node during the search process. It helps in reconstructing the path to the solution once the goal state is found
    string action;// his member stores the action taken from the parent node to reach the current node. It represents the action, such as a direction or movement, that led from the parent node to the current node.
};

// Base class for search algorithms
class SearchAlgorithm {
public:
    virtual void recordResult(const PuzzleState &currentState, const vector<tuple<string, int, PuzzleState>> &steps) = 0;

    bool isGoalState(const PuzzleState &state) {// chick  if it is a goal or not
        const vector<vector<int>> goal = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
        return state.board == goal;
    }

    string toString(const PuzzleState &state) {
        string result = "";
        for (const auto &row: state.board) {
            for (int num: row) {
                result += to_string(num);//print as a string
            }
        }
        return result;
    }
};

void printPuzzle(const PuzzleState &state) {
    for (const auto &row: state.board) {
        for (int num: row) {
            cout << num << " ";
        }
        cout << endl;
    }
    cout << endl;
}

// Depth-First Search class
class DFS : public SearchAlgorithm {
public:
    int maxDepth;

    DFS() : maxDepth(0) {}// constructor -- when we go depper the max depth will be updated

    void dfs(const PuzzleState &initialState) {

        auto start = high_resolution_clock::now();// start time record

        stack<Node *> st;

        unordered_set<string> explored;// The purpose of this set is to store string representations of the states that have already been explored

        Node *startNode = new Node{initialState, nullptr, ""};// start node doesn't have parents and no actions taken

        st.push(startNode);

        explored.insert(toString(initialState));// add explored to set.

        int nodesExpanded = 0;// counter

        vector<string> directions = {"left", "right", "up", "down"};// directions

        while (!st.empty()) {

            Node *currentNode = st.top();//

            st.pop();//  After retrieving the current node, it is removed from the stack since it's going to be processed for exploration

            if (isGoalState(currentNode->state)) {                        // if crrentnode is state
                auto stop = high_resolution_clock::now();                 // time stop
                auto duration = duration_cast<milliseconds>(stop - start);// calc time

                cout << "Goal State Found:\n";
                printSolution(currentNode);
                cout << "Cost of Path: " << currentNode->state.board.size() - 1 << endl;
                cout << "Nodes Expanded: " << nodesExpanded << endl;
                cout << "Max Depth: " << maxDepth << endl;
                cout << "Running Time: " << duration.count() << " milliseconds\n";
                recordResult(currentNode->state, {});
                return;
            }

            nodesExpanded++;

            if (static_cast<int>(currentNode->state.board.size()) > maxDepth) {
                maxDepth = currentNode->state.board.size();
            }

            if (maxDepth > 50) {// here i limit the depth
                break;
            }

            vector<int> dr = {0, 0, -1, 1};// directions for rows
            vector<int> dc = {-1, 1, 0, 0};// directions for col

            for (int i = 3; i >= 0; --i) {

                int newRow = currentNode->state.zeroRow + dr[i];// the corrent pos for row
                int newCol = currentNode->state.zeroCol + dc[i];// the corrent pos for col

                if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {// check if it in range

                    PuzzleState nextState = currentNode->state;
                    swap(nextState.board[currentNode->state.zeroRow][currentNode->state.zeroCol],
                         nextState.board[newRow][newCol]);
                    nextState.zeroRow = newRow;
                    nextState.zeroCol = newCol;

                    string nextStateString = toString(nextState);

                    if (explored.find(nextStateString) == explored.end()) {// checks if the nextState has not been explored
                        Node *nextNode = new Node{nextState, currentNode, directions[i]};
                        st.push(nextNode);
                        explored.insert(nextStateString);
                    }
                }
            }
        }

        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);

        cout << "Goal state not reachable.\n";
        cout << "Running Time: " << duration.count() << " milliseconds\n";
    }

    void recordResult(const PuzzleState &currentState, const vector<tuple<string, int, PuzzleState>> &steps) override {
    }

    // Helper function to print the solution
    void printSolution(Node *node) {
        stack<Node *> solutionStack;
        while (node != nullptr) {
            solutionStack.push(node);
            node = node->parent;
        }

        while (!solutionStack.empty()) {
            Node *stepNode = solutionStack.top();
            solutionStack.pop();
            if (!stepNode->action.empty()) {
                cout << "Direction: " << stepNode->action << endl;
                printPuzzle(stepNode->state);
            }
        }
    }
};

// Breadth-First Search class
class BFS : public SearchAlgorithm {
public:
    void bfs(const PuzzleState &initialState) {   // BFS algorithm takes the initial state of the puzzle as input and explores its states using a queue-based approach
        auto start = high_resolution_clock::now();// record starting time


        queue<pair<PuzzleState, vector<tuple<string, int, PuzzleState>>>> q;// vector contain tuple --> string (directions) --> int (setp number) --> puzzlestate (represnting ther state after taking the corresponding step)

        unordered_set<string> visited;// set save visited states as strings

        PuzzleState currentState = initialState;// initializes the current state

        vector<tuple<string, int, PuzzleState>> initialSteps;// stores the initial steps as an empty vector

        q.push({initialState, initialSteps});// Pushes the initial state and its steps into the queue.

        visited.insert(toString(initialState));// Marks the initial state as visited

        int nodesExpanded = 0;// counter

        vector<string> directions = {"left", "right", "up", "down"};// directions i saved in  vector

        while (!q.empty()) {// stop if goal state found

            PuzzleState current = q.front().first;// get the front (PazzleState)

            vector<tuple<string, int, PuzzleState>> stepsSoFar = q.front().second;// steps taken
            q.pop();                                                              // Removes the front element (which has already been accessed and stored in stepsSoFar) from the queue.

            if (isGoalState(current)) {
                auto stop = high_resolution_clock::now();                 // stop time counter
                auto duration = duration_cast<milliseconds>(stop - start);// calc it

                cout << "Goal State Found:\n";
                for (const auto &step: stepsSoFar) {
                    cout << "Iteration " << get<1>(step) << ":\n";// int steps taken
                    cout << "Direction: " << get<0>(step) << endl;// string direction
                    printPuzzle(get<2>(step));                    // pazzle state
                }
                cout << "Path to Goal: " << stepsSoFar.size() - 1 << " steps\n";
                cout << "Cost of Path: " << stepsSoFar.size() - 1 << endl;
                cout << "Nodes Expanded: " << nodesExpanded << endl;
                cout << "Search Depth: " << stepsSoFar.size() - 1 << endl;
                cout << "Running Time: " << duration.count() << " milliseconds\n";
                recordResult(current, stepsSoFar);
                return;
            }

            nodesExpanded++;// counter increased

            vector<int> dr = {0, 0, -1, 1};// direction by row
            vector<int> dc = {-1, 1, 0, 0};// direction by col

            for (int i = 0; i < 4; ++i) {
                int newRow = current.zeroRow + dr[i];
                int newCol = current.zeroCol + dc[i];

                if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {
                    PuzzleState nextState = current;// create a copy of the current state

                    swap(nextState.board[current.zeroRow][current.zeroCol], nextState.board[newRow][newCol]);// swap old zero pos to new pos

                    nextState.zeroRow = newRow;// save it as new
                    nextState.zeroCol = newCol;// save it as new

                    string nextStateString = toString(nextState);// convert it to strig to print it

                    if (visited.find(nextStateString) == visited.end()) {
                        vector<tuple<string, int, PuzzleState>> nextSteps = stepsSoFar;
                        nextSteps.push_back({directions[i], nextSteps.size() + 1, nextState});

                        q.push({nextState, nextSteps});
                        visited.insert(nextStateString);
                    }
                }
            }
        }

        auto stop = high_resolution_clock::now();                 // stop time
        auto duration = duration_cast<milliseconds>(stop - start);// calc time

        cout << "Goal state not reachable.\n";
        cout << "Running Time: " << duration.count() << " milliseconds\n";
    }

    void recordResult(const PuzzleState &currentState, const vector<tuple<string, int, PuzzleState>> &steps) override {
    }
};

int main() {
    PuzzleState initialState;
    cout << "Enter 9 numbers from 0 to 8 for the initial state of the puzzle:\n";

    for (int i = 0; i < 3; ++i) {
        vector<int> row;
        for (int j = 0; j < 3; ++j) {
            int num;
            cin >> num;
            row.push_back(num);
            if (num == 0) {
                initialState.zeroRow = i;
                initialState.zeroCol = j;
            }
        }
        initialState.board.push_back(row);
    }

    cout << "Initial State:\n";

    cout << "Choose the search algorithm (1 for BFS, 2 for DFS, 3 for A*): ";
    int choice;
    cin >> choice;

    if (choice == 1) {
        BFS bfsSolver;
        cout << "BFS:\n";
        bfsSolver.bfs(initialState);
    } else if (choice == 2) {
        DFS dfsSolver;
        cout << "DFS:\n";
        dfsSolver.dfs(initialState);
    } else if (choice == 3) {
        AStar astarSolver;

        // Assuming 'initialState' is defined earlier

        // Using Manhattan heuristic
        cout << "A* Search with Manhattan Heuristic:\n";
        astarSolver.aStar(initialState, false); // 'false' indicates using the Manhattan heuristic

        // Using Euclidean heuristic
        cout << "\nA* Search with Euclidean Heuristic:\n";
        astarSolver.aStar(initialState, true); //
    } else {
        cout << "Invalid choice. Exiting...\n";
    }

    return 0;
}
