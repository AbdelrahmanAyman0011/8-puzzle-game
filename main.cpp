#include <bits/stdc++.h>
using namespace std;
using namespace chrono;

const int SIZE = 3;

struct PuzzleState {
    vector<vector<int>> board;
    int zeroRow, zeroCol;
};

void printPuzzle(const PuzzleState& state) {
    for (const auto& row : state.board) {
        for (int num : row) {
            cout << num << " ";
        }
        cout << endl;
    }
    cout << endl;
}

bool isGoalState(const PuzzleState& state) {
    const vector<vector<int>> goal = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
    return state.board == goal;
}

string toString(const PuzzleState& state) {
    string result = "";
    for (const auto& row : state.board) {
        for (int num : row) {
            result += to_string(num);
        }
    }
    return result;
}
// Move the empty space (represented by 0) in the puzzle board upward.
vector<int> moveUp(const vector<int>& state) {
    // Create a copy of the current state to modify.
    vector<int> newState = state;

    // Find the iterator pointing to the position of the empty space (0) in the puzzle.
    auto it = find(newState.begin(), newState.end(), 0);

    // Calculate the index of the empty space in the puzzle board.
    int index = distance(newState.begin(), it);

    // Check if moving up is a valid move (not in the first row).
    if (index >= SIZE) {
        // Swap the empty space with the element above it in the puzzle board.
        swap(newState[index], newState[index - SIZE]);

        // Return the new state after the move.
        return newState;
    } else {
        // If moving up is not valid, return an empty vector to indicate an invalid move.
        return vector<int>();
    }
}

// Move the empty space (represented by 0) in the puzzle board downward.
vector<int> moveDown(const vector<int>& state) {
    // Create a copy of the current state to modify.
    vector<int> newState = state;

    // Find the iterator pointing to the position of the empty space (0) in the puzzle.
    auto it = find(newState.begin(), newState.end(), 0);

    // Calculate the index of the empty space in the puzzle board.
    int index = distance(newState.begin(), it);

    // Check if moving down is a valid move (not in the last row).
    if (index < SIZE * (SIZE - 1)) {
        // Swap the empty space with the element below it in the puzzle board.
        swap(newState[index], newState[index + SIZE]);

        // Return the new state after the move.
        return newState;
    } else {
        // If moving down is not valid, return an empty vector to indicate an invalid move.
        return vector<int>();
    }
}
// Move the empty space (represented by 0) in the puzzle board to the left.
vector<int> moveLeft(const vector<int>& state) {
    // Create a copy of the current state to modify.
    vector<int> newState = state;

    // Find the iterator pointing to the position of the empty space (0) in the puzzle.
    auto it = find(newState.begin(), newState.end(), 0);

    // Calculate the index of the empty space in the puzzle board.
    int index = distance(newState.begin(), it);

    // Check if moving left is a valid move (not in the leftmost column).
    if (index % SIZE != 0) {
        // Swap the empty space with the element to its left in the puzzle board.
        swap(newState[index], newState[index - 1]);

        // Return the new state after the move.
        return newState;
    } else {
        // If moving left is not valid, return an empty vector to indicate an invalid move.
        return vector<int>();
    }
}

// Move the empty space (represented by 0) in the puzzle board to the right.
vector<int> moveRight(const vector<int>& state) {
    // Create a copy of the current state to modify.
    vector<int> newState = state;

    // Find the iterator pointing to the position of the empty space (0) in the puzzle.
    auto it = find(newState.begin(), newState.end(), 0);

    // Calculate the index of the empty space in the puzzle board.
    int index = distance(newState.begin(), it);

    // Check if moving right is a valid move (not in the rightmost column).
    if ((index + 1) % SIZE != 0) {
        // Swap the empty space with the element to its right in the puzzle board.
        swap(newState[index], newState[index + 1]);

        // Return the new state after the move.
        return newState;
    } else {
        // If moving right is not valid, return an empty vector to indicate an invalid move.
        return vector<int>();
    }
}
// Definition of the Node class representing a state in the puzzle.
class Node {
public:
    vector<int> state;    // The state of the puzzle board.
    Node* parent;         // Pointer to the parent node.
    string operation;     // The operation (move) performed to reach this state.
    int depth;            // The depth of the node in the search tree.
    int cost;             // The cost associated with reaching this node.

    // Constructor to initialize the Node.
    Node(const vector<int>& state, Node* parent, const string& operation, int depth, int cost)
        : state(state), parent(parent), operation(operation), depth(depth), cost(cost) {}
};

// Function to create a new Node with given parameters.
Node* createNode(const vector<int>& state, Node* parent, const string& operation, int depth, int cost) {
    return new Node(state, parent, operation, depth, cost);
}

// Function to expand the given node and generate its child nodes.
vector<Node*> expandNode(Node* node, vector<Node*>& nodes) {
    vector<Node*> expandedNodes;

    // Create child nodes by applying possible moves (up, down, left, right).
    expandedNodes.push_back(createNode(moveUp(node->state), node, "Up", node->depth + 1, 0));
    expandedNodes.push_back(createNode(moveDown(node->state), node, "Down", node->depth + 1, 0));
    expandedNodes.push_back(createNode(moveLeft(node->state), node, "Left", node->depth + 1, 0));
    expandedNodes.push_back(createNode(moveRight(node->state), node, "Right", node->depth + 1, 0));

    // Remove nodes with an empty state (invalid moves) from the list.
    expandedNodes.erase(remove_if(expandedNodes.begin(), expandedNodes.end(), [](Node* n) {
                            return n->state.empty();
                        }), expandedNodes.end());

    return expandedNodes;
}

// Function to convert a vector of integers to a string.
string vectorToString(const vector<int>& vec) {
    string result;
    for (int num : vec) {
        result += to_string(num);
    }
    return result;
}

// Depth-First Search (DFS) algorithm to solve the puzzle.
vector<string> dfs(const vector<int>& start, const vector<int>& goal, int& nodesExpanded, int& searchDepth, double& runningTime) {
    vector<Node*> nodes;             // List to store all nodes generated during the search.
    vector<Node*> visited;           // List to store visited nodes.
    unordered_set<string> stateDict; // Set to store unique string representations of visited states.
    int depthLimit = 5000;           // Depth limit to prevent infinite search.

    stack<Node*> nodeStack;           // Stack to perform DFS.
    nodeStack.push(createNode(start, nullptr, "", 0, 0));
    auto startTime = high_resolution_clock::now(); // Record the start time of the search.

    // Main DFS loop.
    while (!nodeStack.empty()) {
        auto elapsedTime = duration_cast<seconds>(high_resolution_clock::now() - startTime);

        // Check if the search time limit (80 seconds) is not exceeded.
        if (elapsedTime.count() < 80) {
            Node* node = nodeStack.top();
            nodeStack.pop();

            string stateStr = vectorToString(node->state);

            // Check if the current state has been visited before.
            if (stateDict.find(stateStr) != stateDict.end()) {
                continue; // Skip the current node if the state has been visited.
            }

            stateDict.insert(stateStr); // Mark the current state as visited.
            visited.push_back(node);

            cout << stateStr << endl; // Print the current state (for visualization).

            // Check if the goal state is reached.
            if (node->state == goal) {
                vector<string> moves;
                Node* temp = node;

                // Backtrack to reconstruct the path to the goal.
                while (temp != nullptr) {
                    moves.insert(moves.begin(), temp->operation);
                    temp = temp->parent;
                }

                auto stopTime = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(stopTime - startTime);

                // Update statistics.
                nodesExpanded = visited.size();
                searchDepth = moves.size() - 1;
                runningTime = duration.count() / 1000.0;

                return moves; // Return the sequence of moves to reach the goal.
            }

            // Check if the depth limit is not exceeded.
            if (node->depth < depthLimit) {
                // Expand the current node and add its children to the stack.
                vector<Node*> expandedNodes = expandNode(node, nodes);
                for (Node* n : expandedNodes) {
                    if (find(visited.begin(), visited.end(), n) == visited.end()) {
                        nodeStack.push(n);
                    }
                }
            }
        } else {
            return vector<string>(); // Return an empty vector if the time limit is exceeded.
        }
    }

    return vector<string>(); // Return an empty vector if the goal state is not reachable.
}

void printDFSResults(const vector<string>& solution, int nodesExpanded, int searchDepth, double runningTime) {
    if (!solution.empty()) {
        cout << "DFS - Goal State Found:\n";
        for (const auto& move : solution) {
            cout << "Move: " << move << endl;
        }
        cout << "Path to Goal: " << solution.size() - 1 << " steps\n";
        cout << "Cost of Path: " << solution.size() - 1 << endl;
        cout << "Nodes Expanded: " << nodesExpanded << endl;
        cout << "Search Depth: " << searchDepth << endl;
        cout << "Running Time: " << runningTime << " seconds\n";
    } else {
        cout << "DFS - Goal state not reachable.\n";
    }
}

// Function to perform BFS and print the steps to reach the goal state with directions and iterations
void solveBFS(const PuzzleState& initialState) {
    auto start = high_resolution_clock::now();  // for time

    queue<pair<PuzzleState, vector<tuple<string, int, PuzzleState>>>> q;  // Tuple of direction, iteration, and PuzzleState
    unordered_set<string> visited;  // Creates a set to keep track of visited states (represented as strings).

    vector<tuple<string, int, PuzzleState>> initialSteps;  // Tuple of direction, iteration, and PuzzleState
    q.push({initialState, initialSteps});  // Pushes the initial state and its corresponding steps into the queue.
    visited.insert(toString(initialState));  // Inserts the string representation of the initial state into the set of visited states.

    int nodesExpanded = 0;

    // Define the directions for each movement
    vector<string> directions = {"left", "right", "up", "down"};

    while (!q.empty()) {
        PuzzleState current = q.front().first;
        vector<tuple<string, int, PuzzleState>> stepsSoFar = q.front().second;
        q.pop();

        if (isGoalState(current)) {
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(stop - start);

            cout << "BFS - Goal State Found:\n";
            for (const auto& step : stepsSoFar) {
                cout << "Iteration " << get<1>(step) << ":\n";
                cout << "Direction: " << get<0>(step) << endl;
                printPuzzle(get<2>(step));

            }
            cout << "Path to Goal: " << stepsSoFar.size() - 1 << " steps\n";
            cout << "Cost of Path: " << stepsSoFar.size() - 1 << endl;
            cout << "Nodes Expanded: " << nodesExpanded << endl;
            cout << "Search Depth: " << stepsSoFar.size() - 1 << endl;
            cout << "Running Time: " << duration.count()/1000.0 << " seconds\n";
            return;
        }

        nodesExpanded++;

        // Generate next states by moving the zero tile
        vector<int> dr = {0, 0, -1, 1};  // Directions: [left, right, up, down]
        vector<int> dc = {-1, 1, 0, 0};  // Directions: [left, right, up, down]

        for (int i = 0; i < 4; ++i) {
            int newRow = current.zeroRow + dr[i];
            int newCol = current.zeroCol + dc[i];

            if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {  // This condition checks if the new indices are within the valid range of the puzzle board (3x3 puzzle)
                PuzzleState nextState = current; //is a copy of the current state (current). The state is represented as a 2D vector (board), and the zero tile is moved to its new position by swapping its value with the value at the new position (newRow, newCol). This simulates the movement of the zero tile in one of the four directions.
                swap(nextState.board[current.zeroRow][current.zeroCol], nextState.board[newRow][newCol]);
                nextState.zeroRow = newRow;
                nextState.zeroCol = newCol;

                string nextStateString = toString(nextState);
                /*In summary,
                                                                     * this part of the code explores possible moves of the zero tile in different directions,
                                                                     * generates new states, and adds them to the queue for further exploration until the goal state is reached.*/

                if (visited.find(nextStateString) == visited.end()) {//This condition checks if the string representation of the new state is not in the visited set, meaning the state has not been explored before.
                    vector<tuple<string, int, PuzzleState>> nextSteps = stepsSoFar;
                    nextSteps.push_back({directions[i], nextSteps.size() + 1, nextState}); //If the state is new, it creates a new vector (nextSteps) by copying the steps taken so far (stepsSoFar) and appending a tuple containing the direction of the move (directions[i]), the iteration number (nextSteps.size() + 1), and the new state (nextState). This vector keeps track of the steps taken to reach this new state

                    q.push({nextState, nextSteps});
                    visited.insert(nextStateString);
                }
            }
        }
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    cout << "BFS - Goal state not reachable.\n";
    cout << "Running Time: " << duration.count()/1000.0 << " seconds\n";
}

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
    printPuzzle(initialState);

    cout << "Choose algorithm (1 for BFS, 2 for DFS): ";
    int choice;
    cin >> choice;

    if (choice == 1) {
        solveBFS(initialState);

    } else if (choice == 2) {
        vector<int> start_state(initialState.board.size() * initialState.board[0].size());
        for (int i = 0; i < initialState.board.size(); ++i) {
            for (int j = 0; j < initialState.board[i].size(); ++j) {
                start_state[i * SIZE + j] = initialState.board[i][j];
            }
        }

        int nodesExpanded = 0;
        int searchDepth = 0;
        double runningTime = 0.0;

        vector<string> solution = dfs(start_state, {0, 1, 2, 3, 4, 5, 6, 7, 8}, nodesExpanded, searchDepth, runningTime);
        printDFSResults(solution, nodesExpanded, searchDepth, runningTime);
    } else {
        cout << "Invalid choice.\n";
    }

    return 0;
}
