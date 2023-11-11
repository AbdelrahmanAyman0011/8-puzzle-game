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

vector<int> move_up(const vector<int>& state) {
    vector<int> new_state = state;
    auto it = find(new_state.begin(), new_state.end(), 0);
    int index = distance(new_state.begin(), it);
    if (index >= SIZE) {
        swap(new_state[index], new_state[index - SIZE]);
        return new_state;
    } else {
        return vector<int>();
    }
}

vector<int> move_down(const vector<int>& state) {
    vector<int> new_state = state;
    auto it = find(new_state.begin(), new_state.end(), 0);
    int index = distance(new_state.begin(), it);
    if (index < SIZE * (SIZE - 1)) {
        swap(new_state[index], new_state[index + SIZE]);
        return new_state;
    } else {
        return vector<int>();
    }
}

vector<int> move_left(const vector<int>& state) {
    vector<int> new_state = state;
    auto it = find(new_state.begin(), new_state.end(), 0);
    int index = distance(new_state.begin(), it);
    if (index % SIZE != 0) {
        swap(new_state[index], new_state[index - 1]);
        return new_state;
    } else {
        return vector<int>();
    }
}

vector<int> move_right(const vector<int>& state) {
    vector<int> new_state = state;
    auto it = find(new_state.begin(), new_state.end(), 0);
    int index = distance(new_state.begin(), it);
    if ((index + 1) % SIZE != 0) {
        swap(new_state[index], new_state[index + 1]);
        return new_state;
    } else {
        return vector<int>();
    }
}

class Node {
public:
    vector<int> state;
    Node* parent;
    string operation;
    int depth;
    int cost;

    Node(const vector<int>& state, Node* parent, const string& operation, int depth, int cost)
        : state(state), parent(parent), operation(operation), depth(depth), cost(cost) {}
};

Node* create_node(const vector<int>& state, Node* parent, const string& operation, int depth, int cost) {
    return new Node(state, parent, operation, depth, cost);
}

vector<Node*> expand_node(Node* node, vector<Node*>& nodes) {
    vector<Node*> expanded_nodes;
    expanded_nodes.push_back(create_node(move_up(node->state), node, "Up", node->depth + 1, 0));
    expanded_nodes.push_back(create_node(move_down(node->state), node, "Down", node->depth + 1, 0));
    expanded_nodes.push_back(create_node(move_left(node->state), node, "Left", node->depth + 1, 0));
    expanded_nodes.push_back(create_node(move_right(node->state), node, "Right", node->depth + 1, 0));
    expanded_nodes.erase(remove_if(expanded_nodes.begin(), expanded_nodes.end(), [](Node* n) {
                             return n->state.empty();
                         }), expanded_nodes.end());
    return expanded_nodes;
}

string vector_to_string(const vector<int>& vec) {
    string result;
    for (int num : vec) {
        result += to_string(num);
    }
    return result;
}

vector<string> dfs(const vector<int>& start, const vector<int>& goal, int& nodesExpanded, int& searchDepth, double& runningTime) {
    vector<Node*> nodes;
    vector<Node*> visited;
    unordered_set<string> state_dict;
    int depth_limit = 5000; // ential value ..

    stack<Node*> nodeStack; // Use a stack instead of recursion
    nodeStack.push(create_node(start, nullptr, "", 0, 0));
    auto start_time = high_resolution_clock::now();

    while (!nodeStack.empty()) {
        auto elapsed_time = duration_cast<seconds>(high_resolution_clock::now() - start_time);
        if (elapsed_time.count() < 80) {
            Node* node = nodeStack.top();
            nodeStack.pop();

            string state_str = vector_to_string(node->state);
            if (state_dict.find(state_str) != state_dict.end()) {
                continue;
            }
            state_dict.insert(state_str);
            visited.push_back(node);

            cout << state_str << endl;

            if (node->state == goal) {
                vector<string> moves;
                Node* temp = node;
                while (temp != nullptr) {
                    moves.insert(moves.begin(), temp->operation);
                    temp = temp->parent;
                }

                auto stop_time = high_resolution_clock::now();
                auto duration = duration_cast<milliseconds>(stop_time - start_time);

                nodesExpanded = visited.size();
                searchDepth = moves.size() - 1;
                runningTime = duration.count() / 1000.0;

                return moves;
            }

            if (node->depth < depth_limit) {
                vector<Node*> expanded_nodes = expand_node(node, nodes);
                for (Node* n : expanded_nodes) {
                    if (find(visited.begin(), visited.end(), n) == visited.end()) {
                        nodeStack.push(n);
                    }
                }
            }
        } else {
            return vector<string>();
        }
    }

    return vector<string>();
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
                printPuzzle(get<2>(step));
                cout << "Direction: " << get<0>(step) << endl;
            }
            cout << "Path to Goal: " << stepsSoFar.size() - 1 << " steps\n";
            cout << "Cost of Path: " << stepsSoFar.size() - 1 << endl;
            cout << "Nodes Expanded: " << nodesExpanded << endl;
            cout << "Search Depth: " << stepsSoFar.size() - 1 << endl;
            cout << "Running Time: " << duration.count() << " milliseconds\n";
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
    cout << "Running Time: " << duration.count() << " milliseconds\n";
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
