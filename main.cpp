#include <bits/stdc++.h>

using namespace std;
using namespace chrono;

// Structure to represent the state of the puzzle
struct PuzzleState {
    vector<vector<int>> board;
    int zeroRow, zeroCol;
};

// Function to print the current state of the puzzle
void printPuzzle(const PuzzleState& state) {
    for (const auto& row : state.board) {
        for (int num : row) {
            cout << num << " ";
        }
        cout << endl;
    }
    cout << endl;
}

// Function to check if the puzzle is in the goal state
bool isGoalState(const PuzzleState& state) {
    const vector<vector<int>> goal = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
    return state.board == goal;
}

// Function to convert a PuzzleState to a string for hashing
string toString(const PuzzleState& state) {
    string result = "";
    for (const auto& row : state.board) {
        for (int num : row) {
            result += to_string(num);
        }
    }
    return result;
}

// Function to perform BFS and print the steps to reach the goal state with directions and iterations
void solvePuzzle(const PuzzleState& initialState) {
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

            cout << "Goal State Found:\n";
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

    cout << "Goal state not reachable.\n";
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

    solvePuzzle(initialState);

    return 0;
}
