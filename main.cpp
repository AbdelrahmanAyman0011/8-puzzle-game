#include <bits/stdc++.h>

using namespace std;
using namespace chrono;

// Structure to represent the state of the puzzle
struct PuzzleState {
    vector<vector<int>> board;
    int zeroRow, zeroCol;
};

// Base class for search algorithms
class SearchAlgorithm {
public:
    virtual void recordResult(const PuzzleState& currentState, const vector<tuple<string, int, PuzzleState>>& steps) = 0;

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
};

// Depth-First Search class
class DFS : public SearchAlgorithm {
public:
    void dfs(const PuzzleState& initialState) {
        auto start = high_resolution_clock::now();

        stack<pair<PuzzleState, vector<tuple<string, int, PuzzleState>>>> frontier;
        unordered_set<string> explored;
        PuzzleState currentState = initialState;
        vector<tuple<string, int, PuzzleState>> initialSteps;
        frontier.push({initialState, initialSteps});
        explored.insert(toString(initialState));

        int nodesExpanded = 0;

        vector<string> directions = {"left", "right", "down", "up"};  // Change the order of directions

        while (!frontier.empty()) {
            PuzzleState current = frontier.top().first;
            vector<tuple<string, int, PuzzleState>> stepsSoFar = frontier.top().second;
            frontier.pop();

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
                recordResult(current, stepsSoFar);
                return;
            }

            nodesExpanded++;
            vector<int> dr = {0, 0, -1, 1};
            vector<int> dc = {-1, 1, 0, 0};

            // Change the order of directions in this loop
            for (int i = 0; i < 4; ++i) {
                int newRow = current.zeroRow + dr[i];
                int newCol = current.zeroCol + dc[i];

                if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {
                    PuzzleState nextState = current;
                    swap(nextState.board[current.zeroRow][current.zeroCol], nextState.board[newRow][newCol]);
                    nextState.zeroRow = newRow;
                    nextState.zeroCol = newCol;

                    string nextStateString = toString(nextState);

                    if (explored.find(nextStateString) == explored.end()) {
                        vector<tuple<string, int, PuzzleState>> nextSteps = stepsSoFar;
                        nextSteps.push_back({directions[i], nextSteps.size() + 1, nextState});

                        frontier.push({nextState, nextSteps});
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

    // Implement the recordResult function for DFS
    void recordResult(const PuzzleState& currentState, const vector<tuple<string, int, PuzzleState>>& steps) override {
        cout << "Record DFS result here.\n";
        // You can use 'currentState' and 'steps' to store or display the result
    }
};

// Breadth-First Search class
class BFS : public SearchAlgorithm {
public:
    void bfs(const PuzzleState& initialState) {
        auto start = high_resolution_clock::now();

        queue<pair<PuzzleState, vector<tuple<string, int, PuzzleState>>>> q;
        unordered_set<string> visited;
        PuzzleState currentState = initialState;
        vector<tuple<string, int, PuzzleState>> initialSteps;
        q.push({initialState, initialSteps});
        visited.insert(toString(initialState));

        int nodesExpanded = 0;

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
                recordResult(current, stepsSoFar);
                return;
            }

            nodesExpanded++;

            vector<int> dr = {0, 0, -1, 1};
            vector<int> dc = {-1, 1, 0, 0};

            for (int i = 0; i < 4; ++i) {
                int newRow = current.zeroRow + dr[i];
                int newCol = current.zeroCol + dc[i];

                if (newRow >= 0 && newRow < 3 && newCol >= 0 && newCol < 3) {
                    PuzzleState nextState = current;
                    swap(nextState.board[current.zeroRow][current.zeroCol], nextState.board[newRow][newCol]);
                    nextState.zeroRow = newRow;
                    nextState.zeroCol = newCol;

                    string nextStateString = toString(nextState);

                    if (visited.find(nextStateString) == visited.end()) {
                        vector<tuple<string, int, PuzzleState>> nextSteps = stepsSoFar;
                        nextSteps.push_back({directions[i], nextSteps.size() + 1, nextState});

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

    // Implement the recordResult function for BFS
    void recordResult(const PuzzleState& currentState, const vector<tuple<string, int, PuzzleState>>& steps) override {
        cout << "Record BFS result here.\n";
        // You can use 'currentState' and 'steps' to store or display the result
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

    cout << "Choose the search algorithm (1 for BFS, 2 for DFS): ";
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
    } else {
        cout << "Invalid choice. Exiting...\n";
    }

    return 0;
}
