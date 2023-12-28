#include <bits/stdc++.h>
#include <conio.h>

using namespace std;
using namespace chrono;

// Structure to represent the state of the puzzle
// const that holds the state of the goal
const int goal[3][3]={0,1,2,3,4,5,6,7,8};
bool ISGOAL(int arr[3][3]){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            if(arr[i][j]!=goal[i][j]){
                return false;
            }
        }
    }
    return true;
}
//vector that map each number to (x,y) value in the goal state  like places[2] is(0,2) in the goal state
vector<pair<int, int>>places= {{0, 0}, {0, 1}, {0, 2}, {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}};

void swapn(int *a, int *b)
{  /* function to swap to
     values ie. before swap: a=2, b=3
     after swap: b=2, a=3
   */
    int temp = *a;
    *a = *b;
    *b = temp;
}
class node{
public:
    int state[3][3]; //2D array that has the tiles in it
    int x,y;        // variable to store position of 0 in the 2D matrix
    int g=0,h=0;    //variable g holds the value of g(n) and h holds value of h(n) which is number of misplaced tiles
    int f=0;          // f holds the value of f(n)
    node* parent=NULL;
    //default constructor
    node(){

    }                                                  //constructor for creating a new node
    node(node* n){
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                this->state[i][j]=n->state[i][j];
            }
        }
        this->x = n->x;
        this->y = n->y;
        this->g = n->g;
        this->f = n->f;
        this->parent = n->parent;
    }                                       //checking if 2 nodes are equal or not

public:
    bool isequal(node* n){
        if(this->x!=n->x or this->y!=n->y){
            return false;
        }
        else{
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    if(this->state[i][j]!=n->state[i][j])
                        return false;
                }
            }
            return true;
        }}                                   // function to calculate h(n) to the current node (manhattan distance)
    int calculate_h(bool ismanhattan){
        h=0;
        int value=0;
        for(int i=0;i<3;i++){
            for(int j=0;j<3;j++){
                if(state[i][j]!=goal[i][j]){
                    value=state[i][j];
                    if(ismanhattan==true){
                    h+=abs(i-places[value].first)+ abs(j-places[value].second);
                }
                else{
                    h+=sqrt(pow(i-places[value].first,2)+(pow(j-places[value].second,2)));
                    //h = sqrt((current cell:x - goal:x)2 + (current cell.y - goal:y)2)
                }
                }

            }
        }
        return h;
    }                               //function to find the least f(n) between to states
    node* min_F(node* mynode){
        return this->f < mynode->f? this : mynode;
    }


    //move left function
    node* move_left(bool ismanhattan){
        node* my_NewNode=new node(this);    // creating new node to expand

        swapn(&my_NewNode->state[x][y],&my_NewNode->state[x][y-1]);
        my_NewNode->y-=1;                       // reducing the index of y of the zero tile
        my_NewNode->h=my_NewNode->calculate_h(ismanhattan);            // recalculating the h(n)
        my_NewNode->g+=1;                      //increasing g by 1
        my_NewNode->f=my_NewNode->h+my_NewNode->g;
        my_NewNode->parent=this;
        return my_NewNode;
    }                               //function to move right
    node* move_right(bool ismanhattan){
        node* my_NewNode=new node(this);    // creating new node to expand

        swapn(&my_NewNode->state[x][y],&my_NewNode->state[x][y+1]);
        my_NewNode->y+=1;                       // reducing the index of y of the zero tile
        my_NewNode->h=my_NewNode->calculate_h(ismanhattan);            // recalculating the h(n)
        my_NewNode->g+=1;                      //increasing g by 1
        my_NewNode->f=my_NewNode->h+my_NewNode->g;
        my_NewNode->parent=this;
        return my_NewNode;
    }
    //function to move up
    node* move_up(bool ismanhattan){
        node* my_NewNode=new node(this);    // creating new node to expand

        swapn(&my_NewNode->state[x][y],&my_NewNode->state[x-1][y]);
        my_NewNode->x-=1;                       // reducing the index of y of the zero tile
        my_NewNode->h=my_NewNode->calculate_h(ismanhattan);             // recalculating the h(n)
        my_NewNode->g+=1;                      //increasing g by 1
        my_NewNode->f=my_NewNode->h+my_NewNode->g;
        my_NewNode->parent=this;
        return my_NewNode;
    }
    //function to move down
    node* move_down(bool ismanhattan){
        node* my_NewNode=new node(this);    // creating new node to expand

        swapn(&my_NewNode->state[x][y],&my_NewNode->state[x+1][y]);
        my_NewNode->x+=1;                       // reducing the index of y of the zero tile
        my_NewNode->h=my_NewNode->calculate_h(ismanhattan);             // recalculating the h(n)
        my_NewNode->g+=1;                      //increasing g by 1
        my_NewNode->f=my_NewNode->h+my_NewNode->g;
        my_NewNode->parent=this;
        return my_NewNode;
    }







};

struct compare{
    /*  It is custom comparator which helps
        us to compare two Node type objects
    */
    bool operator()(node* const& n1, node* const& n2)
    {  /*
        we are comparing two objects based on their heuristic function values ie.  f value of Node.
        this helps us in creating minimum heap
        */
        return n1->f > n2->f;
    }
};


//vector that holds all the nodes that have been visited
vector<node*>explored;
vector<node*>added; //vector to use when check in node has been addded to the frontier list or not since accessing elements in frontier is O(N) while here will be O(1)
// function to check wether this node was added to frontier or not
bool isPresentIn_Frontier(node *mynode)
{
    int size=added.size();
    for(int i=0; i<size; i++)
    {
        if(mynode->isequal(added[i]))         //checking if both nodes are equal
        {
            return true;
        }
    }
    return false;
}

// creating priority queue to hold the nodes and highest priority goes to the node with the least f(n)
priority_queue<node*,vector<node*>,compare>frontier;

//function to add the node to frontier
void AddTo_Frontier(node* n0){
    if(!isPresentIn_Frontier(n0)){
        frontier.push(n0);
        added.push_back(n0);
    }
}
void AddTo_visited(){
    explored.push_back(frontier.top());
    frontier.pop();
}
void printState(int arr[3][3]){
    cout<<"\n";
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            cout<<arr[i][j]<<" ";
        }
        cout<<"\n";
    }
}
void check_direction(int current_x,int previous_x,int current_y,int previous_y){
    if(current_x==previous_x+1){
        cout<<"Direction Down\n";
    }
    else if(current_x==previous_x-1){
        cout<<"Direction UP\n";
    }
    else if(current_y==previous_y+1){
        cout<<"Direction Right\n";
    }
    else
        cout<<"Direction Left\n";
}

void solveA(node* mynode,bool ismanhattan){
    node* current=mynode;
    auto start = high_resolution_clock::now();// start time record

    while(!ISGOAL(current->state)){
        bool UpValid=current->x-1 >= 0;                  //value to check if moving up is inbound
        bool DValid=current->x+1 <= 2;                   //value to check if moving down is inbound
        bool RValid=current->y+1 <= 2;                   //value to check if moving right inbound
        bool LValid=current->y-1 >= 0;                   //value to check if moving left is inbound

        //if the move is valid it will make a new node with the move and add it to the frontier list
        if(UpValid){AddTo_Frontier(current->move_up(ismanhattan));}
        if(DValid){AddTo_Frontier(current->move_down(ismanhattan));}
        if(RValid){AddTo_Frontier(current->move_right(ismanhattan));}
        if(LValid){AddTo_Frontier(current->move_left(ismanhattan));}
        AddTo_visited();                                         //choosing the node with least F to be added to the visited list
        current=explored[explored.size()-1];

    }
    vector<node*>path;
    while(current!=NULL){
        path.push_back(current);
        current=current->parent;
    }
    reverse(path.begin(), path.end());
    for(int i=0;i<path.size();i++){
        if(i!=0){
            int current_x=path[i]->x,previous_x=path[i]->parent->x;
            int current_y=path[i]->y,previous_y=path[i]->parent->y;
            check_direction(current_x,previous_x,current_y,previous_y);
            printState(path[i]->state);}
    }
    auto stop = high_resolution_clock::now();                 // time stop
    auto duration = duration_cast<milliseconds>(stop - start);// calc time

    cout<<"GOAL IS REACHED \n";
    cout<<"depth of the path: "<<path.size()-1 << endl ;
    cout<<"number of nodes visited: "<<explored.size() << endl;
    cout<<"number of nodes expanded: "<<added.size() << endl ;
    cout << "Running Time: " << duration.count() << " milliseconds\n";



}

//---------------------------------------------------------------------------------

// Structure to represent the state of the puzzle

struct PuzzleState {// 2d vector  and accessed on zero
    vector<vector<int>> board;
    int zeroRow, zeroCol;
};

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
                cout << "Cost of Path: " << currentNode->state.board.size() << endl;
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

        auto stop = high_resolution_clock::now();                  // stop time
        auto duration = duration_cast<milliseconds>(stop - start);// calc time

        cout << "Goal state not reachable.\n";
        cout << "Running Time: " << duration.count() << " milliseconds\n";
    }

    void recordResult(const PuzzleState &currentState, const vector<tuple<string, int, PuzzleState>> &steps) override {
    }
};

int main() {
    PuzzleState initialState;
    node* mynode=new node();
    AddTo_Frontier(mynode);
    cout << "Enter 9 numbers from 0 to 8 for the initial state of the puzzle:\n";

    for (int i = 0; i < 3; ++i) {
        vector<int> row;
        for (int j = 0; j < 3; ++j) {
            int num;
            cin >> num;
            mynode->state[i][j]=num;
            row.push_back(num);
            if (num == 0) {
                initialState.zeroRow = i;
                initialState.zeroCol = j;
                mynode->x=i;
                mynode->y=j;
            }
        }
        initialState.board.push_back(row);
    }
    AddTo_visited();
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
        cout<<"solve by Euclidian (1) or by manhattan(2): ";
        int ans;cin>>ans;
        bool ismanhattan;
        if(ans==2){
            ismanhattan=true;
        }
        else{
            ismanhattan=false;
        }
        solveA(mynode,ismanhattan);

    } else {
        cout << "Invalid choice. Exiting...\n";
    }
    cout<<"Press Enter to Continue...\n";
    _getch();

    return 0;
}
