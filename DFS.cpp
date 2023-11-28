#include <iostream>
#include <set>
#include <vector>
#include<map>
#include<chrono>

using namespace std;

const int N = 3;
const int MAX_DEPTH = 50; // 限制最大递归深度

class State {
public:
    vector<vector<int>> board; //3x3的棋盘
    pair<int, int> zero_position; // 空格的位置

    //重载小于运算符
    bool operator<(const State &other) const {
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (board[i][j] != other.board[i][j])
                    return board[i][j] < other.board[i][j];
        return false;
    }
};


// 判断一个状态是否为目标状态
bool isTargetState(const State &state, const map<int,int> &target_map) {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if(target_map.at(state.board[i][j])!=i*N+j) {
                return false;
            }
        }
    }
    return true;
}


void printSolution(const State &finalState, const vector<State> &solution) {
    cout << "Solution:" << endl;
    for (State state : solution) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                cout << state.board[i][j] << " ";
            }
            cout << endl;
        }
        cout << "-----" << endl;
    }
}

bool dfs(State &currentState, set<State> &visited, vector<State> &solution, const map<int,int> &target_map,int depth) {
    if (depth > MAX_DEPTH) {
        return false;  // Maximum recursion depth reached
    }
    
    if (isTargetState(currentState, target_map)) {
        return true;
    }

    visited.insert(currentState);

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    for (int k = 0; k < 4; ++k) {
        int nx = currentState.zero_position.first + dx[k];
        int ny = currentState.zero_position.second + dy[k];

        if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
            State nextState = currentState;
            swap(nextState.board[currentState.zero_position.first][currentState.zero_position.second],
                 nextState.board[nx][ny]);

            nextState.zero_position.first = nx;
            nextState.zero_position.second = ny;

            // 如果该状态还未出现过，递归调用
            if (visited.find(nextState) == visited.end()) {
                solution.push_back(nextState);
                if (dfs(nextState, visited, solution,target_map,depth+1)) {
                    return true;
                }
                solution.pop_back();
            }
        }
    }

    return false;
}

int main() {
    int initialArray[9];
    cout<<"Please input initial state:"<<endl;
    for(int & i : initialArray) {
        int x;
        cin>>x;
        i=x;
    }
    vector<vector<int>> initialBoard = {
            {initialArray[0], initialArray[1], initialArray[2]},
            {initialArray[3], initialArray[4], initialArray[5]},
            {initialArray[6], initialArray[7], initialArray[8]}
    };

    map<int,int> target_map;

    cout<<"Please input target state:"<<endl;
    for(int i=0;i<N*N;i++) {
        int x;
        cin>>x;
        target_map[x]=i;
    }

    auto start = chrono::high_resolution_clock::now();
    int zero_x,zero_y;
    for(int i=0;i<N;i++) {
        for(int j=0;j<N;j++) {
            if(initialBoard[i][j] == 0) {
                zero_x = i;
                zero_y = j;
            }
        }
    }

    State initialState = {initialBoard, {zero_x, zero_y}};

    //深度优先搜索记录解决方案时，使用vector即可，无法达到目标状态时，回溯即可
    vector<State> solution;

    set<State> visited;

    solution.push_back(initialState);
    dfs(initialState, visited, solution, target_map,0);

    if (!solution.empty()) {
        printSolution(initialState, solution);
    } else {
        cout << "No solution found." << endl;
    }

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    cout << "Execution Time: " << duration.count() << " microseconds" <<endl;

    return 0;
}