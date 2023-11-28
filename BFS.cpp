#include <iostream>
#include<map>
#include <queue>
#include <set>
#include<chrono>

using namespace std;

const int N = 3;

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

//打印解决方案
void printSolution(const State &finalState, const map<State, State> &cameFrom) {
    vector<State> solution;
    State currentState = finalState;

    while (cameFrom.find(currentState) != cameFrom.end()) {
        solution.push_back(currentState);
        currentState = cameFrom.at(currentState);
    }

    solution.push_back(currentState);

    cout << "Solution:" << endl;
    for (int i = solution.size() - 1; i >= 0; --i) {
        for (int j = 0; j < N; ++j) {
            for (int k = 0; k < N; ++k) {
                cout << solution[i].board[j][k] << " ";
            }
            cout << endl;
        }
        cout << "-----" << endl;
    }
}


void bfs(const State &initialState, const map<int,int> &target_map) {
    queue<State> q; //待扩展的状态
    set<State> visited; //保存已遍历过的节点的集合
    map<State, State> cameFrom; //从哪个状态转移而来

    q.push(initialState);
    visited.insert(initialState);

    while (!q.empty()) {
        State currentState = q.front();
        q.pop();

        if (isTargetState(currentState,target_map)) {
            cout<<"Solution found!"<<endl;
            // 打印解决方案
            printSolution(currentState, cameFrom);
            return;
        }

        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};

        for (int k = 0; k < 4; ++k) {
            int nx = currentState.zero_position.first + dx[k];
            int ny = currentState.zero_position.second + dy[k];

            if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
                State nextState = currentState; //再构造一个状态
                swap(nextState.board[currentState.zero_position.first][currentState.zero_position.second],
                     nextState.board[nx][ny]);

                nextState.zero_position.first = nx;
                nextState.zero_position.second = ny;

                if (visited.find(nextState) == visited.end()) {
                    q.push(nextState);
                    visited.insert(nextState);
                    cameFrom[nextState] = currentState;
                }
            }
        }
    }

    cout << "No solution found." << endl;
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

    bfs(initialState,target_map);
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    cout << "Execution Time: " << duration.count() << " microseconds" <<endl;

    return 0;
}
