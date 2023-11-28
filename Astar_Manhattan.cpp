#include<iostream>
#include<vector>
#include<map>
#include<queue>
#include<chrono>

const int N=3;

using namespace std;

class State {
public:
    vector<vector<int>> board; //3x3的棋盘
    int heuristic; // 启发式函数值h(n)
    int current_cost; // 当前代价值g(n)
    pair<int, int> zero_position; // 空格的位置

    // 重载小于运算符
    // 判断两个棋盘状态的f(n)大小
    // 在优先级队列中,f(n)的值越小，越靠近队列头部
    bool operator<(const State &other) const {
        return heuristic + current_cost > other.heuristic + other.current_cost;
    }
};

// 移动的四个方向：上、下、左、右
const int dx[] = {0, 0, -1, 1};
const int dy[] = {-1, 1, 0, 0};

//棋盘上当前位置到所有可能位置的距离(曼哈顿距离)
const int all_distance[9][9]={{0,1,2,1,2,3,2,3,4},
                              {1,0,1,2,1,2,3,2,3},
                              {2,1,0,3,2,1,4,3,2},
                              {1,2,3,0,1,2,1,2,3},
                              {2,1,2,1,0,1,2,1,2},
                              {3,2,1,2,1,0,3,2,1},
                              {2,3,4,1,2,3,0,1,2},
                              {3,2,3,2,1,2,1,0,1},
                              {4,3,2,3,2,1,2,1,0}};

// 计算当前状态的启发式函数值(曼哈顿距离)
// map: key是某个数字(1~8), value是该数字在目标状态的位置(0~8), 用于描述目标状态
int calculateHeuristic(const vector<vector<int>> &board, const map<int,int> &target_map) {
    int heuristic = 0;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (board[i][j] != 0) {
                int x = i*N+j;
                int y = target_map.at(board[i][j]);
                int dis_to_target = all_distance[x][y];
                heuristic += dis_to_target;
            }
        }
    }
    return heuristic;
}

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
void printSolution(const State &final_state, const State &initial_state, map<vector<vector<int>>, vector<vector<int>>> &cameFrom) {
    vector<vector<vector<int>>> solution;
    vector<vector<int>> current_board = final_state.board;

    //从目标状态向初始状态回溯
    while (!(current_board == initial_state.board)) {
        solution.push_back(current_board);
        current_board = cameFrom[current_board];
    }

    solution.push_back(initial_state.board);

    cout << "Solution:" << endl;
    for (int i = solution.size() - 1; i >= 0; --i) {
        for (int j = 0; j < N; ++j) {
            for (int k = 0; k < N; ++k) {
                cout << solution[i][j][k] << " ";
            }
            cout << endl;
        }
        cout << "-----" << endl;
    }
}

// A*搜索
void solvePuzzle(const vector<vector<int>> &initial_board, const map<int,int> &target_map) {
    priority_queue<State> pq;  //优先级队列
    map<vector<vector<int>>, int> costMap; //每一个状态下的代价函数f(n)
    map<vector<vector<int>>, vector<vector<int>>> cameFrom; //从哪个状态转移而来

    int zero_x,zero_y;
    for(int i=0;i<N;i++) {
        for(int j=0;j<N;j++) {
            if(initial_board[i][j] == 0) {
                zero_x = i;
                zero_y = j;
            }
        }
    }

    State startState = {initial_board, calculateHeuristic(initial_board, target_map), 0, {zero_x, zero_y}};
    pq.push(startState);
    costMap[initial_board] = 0;

    while (!pq.empty()) {
        State currentState = pq.top();
        pq.pop();

        if (isTargetState(currentState, target_map)) {
            // 打印解决方案
            cout << "Solution found!" << endl;
            printSolution(currentState, startState, cameFrom);
            return;
        }

        //考虑0位置上、下、左、右四种移动情况
        for (int k = 0; k < 4; ++k) {
            int nx = currentState.zero_position.first + dx[k];
            int ny = currentState.zero_position.second + dy[k];

            //在可以移动的情况下,将移动之后的状态加入到优先级队列
            if (nx >= 0 && nx < N && ny >= 0 && ny < N) {
                //重新构造一个State
                State nextState = currentState;

                //交换位置
                swap(nextState.board[currentState.zero_position.first][currentState.zero_position.second],
                     nextState.board[nx][ny]);

                nextState.heuristic = calculateHeuristic(nextState.board, target_map);
                nextState.current_cost = currentState.current_cost + 1;
                nextState.zero_position = {nx, ny};

                //移动之后的状态还没有出现过,或者说该状态出现过,但是此时的代价函数更小,就将其加入优先级队列
                if (costMap.find(nextState.board) == costMap.end() || nextState.current_cost < costMap[nextState.board]) {
                    pq.push(nextState);
                    costMap[nextState.board] = nextState.current_cost;
                    cameFrom[nextState.board] = currentState.board;
                }
            }
        }
    }

    cout << "No solution found." << endl;
}

int main() {
    int b[9];
    cout<<"Please input initial state:"<<endl;
    for(int & i : b) {
        int x;
        cin>>x;
        i=x;
    }
    vector<vector<int>> initialBoard = {
            {b[0], b[1], b[2]},
            {b[3], b[4], b[5]},
            {b[6], b[7], b[8]}
    };

    map<int,int> target_map;

    cout<<"Please input target state:"<<endl;
    for(int i=0;i<N*N;i++) {
        int x;
        cin>>x;
        target_map[x]=i;
    }

    auto start = chrono::high_resolution_clock::now();
    solvePuzzle(initialBoard,target_map);
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    cout << "Execution Time: " << duration.count() << " microseconds" <<endl;

    return 0;
}
