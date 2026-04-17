#include <iostream>
#include <map>
#include <list>
#include <vector>
#include <queue>
#include <chrono>
#include <ctime>
#include <algorithm>
using namespace std;

// Using the chrono library: https://kahimyang.com/developer/3146/exploring-c-stdchrono-with-comprehensive-examples for elapsed time of algorithms

// https://www.geeksforgeeks.org/cpp/implementation-of-graph-in-cpp/
class Graph{
    map<int, list<int> > adjList;
};

// DFS Algorithm: https://www.geeksforgeeks.org/dsa/depth-first-search-or-dfs-for-a-graph/
void dfsRec(vector<vector<int>> &adj, vector<bool> &visited, int s, vector<int> &res){

    visited[s] = true;

    res.push_back(s);

    for(int i : adj[s]){
        if(visited[i] == false){
            dfsRec(adj, visited, i, res);
        }
    }
}

vector<int> dfs(vector<vector<int>> &adj){
    vector<bool> visited(adj.size(), false);
    vector<int> res;
    dfsRec(adj, visited, 0, res);
    return res;
}

// BFS Algorithm: https://www.geeksforgeeks.org/dsa/breadth-first-search-or-bfs-for-a-graph/
vector<int> bfs(vector<vector<int>> &adj){
    int V = adj.size();
    vector<bool> visited(V,false);
    vector<int> res;

    queue<int> q;

    int src = 0;
    visited[src] = true;
    q.push(src);

    while(!q.empty()){
        int curr = q.front();
        q.pop();
        res.push_back(curr);

        for(int x : adj[curr]){
            if(!visited[x]){
                visited[x] = true;
                q.push(x);
            }
        }
    }

    return res;

}

// Dikstra
int heuristic(Point a, Point b){
    return abs(a.x - b.x)+ (abs a.y - b.y);
}

vector<Point> Astar(){
    using Node = pair<int, Point>;
    priority_queue<Node, vector<Node>,greater<Node>> pq;
    
    map<Point, int> gCost;
    map<Point, Point> parent;
    
    gCost[start] = 0;
    pq.push({heuristic(start,goal), start});
    
    while (!pq.empty()) {
        Point curr = pq.top().second;
        pq.pop();
        
        if (curr == goal)
          return recontructPath(parent, goal);
        for (auto d : directions) {
            Point next = {curr.x + d.x, curr.y +d.y};
            
            if(!isValid(next.c, next.y)) continue;
            
            int newCost = gCost[curr] + 1;
            
            if(!gCost.count(next) || newCost < gCost[next]) {
                gCost[next] = newCost;
                int priority = newCost + heuristic(next,goal);
                pq.push({priority, next});
                parent[next] = curr;
            }
        }
    }
      return{};
}

// Ant Colony Optimization: https://projectsinventory.com/simulation-of-ant-colony-optimization-gaming-project-in-c/

// Grid Dimensions
const int WIDTH = 10;
const int HEIGHT = 10;

// Directions for movement (right, down, left, up)
const int dx[] = {0, 1, 0, -1};
const int dy[] = {1, 0, -1, 0};

class Ant{
    public:
        int x, y;
        Ant(int startX, int startY) : x(startX), y(startY) {}
        void move(){
            int direction = rand() % 4;
            x += dx[direction];
            y += dy[direction];
            x = max(0, min(WIDTH - 1, x));
            y = max(0, min(HEIGHT - 1, y));

        }
};

void AntColonyOpt(vector<Ant> ants, int numAnts){
    int startX = 0, startY = 0;
    int goalX = WIDTH - 1, goalY = HEIGHT - 1;
 
    // Initialize ants
    for (int i = 0; i < numAnts; ++i) {
        ants.emplace_back(startX, startY);
    }
 
    // Simulation loop
    for (int step = 0; step < 100; ++step) {
        cout << "Step " << step + 1 << ":" << endl;
        for (auto& ant : ants) {
            ant.move();
            cout << "Ant at (" << ant.x << ", " << ant.y << ")" << endl;
        }
 
        // Check if any ant reached the goal
        bool goalReached = false;
        for (const auto& ant : ants) {
            if (ant.x == goalX && ant.y == goalY) {
                goalReached = true;
                break;
            }
        }
 
        if (goalReached) {
            cout << "An ant has reached the goal!" << endl;
            break;
        }
 
        cout << endl;
    }
} 

int main(){

    int numAnts = 0;
    int choice = 0;

    // Displaying Menu
    cout << "*********  Algorithm Times  *********" << endl;
    cout << "1. DFS" << endl;
    cout << "2. BFS" << endl;
    cout << "3. A*" << endl;
    cout << "4. Ant Colony Optimization" << endl;
    cout << "5. Compare all results" << endl << endl;
    cout << "6. Exit" << endl;
    cout << "Select 1-5: " << endl;
    cin >> choice;
    while(choice < 1 || choice > 6){
        cout << "You must enter a number 1-6" << endl;
        cin >> choice;
    }

    switch(choice){
        case 1:
            // DFS
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of DFS traversal: " << duration.count() << endl;
        break;

        case 2:
            // BFS
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of BFS traversal: " << duration.count() << endl;
        break;

        case 3:
            // A*
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of A*: " << duration.count() << endl;
        break;

        case 4:
            // Ant Colony Optimization
            cout << "How many ants are solving the maze?" << endl;
            cout << "Number of Ants: ";
            cin >> numAnts;
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of Ant Colony Optimization: " << duration.count() << endl;
        break;

        case 5:
            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of DFS traversal: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of BFS traversal: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of A*: " << duration.count() << endl;

            auto start = std::chrono::steady_clock::now();

            auto end = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            cout << "Execution time of Ant Colony Optimization: " << duration.count() << endl;
        break;

        case 6:
            cout << "Goodbye!" << endl;
            return 0;
        break;
    }
}

