#include <algorithm>
#include <stack>
#include <climits>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <map>

/*
            Maze Solving Algorithms:
            We are testing the time efficiency between DFS, BFS, A*, and Ant Colony Optimization

            How to run:
            g++ main.cpp -o maze_runner.exe
            ./maze_runner.exe

            When prompted with file path, insert data from one of the generated datasets in the data folder.
            Remove quotations

            To test ACO, only use small perfect or small imperfect. The other algorithms, BFS, DFS, and A* work with any generated dataset

*/



using namespace std;
namespace fs = std::filesystem;

vector<string> loadGrid(const string& path) {
    ifstream in(path);
    vector<string> grid;
    string line;
    while (getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) grid.push_back(line);
    }
    return grid;
}

vector<vector<int>> buildAdj(const vector<string>& g) {
    int h = static_cast<int>(g.size());
    int w = static_cast<int>(g[0].size());
    auto id = [w](int r, int c) { return r * w + c; };
    vector<vector<int>> adj(h * w);
    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (g[r][c] == '#') continue;
            int u = id(r, c);
            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k], nc = c + dc[k];
                if (nr >= 0 && nr < h && nc >= 0 && nc < w && g[nr][nc] == '.') adj[u].push_back(id(nr, nc));
            }
        }
    }
    return adj;
}

pair<int, int> srcGoal(const vector<string>& g) {
    int h = static_cast<int>(g.size());
    int w = static_cast<int>(g[0].size());
    int src = -1, goal = -1;
    for (int r = 0; r < h && src == -1; ++r) for (int c = 0; c < w; ++c) if (g[r][c] == '.') { src = r * w + c; break; }
    for (int r = h - 1; r >= 0 && goal == -1; --r) for (int c = w - 1; c >= 0; --c) if (g[r][c] == '.') { goal = r * w + c; break; }
    if (goal == -1) goal = src;
    return {src, goal};
}

template <class F>
long long us(F&& f) {
    auto t0 = chrono::steady_clock::now();
    f();
    auto t1 = chrono::steady_clock::now();
    return chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
}

int runDfs(const vector<vector<int>>& adj, int startNode, int goalNode) {
    if (startNode < 0 || goalNode < 0) {
        return -1;
    }

    vector<char> visited(adj.size(), 0);
    vector<int> parent(adj.size(), -1);
    vector<int> stack{startNode};

    while (!stack.empty()) {
        int currentNode = stack.back();
        stack.pop_back();

        if (visited[currentNode]) {
            continue;
        }

        visited[currentNode] = 1;

        if (currentNode == goalNode) {
            int pathLength = 0;
            int node = goalNode;

            while (node != -1) {
                pathLength++;
                node = parent[node];
            }

            return pathLength;
        }

        for (int neighbor : adj[currentNode]) {
            if (!visited[neighbor]) {
                if (parent[neighbor] == -1) {
                    parent[neighbor] = currentNode;
                }
                stack.push_back(neighbor);
            }
        }
    }

    return -1;
}

int runBfs(const vector<vector<int>>& adj, int startNode, int goalNode) {
    if (startNode < 0 || goalNode < 0) {
        return -1;
    }

    vector<char> visited(adj.size(), 0);
    vector<int> parent(adj.size(), -1);
    queue<int> nodeQueue;

    nodeQueue.push(startNode);
    visited[startNode] = 1;

    while (!nodeQueue.empty()) {
        int currentNode = nodeQueue.front();
        nodeQueue.pop();

        if (currentNode == goalNode) {
            int pathLength = 0;
            int node = goalNode;

            while (node != -1) {
                pathLength++;
                node = parent[node];
            }

            return pathLength;
        }

        for (int neighbor : adj[currentNode]) {
            if (!visited[neighbor]) {
                visited[neighbor] = 1;
                parent[neighbor] = currentNode;
                nodeQueue.push(neighbor);
            }
        }
    }

    return -1;
}

vector<int> Astar(const vector<string>& g, int src, int goal) {
    vector<int> path;
    int h = static_cast<int>(g.size()), w = static_cast<int>(g[0].size()), n = h * w;
    if (src < 0 || goal < 0 || src >= n || goal >= n) return path;
    auto rc = [w](int id) { return pair<int, int>{id / w, id % w}; };
    auto h1 = [&](int a, int b) { auto [ar, ac] = rc(a); auto [br, bc] = rc(b); return abs(ar - br) + abs(ac - bc); };
    vector<int> dist(n, numeric_limits<int>::max()), parent(n, -1);
    vector<char> closed(n, 0);
    using Node = pair<int, int>;
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    int dr[4] = {-1, 1, 0, 0}, dc[4] = {0, 0, -1, 1};
    dist[src] = 0;
    pq.push({h1(src, goal), src});
    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        if (u == goal) break;
        auto [r, c] = rc(u);
        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k], nc = c + dc[k];
            if (nr < 0 || nr >= h || nc < 0 || nc >= w || g[nr][nc] == '#') continue;
            int v = nr * w + nc, nd = dist[u] + 1;
            if (nd < dist[v]) { dist[v] = nd; parent[v] = u; pq.push({nd + h1(v, goal), v}); }
        }
    }
    if (src == goal) return {src};
    if (parent[goal] == -1) return path;
    for (int at = goal; at != -1; at = parent[at]) path.push_back(at);
    reverse(path.begin(), path.end());
    return path;
}

struct Ant{
        int x, y;
        vector<pair<int,int>> path;
        vector<vector<bool>> visited;
        bool reachedGoal=false;

        Ant(int startX, int startY, int width, int height)
        : x(startX), y(startY), visited(height, vector<bool>(width, false)) {
        path.push_back({x, y});
        visited[y][x] = true;
    }
};

double heuristic(int x, int y, int goalX, int goalY) {
    return 1.0 / (abs(goalX - x) + abs(goalY - y) + 1.0);
}

bool isValid(int x, int y, const vector<string>& maze) {
    return y >= 0 && y < static_cast<int>(maze.size()) &&
           x >= 0 && x < static_cast<int>(maze[0].size()) &&
           maze[y][x] == '.';
}

string pathToString(const vector<pair<int,int>>& path) {
    string result;
    for (const auto& cell : path) {
        result += "(" + to_string(cell.first) + "," + to_string(cell.second) + ")";
    }
    return result;
}

pair<int,int> chooseNextMove(
    Ant& ant,
    const vector<string>& maze,
    const vector<vector<double>>& pheromone,
    int goalX, int goalY,
    double alpha, double beta,
    double exploreRate
) {
    vector<pair<int,int>> neighbors = {
        {ant.x + 1, ant.y},
        {ant.x - 1, ant.y},
        {ant.x, ant.y + 1},
        {ant.x, ant.y - 1}
    };

    vector<pair<int,int>> validMoves;
    vector<double> weights;
    double total = 0.0;

    pair<int,int> previous = {-1, -1};
    if (ant.path.size() >= 2) {
        previous = ant.path[ant.path.size() - 2];
    }

    for (const auto& n : neighbors) {
        int nx = n.first;
        int ny = n.second;

        if (!isValid(nx, ny, maze)) {
            continue;
        }

        // block immediate backtracking if possible
        if (nx == previous.first && ny == previous.second) {
            continue;
        }

        double tau = pow(pheromone[ny][nx], alpha);
        double eta = pow(heuristic(nx, ny, goalX, goalY), beta);
        double score = tau * eta;

        validMoves.push_back({nx, ny});
        weights.push_back(score);
        total += score;
    }

    // if no moves remain, allow backtracking as fallback
    if (validMoves.empty()) {
        for (const auto& n : neighbors) {
            int nx = n.first;
            int ny = n.second;

            if (!isValid(nx, ny, maze)) {
                continue;
            }

            double tau = pow(pheromone[ny][nx], alpha);
            double eta = pow(heuristic(nx, ny, goalX, goalY), beta);
            double score = tau * eta;

            validMoves.push_back({nx, ny});
            weights.push_back(score);
            total += score;
        }
    }

    if (validMoves.empty()) {
        return {-1, -1};
    }

    double randomChoice = static_cast<double>(rand()) / RAND_MAX;

    if (randomChoice < exploreRate) {
        int idx = rand() % validMoves.size();
        return validMoves[idx];
    }

    double r = (static_cast<double>(rand()) / RAND_MAX) * total;
    double cumulative = 0.0;

    for (int i = 0; i < static_cast<int>(validMoves.size()); i++) {
        cumulative += weights[i];
        if (r <= cumulative) {
            return validMoves[i];
        }
    }

    return validMoves.back();
}

vector<pair<int,int>> AntColonyOpt(
    const vector<string>& maze,
    int src,
    int goal,
    int numAnts,
    int iterations
) {
    int height = static_cast<int>(maze.size());
    int width = static_cast<int>(maze[0].size());

    if (src < 0 || goal < 0) return {};

    int startX = src % width;
    int startY = src / width;
    int goalX = goal % width;
    int goalY = goal / width;

    vector<vector<double>> pheromone(height, vector<double>(width, 1.0));

    double alpha = 1.0;
    double beta = 2.0;
    double evaporation = 0.4;
    double Q = 50.0;
    double exploreRate = 0.05;
    int maxSteps = 4 * (width + height);

    vector<pair<int,int>> bestPath;
    int bestLength = INT_MAX;

    srand(static_cast<unsigned>(time(0)));

    for (int iter = 0; iter < iterations; iter++) {
        vector<Ant> ants;
        for (int i = 0; i < numAnts; i++) {
            ants.emplace_back(startX, startY, width, height);
        }

        int successCount = 0;

        map<string, int> pathCounts;
        map<string, vector<pair<int,int>>> pathLookup;
        map<string, bool> pathSuccess;

        for (int i = 0; i < numAnts; i++) {
            Ant& ant = ants[i];

            for (int step = 0; step < maxSteps; step++) {
                if (ant.x == goalX && ant.y == goalY) {
                    ant.reachedGoal = true;
                    break;
                }

                pair<int,int> nextMove = chooseNextMove(
                    ant, maze, pheromone, goalX, goalY, alpha, beta, exploreRate
                );

                if (nextMove.first == -1) {
                    // Dead end -> backtrack
                    if (ant.path.size() > 1) {
                        ant.path.pop_back();
                        ant.x = ant.path.back().first;
                        ant.y = ant.path.back().second;
                    } else {
                        break;
                    }
                } else {
                    ant.x = nextMove.first;
                    ant.y = nextMove.second;
                    ant.path.push_back({ant.x, ant.y});
                    ant.visited[ant.y][ant.x] = true;
                }

                if (ant.x == goalX && ant.y == goalY) {
                    ant.reachedGoal = true;
                    successCount++;

                    if (static_cast<int>(ant.path.size()) < bestLength) {
                        bestLength = static_cast<int>(ant.path.size());
                        bestPath = ant.path;
                    }

                    break;
                }
            }

            string key = pathToString(ant.path);
            pathCounts[key]++;
            pathLookup[key] = ant.path;
            pathSuccess[key] = ant.reachedGoal;
        }

        // Evaporation happens once per iteration
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                pheromone[y][x] *= (1.0 - evaporation);
                if (pheromone[y][x] < 0.01) {
                    pheromone[y][x] = 0.01;
                }
            }
        }

        // Deposit pheromones only from the best successful ants
        vector<const Ant*> successfulAnts;
        for (const auto& ant : ants) {
            if (ant.reachedGoal) {
                successfulAnts.push_back(&ant);
            }
        }

        sort(successfulAnts.begin(), successfulAnts.end(),
            [](const Ant* a, const Ant* b) {
                return a->path.size() < b->path.size();
            });

        int eliteCount = min(3, static_cast<int>(successfulAnts.size()));

        for (int i = 0; i < eliteCount; i++) {
            double deposit = Q / successfulAnts[i]->path.size();

            for (const auto& cell : successfulAnts[i]->path) {
                int x = cell.first;
                int y = cell.second;
                pheromone[y][x] += deposit;
            }
        }

        cout << "\nIteration " << iter + 1 << "/" << iterations << endl;
        cout << "Successful ants: " << successCount << "/" << numAnts << endl;
        cout << "Best path length so far: ";
        if (bestLength == INT_MAX) {
            cout << "none yet" << endl;
        } else {
            cout << bestLength << endl;
        }

        cout << "Unique paths this iteration: " << pathCounts.size() << endl;

        int pathNumber = 1;
        for (const auto& entry : pathCounts) {
            const string& key = entry.first;
            int count = entry.second;
            const vector<pair<int,int>>& path = pathLookup[key];
            bool reachedGoal = pathSuccess[key];

            cout << "Path " << pathNumber
                 << ": used by " << count << " ant(s), "
                 << (reachedGoal ? "reached goal" : "did not reach goal")
                 << ", length = " << path.size()
                 << ", ended at (" << path.back().first << ", " << path.back().second << ")"
                 << endl;

            pathNumber++;
        }

        cout << "-----------------------------------" << endl;
    }

    return bestPath;
}

vector<string> parseStem(const fs::path& file) {
    string s = file.stem().string();
    vector<string> out;
    size_t start = 0;
    while (true) {
        size_t pos = s.find('_', start);
        if (pos == string::npos) { out.push_back(s.substr(start)); break; }
        out.push_back(s.substr(start, pos - start));
        start = pos + 1;
    }
    return out;
}

void runDirectory(const fs::path& dir) {
    vector<fs::path> files;

    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.is_regular_file() && e.path().extension() == ".txt") {
            files.push_back(e.path());
        }
    }

    sort(files.begin(), files.end());

    for (const auto& file : files) {
        auto maze = loadGrid(file.string());
        if (maze.empty()) continue;

        auto adj = buildAdj(maze);
        auto sg = srcGoal(maze);
        int src = sg.first;
        int goal = sg.second;

        auto name = parseStem(file);
        string type = name.size() > 0 ? name[0] : "unknown";
        string size = name.size() > 1 ? name[1] : "unknown";

        int dfsPathLength = -1;
        long long dfsUs = us([&] {
            dfsPathLength = runDfs(adj, src, goal);
        });

        int bfsPathLength = -1;
        long long bfsUs = us([&] {
            bfsPathLength = runBfs(adj, src, goal);
        });

        vector<int> aStarPath;
        long long astarUs = us([&] {
            aStarPath = Astar(maze, src, goal);
        });
        int aStarPathLength = aStarPath.empty() ? -1 : static_cast<int>(aStarPath.size());

        int numAnts = 20;
        int iterations = 50;
        vector<pair<int,int>> acoPath;
        long long acoUs = us([&] {
            acoPath = AntColonyOpt(maze, src, goal, numAnts, iterations);
        });
        int acoPathLength = acoPath.empty() ? -1 : static_cast<int>(acoPath.size());

        cout << file.filename().string()
             << " | perfection: " << type
             << " | size: " << size
             << " | dfs(us)=" << dfsUs
             << " | dfs path length=" << dfsPathLength
             << " | bfs(us)=" << bfsUs
             << " | bfs path length=" << bfsPathLength
             << " | astar(us)=" << astarUs
             << " | astar path length=" << aStarPathLength
             << " | aco(us)=" << acoUs
             << " | aco path length=" << acoPathLength
             << '\n';
    }
}

vector<string> loadDataFile(string input){
    fs::path p(input);  // Grabs the filepath for the size of maze user wants

    if (fs::exists(p) && fs::is_directory(p)) {     // Generates our maze
        runDirectory(p);
        return {};
    }
    auto maze = loadGrid(input);    // stores maze
    if (maze.empty()){
        cout << "Maze is empty. Try a different dataset.";
        return {};
    } 
    return maze;
}

void runMenu(const vector<string>& maze) {
    auto adj = buildAdj(maze);
    auto sg = srcGoal(maze);
    int src = sg.first;
    int goal = sg.second;

    cout << endl << "Choose your algorithm!" << endl;
    cout << "1. DFS\n2. BFS\n3. A*\n4. Ant Colony Optimization\n5. Compare all\n6. Choose new maze\n7. Exit\nSelect 1-7: ";
    int choice = 0;
    cin >> choice;

    while(choice < 1 || choice > 7){
        cout << endl << "Please choose 1-7: ";
        cin >> choice;
    }

    if (choice == 1) {
        int dfsPathLength = -1;
        long long dfsUs = us([&] {
            dfsPathLength = runDfs(adj, src, goal);
        });

        cout << "Execution time of DFS traversal: " << dfsUs << " microseconds\n";
        cout << "DFS path length: " << dfsPathLength << "\n\n";
        runMenu(maze);
    } else if (choice == 2) {
        int bfsPathLength = -1;
        long long bfsUs = us([&] {
            bfsPathLength = runBfs(adj, src, goal);
        });

        cout << "Execution time of BFS traversal: " << bfsUs << " microseconds\n";
        cout << "BFS path length: " << bfsPathLength << "\n\n";
        runMenu(maze);
    } else if (choice == 3) {
        vector<int> path;
        long long t = us([&] { path = Astar(maze, src, goal); });
        cout << "Execution time of A*: " << t << " microseconds\n";
        cout << "A* path length: " << path.size() << endl << endl;
        runMenu(maze);
    } else if (choice == 4) {
        vector<pair<int,int>> path;
        int numAnts = 0;
        int iterations = 0;

        cout << "\n\nAnt colony optimization is an algorithm modeled on the actions of an ant colony\n";
        cout << "How large is your ant colony?" << endl;
        cout << "Recommended number of ants based on maze size:" 
        << endl << "Small - 20" << endl << "Medium - 40" << endl;
        cout << "\nNumber of ants: ";
        cin >> numAnts;
        cout << "\nHow many rounds/times (iterations) would you like your ants to search?" << endl;
        cout << "Recommended iterations based on maze size:" 
        << endl << "Small - 50-75" << endl << "Medium - 100-150" << endl;
        cout << endl << "Iterations: ";
        cin >> iterations;

        long long t = us([&] {
            path = AntColonyOpt(maze, src, goal, numAnts, iterations);
        });
        cout << "\n\nExecution time of ACO: " << t << " microseconds\n";
        cout << "ACO path length: " << path.size() << endl << endl;
        runMenu(maze);
    } else if (choice == 5) {
        int numAnts = 0;
        int iterations = 0;

        cout << "\n\nAnt colony optimization is an algorithm modeled on the actions of an ant colony\n";
        cout << "How large is your ant colony?" << endl;
        cout << "Recommended number of ants based on maze size:"
            << endl << "Small - 20" << endl << "Medium - 40" << endl;
        cout << "\nNumber of ants: ";
        cin >> numAnts;

        cout << "How many rounds/times (iterations) would you like your ants to search?" << endl;
        cout << "Recommended iterations based on maze size:"
            << endl << "Small - 50-75" << endl << "Medium - 100-150" << endl;
        cout << "Iterations: ";
        cin >> iterations;

        int dfsPathLength = -1;
        long long dfsUs = us([&] {
            dfsPathLength = runDfs(adj, src, goal);
        });

        int bfsPathLength = -1;
        long long bfsUs = us([&] {
            bfsPathLength = runBfs(adj, src, goal);
        });

        vector<int> aStarPath;
        long long aStarUs = us([&] {
            aStarPath = Astar(maze, src, goal);
        });
        int aStarPathLength = aStarPath.empty() ? -1 : static_cast<int>(aStarPath.size());

        vector<pair<int,int>> acoPath;
        long long acoUs = us([&] {
            acoPath = AntColonyOpt(maze, src, goal, numAnts, iterations);
        });
        int acoPathLength = acoPath.empty() ? -1 : static_cast<int>(acoPath.size());

        cout << "\n\nExecution time of DFS traversal: " << dfsUs << " microseconds\n";
        cout << "DFS path length: " << dfsPathLength << "\n";

        cout << "Execution time of BFS traversal: " << bfsUs << " microseconds\n";
        cout << "BFS path length: " << bfsPathLength << "\n";

        cout << "Execution time of A*: " << aStarUs << " microseconds\n";
        cout << "A* path length: " << aStarPathLength << "\n";

        cout << "Execution time of ACO: " << acoUs << " microseconds\n";
        cout << "ACO path length: " << acoPathLength << "\n\n";

        runMenu(maze);
}
    else if(choice == 6){

        string input = "";
        cout << endl << "Please choose a maze type by inputting your file path." << endl;
        cout << "File Path: ";
        cin.ignore();
        getline(cin, input);

        vector<string> maze = loadDataFile(input);
        runMenu(maze);  // Algorithm choice menu for maze selection  

    }

    else if (choice == 7){
        cout << endl << "Goodbye!" << endl << endl;
        return;
    }
}

int main(int argc, char** argv) {
    string input = "";

    cout << endl << "Please choose a maze type by inputting your file path." << endl;
    cout << "File Path: ";
    getline(cin, input);

    vector<string> maze = loadDataFile(input);
    if (maze.empty()) {
        return 1;
    }
    runMenu(maze);  // Algorithm choice menu for maze selection

    return 0;
}
