using namespace std;
#include <iostream>
#include <map>
#include <list>
#include <vector>


// https://www.geeksforgeeks.org/cpp/implementation-of-graph-in-cpp/
class Graph{
    map<int, list<int> > adjList;
};


// DFS Algorithm
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

// BFS Algorithm



// Dikstra
//A* - implementing this for heuristic-based optimization
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



