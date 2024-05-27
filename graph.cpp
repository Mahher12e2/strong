
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <unordered_map>

using namespace std;
//maher saker && mohammad omran
// Data structure to store the graph.
struct Graph {
  int V;  // Number of vertices in the graph.
  vector<list<int>> adj;  // Adjacency list representation of the graph.

  // Constructor to initialize the graph with V vertices.
  Graph(int V) { this->V = V; adj.resize(V); }

  // Function to add an edge to the graph.
  void addEdge(int u, int v) { adj[u].push_back(v); adj[v].push_back(u); }

  // Function to perform Gabow and Jens Schmidt's algorithm to check if two vertices u and v are strongly biconnected.
  bool areStronglyBiconnected(int u, int v) {
    // Initialize the visited array to false for all vertices.
    vector<bool> visited(V, false);

    // Initialize the articulation points array to false for all vertices.
    vector<bool> articulationPoints(V, false);

    // Initialize the preorder array to -1 for all vertices.
    vector<int> preorder(V, -1);

    // Initialize the low array to -1 for all vertices.
    vector<int> low(V, -1);

    // Initialize the parent array to -1 for all vertices.
    vector<int> parent(V, -1);

    // Initialize the time.
    int time = 0;

    // Perform DFS on the graph.
    dfs(u, -1, visited, articulationPoints, preorder, low, parent, time);

    // Check if u and v are in the same strongly connected component.
    return low[u] == low[v];
  };
// karam arab && moulla mady && mohammad alloush
private:
  // Function to perform DFS on the graph.
  void dfs(int u, int p, vector<bool>& visited, vector<bool>& articulationPoints, vector<int>& preorder, vector<int>& low, vector<int>& parent, int& time) {
    // Mark the current vertex as visited.
    visited[u] = true;

    // Set the preorder and low values for the current vertex.
    preorder[u] = time;
    low[u] = time;

    // Initialize the number of children for the current vertex.
    int children = 0;

    // Iterate over the adjacent vertices of the current vertex.
    for (auto it = adj[u].begin(); it != adj[u].end(); it++) {
      int v = *it;

      // If the adjacent vertex is not the parent of the current vertex.
      if (v != p) {
        // If the adjacent vertex is not visited.
        if (!visited[v]) {
          // Set the parent of the adjacent vertex to the current vertex.
          parent[v] = u;

          // Increment the number of children for the current vertex.
          children++;

          // Perform DFS on the adjacent vertex.
          dfs(v, u, visited, articulationPoints, preorder, low, parent, time);

          // Update the low value for the current vertex.
          low[u] = min(low[u], low[v]);

          // If the current vertex is the root and has more than one child, then it is an articulation point.
          if (p == -1 && children > 1) { articulationPoints[u] = true; }

          // If the current vertex is not the root and its low value is greater than or equal to its preorder value, then it is an articulation point.
          if (p != -1 && low[v] >= preorder[u]) { articulationPoints[u] = true; }
        } else {
          // If the adjacent vertex is visited, update the low value for the current vertex.
          low[u] = min(low[u], preorder[v]);
        }
      }
    }

    // Increment the time.
    time++;
  }
};
//zayn ahmad && abdullah alwar3a && ibrahim alean && mohammad eaqil && abdullah brham
// Driver code to test the Graph class.
int main() {
  // Create a graph with 5 vertices.
  Graph g(5);

  // Add edges to the graph.
  g.addEdge(0, 1);
  g.addEdge(0, 2);
  g.addEdge(1, 2);
  g.addEdge(2, 3);
  g.addEdge(2, 4);

  // Check if vertices 0 and 3 are strongly biconnected.
  bool areStronglyBiconnected = g.areStronglyBiconnected(0, 3);

  if (areStronglyBiconnected) {
    cout << "Vertices 0 and 3 are strongly biconnected." << endl;
  } else {
    cout << "Vertices 0 and 3 are not strongly biconnected." << endl;
  }

  return 0;
}
```