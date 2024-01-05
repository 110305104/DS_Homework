// A C++ program for Dijkstra's single source shortest path algorithm.
// The program is for adjacency matrix representation of the graph
#include<time.h>
#include <limits.h>
#include <stdio.h>
#include <iostream>
#include<vector>
using namespace std;
// Number of vertices in the graph
#define V 1000

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
    // Initialize min value
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;

    return min_index;
}

// A utility function to print the constructed distance array
void printAllSolution(int dist[], int n)
{
    printf("Vertex   Distance from Source\n");
    for (int i = 0; i < V; i++)
        printf("%d \t\t %d\n", i, dist[i]);
}

void printDistance(int dist[], int n ,int src)
{
    printf("Vertex   Distance from Source %d\n",src);
    printf("%d \t\t %d\n", n, dist[n]);
}

// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
int dijkstra(int graph[V][V], int src, int dst)
{
    int dist[V]; // The output array.  dist[i] will hold the shortest
    // distance from src to i

    bool sptSet[V]; // sptSet[i] will be true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized

    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        // Pick the minimum distance vertex from the set of vertices not
        // yet processed. u is always equal to src in the first iteration.
        int u = minDistance(dist, sptSet);

        // Mark the picked vertex as processed
        sptSet[u] = true;

        // Update dist value of the adjacent vertices of the picked vertex.
        for (int v = 0; v < V; v++)

            // Update dist[v] only if is not in sptSet, there is an edge from
            // u to v, and total weight of path from src to  v through u is
            // smaller than current value of dist[v]
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
                && dist[u] + graph[u][v] < dist[v])
                dist[v] = dist[u] + graph[u][v];
    }

    // print the constructed distance array
    //printAllSolution(dist, V);

    //printDistance(dist, dst, src);

    return dist[dst];
}
// driver program to test above function
int main()
{
    srand(time(NULL));
    /* Let us create the example graph discussed above */
    int graph_w[V][V];

    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (i == j - 1 || i == j + 1)  graph_w[i][j] = 1;
            else graph_w[i][j] = 0;
        }
    }
    graph_w[0][V-1] = 1;
    graph_w[V-1][0] = 1;

    int z;;//sample數量
    cin>>z;
    printf("Average shortest distance (𝑑) of %d source-destination pairs :\n", z);

    double START, END;
    START=clock();

    for(int t=0;t<30;t++){

        long long int total_distance = 0;
        for (int i = 0; i < z; i++)
        {
            int src = 0;
            int dst = 0;
            while (src == dst)
            {
                src = rand() % 1000;
                dst = rand() % 1000;
            }
            total_distance += dijkstra(graph_w, src, dst);
        }
        double average_distance = static_cast<double>(total_distance) / z;
        printf("%f\n",average_distance);
    }

    END=clock();
    double AvgSpendTime = ((END - START) / 30) / CLOCKS_PER_SEC;
    printf("%fsec/次\n",AvgSpendTime);

    return 0;
}