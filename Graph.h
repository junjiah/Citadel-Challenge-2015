#ifndef GRAPH_H_
#define GRAPH_H_

#include <climits>
#include <vector>

struct neighbor {
    int target;
    int weight;
    neighbor(int arg_target, int arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

class Graph
{

private:
    vector<vector<neighbor> > graph;
    int n;

public:
    Graph(int num)
    {
        n = num;
        for (int i = 0; i < n; ++i) {
            graph.push_back(vector<neighbor>());
        }
    }

    vector<int> *operator [](int i) const
    {
        return map[i];
    }

    vector<int> *&operator [](int i)
    {
        return map[i];
    }

    inline void update_edge(int src, int dst, int weight) {
        graph[src].push_back(neighbor(dst, weight));
    }

    int dimension() {
        return n;
    }

};

#endif