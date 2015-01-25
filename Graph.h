#ifndef GRAPH_H_
#define GRAPH_H_

#include <climits>
#include <vector>
#include <set>
#include <utility>
#include <algorithm>

struct Neighbor
{
    int target;
    int weight;
    Neighbor(int arg_target, int arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

struct Route
{
    int dist;
    std::vector<int> path;
    Route() {
        dist = INT_MAX;
    }
    Route(int arg_dist)
        : dist(arg_dist) { }
};

class Graph
{

private:
    std::vector<std::vector<Neighbor> > graph;
    int n;

public:
    Graph(int num)
    {
        n = num;
        for (int i = 0; i < n; ++i)
        {
            graph.push_back(std::vector<Neighbor>());
        }
    }

    inline void add_edge(int src, int dst, int weight)
    {
        graph[src].push_back(Neighbor(dst, weight));
    }

    /**
     * Use Dijkstra algorithm to find shortest distance
     * and route from one point to another.
     * Note: inspired by http://rosettacode.org/wiki/Dijkstra's_algorithm
     * @param  src source point
     * @param  dst destination point
     * @return     Route info with shortest distance and path
     */
    Route find_route(int src, int dst)
    {
        // init static variables
        static std::vector<bool> visited(n, false);
        static std::vector<int> prev(n, -1);
        std::fill(visited.begin(), visited.end(), false);
        std::fill(prev.begin(), prev.end(), -1);

        Route res(-1);

        std::vector<int> dist(n, INT_MAX);
        dist[src] = 0;
        std::set<std::pair<int, int> > q;
        q.insert(std::make_pair(0, src));

        while (!q.empty())
        {
            int u = q.begin()->second;
            // check dst
            if (u == dst)
            {
                res.dist = q.begin()->first;
                break;
            }
            q.erase(q.begin());
            visited[u] = true;
            for (const auto &neighbor : graph[u])
            {
                int v = neighbor.target;
                if (!visited[v])
                {
                    int new_dist = dist[u] + neighbor.weight;
                    if (new_dist < dist[v])
                    {
                        dist[v] = new_dist;
                        q.insert(std::make_pair(new_dist, v));
                        prev[v] = u;
                    }
                }
            }
        }

        // construct the path
        for (; dst != -1; dst = prev[dst])
        {
            res.path.push_back(dst);
        }
        return res;
    }

    int route_advance(Route &r)
    {
        int curr = r.path.back();
        r.path.pop_back();
        int next = r.path.back();

        auto pred = [next](const Neighbor &i) { return i.target == next; };
        auto edge = std::find_if(graph[curr].begin(),
                                 graph[curr].end(),
                                 pred);
        if (r.path.size() > 1) 
        {
            r.dist -= edge->weight;
        } else
        {
            // will reach destination in the next step,
            //  reinit dist to infinity
            r.dist = INT_MAX;
        }
        return next;
    }
};

#endif