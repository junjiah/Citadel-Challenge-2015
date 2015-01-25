#ifndef GRAPH_H_
#define GRAPH_H_

#include <climits>
#include <vector>
#include <set>
#include <utility>
#include <unordered_map>
#include <algorithm>

/**
 * Vertex in the graph.
 */
struct Neighbor
{
    int target;
    int weight;
    Neighbor(int arg_target, int arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

/**
 * Route info containing distance and the path.
 */
struct Route
{
    int dist;
    std::vector<int> path;

    Route() {
        dist = INT_MAX;
    }

    Route(int arg_dist)
        : dist(arg_dist) { }

    int next_intersection() {
        return path[path.size()-2];
    }        
};

/**
 * Assignment of cars targeting specific passengers,
 * with route information.
 */
struct Assignment
{
    CarCtl *car;
    int passenger_id;
    Route route;
    Assignment(CarCtl *arg_car, int arg_passenger_id, Route arg_route) :
    car(arg_car), passenger_id(arg_passenger_id), route(arg_route) { }
};

struct pairhash {
public:
    template <typename T, typename U>
    std::size_t operator()(const std::pair<T, U> &x) const
    {
        return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
};

typedef std::pair<int, int> src_dst;

class Graph
{

private:
    std::vector<std::vector<Neighbor> > graph;
    std::unordered_map<src_dst, int, pairhash> cars_on_road;
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

    /**
     * Move forward according to the path.
     * @param  r route info with distance and path
     * @return   next step
     */
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
            // will reach the destination in the next step,
            //  reinitialize dist to infinity
            r.dist = INT_MAX;
        }
        // cars on the way, record into congestion map
        cars_on_road[src_dst(curr, next)] += 1;
        return next;
    }

    /**
     * Find how many cars on the road src->dst
     * @param  src source of the road
     * @param  dst destination of the road
     * @return     car number on the road
     */
    int congestion(int src, int dst)
    {
        static src_dst road_prob;
        road_prob.first = src;
        road_prob.second = dst;

        auto search_result = cars_on_road.find(road_prob);
        if (search_result != cars_on_road.end())
            return search_result->second;
        else 
            return 0;
    }

    /**
     * Reduce car number on the corresponding road.
     * @param i one car's intersection number
     */
    void car_arrived(int i) 
    {
        for (auto&& kv : cars_on_road) 
        {
            if (kv.first.second == i) 
            {
                kv.second -= 1;
                return;
            }
        }
    }
};

#endif