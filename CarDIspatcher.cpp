#include "RouteLib.h"
#include "CarDispatcher.h"
#include <iostream>
#include <climits>

CarDispatcher::CarDispatcher(
    std::vector<IntersectionInfo> intersections,
    std::vector<RoadInfo> roads)
{
    int n = intersections.size();

    // build graph with adjency list
    graph = new Graph(n);
    for (auto r : roads)
    {
        int src = r.src_intersection_id,
            dst = r.dst_intersection_id;
        graph->add_edge(src, dst, r.weight);
    }

    std::cout << "Graph init done." << std::endl;

}

void CarDispatcher::onTurn(
    std::vector<CarCtl> &cars_at_intersections,
    const std::vector<PassengerRequest> &passenger_requests)
{
    printf("car num: %lu, passenger num: %lu\n", cars_at_intersections.size(),
           passenger_requests.size());

    int i;
    std::cin >> i;

    // process loaded cars to next intersection,
    // then remove from the vector
    auto car_iter = cars_at_intersections.begin();
    while (car_iter != cars_at_intersections.end())
    {
        if (car_iter->passenger_id != -1)
        {
            std::vector<int> &path = car_route[car_iter->car_id];
            car_iter->intersection_id = path.back();
            path.pop_back();
            cars_at_intersections.erase(car_iter++);
        }
        else
        {
            ++car_iter;
        }
    }

    // for each passenger, calculate nearest vehicle
    for (auto p : passenger_requests)
    {
        Route *best_route;
        CarCtl *nearest_car;
        int min_dist = INT_MAX;
        for (auto car : cars_at_intersections)
        {
            Route r = graph->find_route(car.intersection_id,
                                        p.src_intersection_id);
            if (r.dist < min_dist)
            {
                best_route = &r;
                nearest_car = &car;
                min_dist = r.dist;
            }
            // break if ready to pick up
            if (!min_dist)
                break;
        }

        // nearest_car is NULL if no cars available
        if (!nearest_car)
        {
            break;
        }

        // pick up!
        if (!min_dist)
        {
            nearest_car->passenger_id = p.passenger_id;
            Route r = graph->find_route(p.src_intersection_id,
                                        p.dst_intersection_id);
            nearest_car->intersection_id = r.path.back();
            r.path.pop_back();
            car_route[nearest_car->car_id] = r.path;
        }
        else // approach the passenger
        {
            nearest_car->intersection_id = best_route->path[0];
        }
    }
}