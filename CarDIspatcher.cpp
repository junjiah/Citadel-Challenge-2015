#include "RouteLib.h"
#include "CarDispatcher.h"
#include <iostream>
#include <climits>

CarDispatcher::CarDispatcher(
    std::vector<IntersectionInfo> intersections,
    std::vector<RoadInfo> roads)
{
    int n = intersections.size();

    graph = new Graph(n);
    for (auto r : roads) {
        int i = r.src_intersection_id,
            j = r.dst_intersection_id;
        graph->update_edge(i, j, r.weight);
    }


    dist = new SrcDestMap(n);
    next = new SrcDestMap(n);
    // init dist
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if (i == j)
                (*dist)[i][j] = 0;
            else
                (*dist)[i][j] = INT_MAX;

            (*next)[i][j] = -1;
        }
    }
    std::cout << "DIST, NEXT init over." << std::endl;

    /*
        Compute distance and path-next info.
    */

    // using Floyd Warshall algorithm
    for (auto r : roads)
    {
        int i = r.src_intersection_id,
            j = r.dst_intersection_id;
        (*dist)[i][j] = r.weight;
        (*next)[i][j] = j;
    }
    std::cout << "DIST, NEXT updated by edges." << std::endl;

    // assume all intersections are in the map
    for (int k = 0; k < n; ++k)
    {
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                if ((*dist)[i][k] + (*dist)[k][j] < (*dist)[i][j])
                {
                    (*dist)[i][j] = (*dist)[i][k] + (*dist)[k][j];
                    (*next)[i][j] = (*next)[i][k];
                }
            }
        }
    }
    std::cout << "DIST, NEXT updated by Floyd Warshall algorithm."
              << std::endl;
}

void CarDispatcher::onTurn(
    std::vector<CarCtl> &cars_at_intersections,
    const std::vector<PassengerRequest> &passenger_requests)
{
    printf("car num: %lu, passenger num: %lu\n", cars_at_intersections.size(),
           passenger_requests.size());
    // process loaded cars to next intersection,
    // then remove from the vector
    auto car_iter = cars_at_intersections.begin();
    while (car_iter != cars_at_intersections.end())
    {
        if (car_iter->passenger_id != -1)
        {
            int dest = car_dest[car_iter->car_id];
            car_iter->intersection_id =
                (*next)[car_iter->intersection_id][dest];
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
        int min_dist = INT_MAX;
        CarCtl *nearest_car;
        for (auto car : cars_at_intersections)
        {
            int d = (*dist)[car.intersection_id][p.src_intersection_id];
            if (d < min_dist)
            {
                min_dist = d;
                nearest_car = &car;
            }
        }
        // nearest_car is NULL if no cars available
        if (!nearest_car)
        {
            break;
        }

        // pick up!
        if (nearest_car->intersection_id == p.src_intersection_id)
        {
            nearest_car->passenger_id = p.passenger_id;
            car_dest[nearest_car->car_id] = p.dst_intersection_id;
            nearest_car->intersection_id =
                (*next)[p.src_intersection_id][p.dst_intersection_id];
        }
        else // approach the passenger
        {
            int next_step =
                (*next)[nearest_car->intersection_id][p.src_intersection_id];
            nearest_car->intersection_id = next_step;
        }
    }
}