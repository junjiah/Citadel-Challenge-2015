#include "RouteLib.h"
#include "CarDispatcher.h"
#include <iostream>
#include <climits>
#include <unordered_set>

#define DB 0

CarDispatcher::CarDispatcher(
    std::vector<IntersectionInfo> intersections,
    std::vector<RoadInfo> roads)
{
    int n = intersections.size();

    // build graph with adjency list
    graph = new Graph(n);
    for (const auto &r : roads)
    {
        int src = r.src_intersection_id,
            dst = r.dst_intersection_id;
        graph->add_edge(src, dst, r.weight);
    }
}

void CarDispatcher::onTurn(
    std::vector<CarCtl> &cars_at_intersections,
    const std::vector<PassengerRequest> &passenger_requests)
{
    static std::vector<PassengerRequest> all_requests;
    all_requests.insert(all_requests.end(),
                        passenger_requests.begin(),
                        passenger_requests.end());

    if (DB) // DEBUG print
    {
        printf("car num: %lu, passenger num: %lu\n",
               cars_at_intersections.size(),
               all_requests.size());
    }

    // no controllable cars
    if (cars_at_intersections.empty())
    {
        return;
    }

    // process loaded cars to next intersection,
    // otherwise record its position
    std::unordered_map<int, std::vector<CarCtl *> > positions;
    for (auto && car : cars_at_intersections)
    {
        if (car.passenger_id != -1)
        {
            Route &r = car_route[car.car_id];
            int next = graph->route_advance(r);
            car.intersection_id = next;
        }
        else
        {
            positions[car.intersection_id].push_back(&car);
        }
    }

    // pickup available passengers
    auto p = all_requests.begin();
    for (; p != all_requests.end();)
    {
        if (positions.find(p->src_intersection_id) != positions.end() &&
                !positions[p->src_intersection_id].empty())
        {
            // choose a car to pickup
            auto &available_cars = positions[p->src_intersection_id];
            auto pickup_car_ptr = available_cars.back();
            available_cars.pop_back();

            pickup_car_ptr->passenger_id = p->passenger_id;
            Route r = graph->find_route(p->src_intersection_id,
                                        p->dst_intersection_id);
            int next = graph->route_advance(r);
            pickup_car_ptr->intersection_id = next;
            car_route[pickup_car_ptr->car_id] = r;
            p = all_requests.erase(p);
        }
        else
        {
            ++p;
        }
    }

    // for each passenger, call nearest vehicle
    std::unordered_set<int> assigned_cars;

    for (const auto &p : all_requests)
    {
        Route best_route(INT_MAX);
        CarCtl *nearest_car = NULL;
        for (auto && car : cars_at_intersections)
        {
            // only process empty and unassigned car
            if (car.passenger_id != -1 ||
                    assigned_cars.find(car.car_id) != assigned_cars.end())
                continue;

            Route r = graph->find_route(car.intersection_id,
                                        p.src_intersection_id);
            if (r.dist < best_route.dist)
            {
                best_route = r;
                nearest_car = &car;
            }
        }
        // exit if no available cars, otherwise
        // let nearest car approach the passenger
        if (nearest_car == NULL)
            break;
        else
        {
            assigned_cars.insert(nearest_car->car_id);
            int next = graph->route_advance(best_route);
            nearest_car->intersection_id = next;
        }
    }
}