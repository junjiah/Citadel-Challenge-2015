#include "RouteLib.h"
#include "CarDispatcher.h"
#include <iostream>
#include <climits>
#include <unordered_set>

#define DB 0

struct Assignment
{
    CarCtl *car;
    int passenger_id;
    Route route;
    Assignment(CarCtl *arg_car, int arg_passenger_id, Route arg_route) :
    car(arg_car), passenger_id(arg_passenger_id), route(arg_route) { }
};

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
    else
    {
        if (DB)
            std::cin.get();
    }

    // process loaded cars to next intersection,
    // otherwise record its position
    std::unordered_map<int, std::vector<CarCtl *> > positions;
    std::vector<CarCtl *> unassigned_cars;
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

    // record remaining unassigned cars
    for (auto kv : positions)
        for (auto car : kv.second)
            unassigned_cars.push_back(car);

    // sort assignment (car and 
    // passenger request by distances
    std::vector<Assignment> assignments;
    std::unordered_set<int> assigned_car_ids, assigned_passenger_ids;
    for (auto car : unassigned_cars)
    {
        for (const auto &p : all_requests)
        {
            auto a = Assignment(car,
                                p.passenger_id,
                                graph->find_route(car->intersection_id,
                                                  p.src_intersection_id));
            assignments.push_back(a);
        }
    }
    std::sort(assignments.begin(),
              assignments.end(),
              [](Assignment a, Assignment b)
    {
        return a.route.dist < b.route.dist;
    });

    // process assignments, assign cars to nearest passenger
    for (auto&& a : assignments)
    {
        // only process unassigned car and passenger
        if (assigned_car_ids.find(a.car->car_id) ==
                assigned_car_ids.end() &&
                assigned_passenger_ids.find(a.passenger_id) ==
                assigned_passenger_ids.end())
        {
            a.car->intersection_id = graph->route_advance(a.route);
            assigned_car_ids.insert(a.car->car_id);
            assigned_passenger_ids.insert(a.passenger_id);
        }
    }
}