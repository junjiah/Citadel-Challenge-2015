#ifndef CAR_DISPATCHER_H_
#define CAR_DISPATCHER_H_
#include <map>
#include <set>
#include <vector>
#include <stdio.h>
#include <unordered_map>
#include "TransportationTypes.h"
#include "SrcDestMap.h"
#include "Graph.h"

class CarDispatcher
{

private:
    SrcDestMap *dist;
    SrcDestMap *next;
    std::unordered_map<int, int> car_dest;

    Graph *graph;

public:

    CarDispatcher(
        std::vector<IntersectionInfo> intersections,
        std::vector<RoadInfo> roads);

    void onTurn(
        std::vector<CarCtl> &cars_at_intersections,
        const std::vector<PassengerRequest> &passenger_requests);

    ~CarDispatcher { delete graph; }
};



#endif
