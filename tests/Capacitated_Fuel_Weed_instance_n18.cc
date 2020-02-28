#include <vector>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
struct DataModel {
  const std::vector<std::vector<int64>> distance_matrix{
      {0, 136213, 125710, 55354, 374056, 345416, 354203, 375772, 410710, 502547, 483406, 617548, 655624, 617132, 611777, 602279, 570711, 581569},
      {136213, 0, 59141, 201066, 534411, 505772, 514558, 536127, 571066, 662904, 614472, 696751, 734828, 696335, 690980, 681482, 649914, 660772},
      {125710, 59141, 0, 145000, 472132, 443492, 452279, 473847, 515000, 600624, 552193, 618970, 641543, 603051, 597696, 588198, 556630, 567487},
      {55354, 201066, 145000, 0, 321274, 292635, 301421, 322990, 360000, 449767, 415980, 550122, 588198, 549706, 544350, 534853, 503284, 514142},
      {374056, 534411, 472132, 321274, 0, 30000, 75355, 22071, 189350, 111066, 124142, 258284, 321213, 371777, 389203, 406630, 410268, 429410},
      {345416, 505772, 443492, 292635, 30000, 0, 5000, 5000, 149350, 165208, 178284, 320711, 369142, 419706, 437132, 454558, 458197, 477339},
      {354203, 514558, 452279, 301421, 75355, 5000, 0, 7070, 125208, 186924, 206213, 342426, 404142, 454706, 472132, 489558, 493197, 512339},
      {375772, 536127, 473847, 322990, 22071, 5000, 7070, 0, 138137, 146924, 160000, 302426, 371569, 422132, 439558, 456985, 460624, 479767},
      {410710, 571066, 515000, 360000, 189350, 149350, 125208, 138137, 0, 254142, 318995, 446924, 536777, 587340, 604767, 622193, 625833, 644975},
      {502547, 662904, 600624, 449767, 111066, 165208, 186924, 146924, 254142, 0, 52426, 170711, 260563, 311127, 338198, 370269, 406127, 422339},
      {483406, 614472, 552193, 415980, 124142, 178284, 206213, 160000, 318995, 52426, 0, 105000, 186569, 237132, 254558, 280772, 322487, 338701},
      {617548, 696751, 618970, 550122, 258284, 320711, 342426, 302426, 446924, 170711, 105000, 0, 75711, 184853, 213994, 246066, 287782, 303995},
      {655624, 734828, 641543, 588198, 321213, 369142, 404142, 371569, 536777, 260563, 186569, 75711, 0, 90000, 119142, 157426, 199142, 215355},
      {617132, 696335, 603051, 549706, 371777, 419706, 454706, 422132, 587340, 311127, 237132, 184853, 90000, 0, 15000, 51213, 95000, 109142},
      {611777, 690980, 597696, 544350, 389203, 437132, 472132, 439558, 604767, 338198, 254558, 213994, 119142, 15000, 0, 10000, 62071, 74142},
      {602279, 681482, 588198, 534853, 406630, 454558, 489558, 456985, 622193, 370269, 280772, 246066, 157426, 51213, 10000, 0, 37071, 44142},
      {570711, 649914, 556630, 503284, 410268, 458197, 493197, 460624, 625833, 406127, 322487, 287782, 199142, 95000, 62071, 37071, 0, 7070},
      {581569, 660772, 567487, 514142, 429410, 477339, 512339, 479767, 644975, 422339, 338701, 303995, 215355, 109142, 74142, 44142, 7070, 0},
  };
  const std::vector<int64> demands{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  const std::vector<int64> vehicle_capacities{5, 5, 5, 5, 5, 5, 5, 5, 5, 5};
  //const std::vector<int64> vehicle_distance_capacities{1000000, 1000000, 1000000, 1000000};
  const int num_vehicles = 10;
  const RoutingIndexManager::NodeIndex depot{0};
};

//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
 int64 total_distance{0};
 int64 total_load{0};
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64 index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64 route_distance{0};
    int64 route_load{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false) {
      int64 node_index = manager.IndexToNode(index).value();
      route_load += data.demands[node_index];
      route << node_index << " Load(" << route_load << ") -> ";
      int64 previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    route_distance = route_distance/100;
    LOG(INFO) << "Distance of the route: " << route_distance << "m";
    LOG(INFO) << "Load of the route: " << route_load;
    total_distance += route_distance;
    total_load += route_load;
  }
  LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  LOG(INFO) << "Total load of all routes: " << total_load;
  LOG(INFO) << "";
  LOG(INFO) << "Advanced usage:";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void VrpCapacityFuel() {
  // Instantiate the data problem.
  DataModel data;

  // Create Routing Index Manager
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  // Create Routing Model.
  RoutingModel routing(manager);

  // Create and register a transit callback.
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Add Capacity constraint.
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [&data, &manager](int64 from_index) -> int64 {
          // Convert from routing variable Index to demand NodeIndex.
          int from_node = manager.IndexToNode(from_index).value();
          return data.demands[from_node];
        });
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,    // transit callback index
        int64{0},                 // null capacity slack
        data.vehicle_capacities,  // vehicle maximum capacities
        true,                     // start cumul to zero
        "Capacity");
    routing.AddDimension(
        transit_callback_index,    // transit callback index
        int64{0},                 // null capacity slack
        1300000,//data.vehicle_distance_capacities,  // vehicle maximum distance capacities
        true,                     // start cumul to zero
        "Distance");

  // Setting first solution heuristic.
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::SAVINGS);
  //searchParameters.set_local_search_metaheuristic(LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
  searchParameters.set_solution_limit(1);
  //searchParameters.mutable_time_limit()->set_seconds(30);


  // Solve the problem.
  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  // Print solution on console.
  PrintSolution(data, manager, routing, *solution);
}
}  // namespace operations_research

int main(int argc, char** argv) {
  operations_research::VrpCapacityFuel();
  return EXIT_SUCCESS;
}
