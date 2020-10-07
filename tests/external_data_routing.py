"""Example program for solving VRP that reads in data."""

from __future__ import print_function # TODO: understand where these are used
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model(): # We can replaice this with a VRPModeling structure?, or a read-in
    """Stores the data for the problem. Creates a dictionary, to be exact. """

    data = {}
    data['distance_matrix'] = []
    data['num_vehicles'] = 1
    data['depot'] = 0 # TODO: Find out what this means
    return data

def print_solution(data, manager, routing, solution):
    # We can also modify this
        """Prints solution on console."""
        max_route_distance = 0
        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += ' {} -> '.format(manager.IndexToNode(index))
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(
                    previous_index, index, vehicle_id)
            plan_output += '{}\n'.format(manager.IndexToNode(index))
            plan_output += 'Distance of the route: {}m\n'.format(route_distance)
            print(plan_output)
            max_route_distance = max(route_distance, max_route_distance)
        print('Maximum of the route distances: {}m'.format(max_route_distance))

def main():
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model()

    dim = len(data['distance_matrix'])
    num_vehicles = data['num_vehicles']
    depot = data['depot']
    # create a routing index, # TODO: Find out what this does
    manager = pywrapcp.RoutingIndexManager(dim, num_vehicles, depot)

    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index): # TODO: Is this really necessary?
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        3000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100) # TODO: This thing is BAD...the number has to be arbitrarily tuned. get rid of if possible.

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(searchsearch_parameters)

    # Print solution on console
    if solution:
        print_solution(data, manager, routing, solution)

if __name__ == '__main__':
    main()
