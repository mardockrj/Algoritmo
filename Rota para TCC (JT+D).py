"""Vehicles Routing Problem (VRP) with Time Windows."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model():
    """Stores the data for the problem."""
    data = {'time_matrix': [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 319, 314, 314, 297, 297, 276, 96, 168, 357, 308, 146, 146, 189, 152, 152],
        [0, 319, 0, 49, 49, 22, 22, 71, 70, 179, 21, 119, 249, 249, 216, 216, 211],
        [0, 314, 49, 0, 0, 32, 32, 108, 146, 200, 81, 168, 222, 222, 179, 189, 184],
        [0, 314, 49, 0, 0, 26, 26, 108, 146, 200, 81, 168, 222, 222, 179, 189, 184],
        [0, 297, 22, 26, 26, 0, 0, 65, 125, 114, 60, 119, 227, 227, 195, 195, 189],
        [0, 297, 22, 32, 32, 0, 0, 65, 125, 114, 60, 119, 227, 227, 195, 195, 189],
        [0, 276, 71, 108, 108, 65, 65, 0, 103, 114, 92, 41, 238, 238, 216, 206, 200],
        [0, 96, 70, 146, 146, 125, 125, 103, 0, 65, 179, 152, 141, 141, 135, 114, 103],
        [0, 168, 179, 200, 200, 114, 114, 114, 65, 0, 211, 141, 189, 189, 195, 168, 157],
        [0, 357, 21, 81, 81, 60, 60, 92, 179, 211, 0, 119, 287, 287, 254, 254, 249],
        [0, 308, 119, 168, 168, 119, 119, 41, 152, 141, 119, 0, 292, 292, 276, 265, 254],
        [0, 146, 249, 222, 222, 227, 227, 238, 141, 189, 287, 292, 0, 0, 20, 20, 38],
        [0, 146, 249, 222, 222, 227, 227, 238, 141, 189, 287, 292, 0, 0, 32, 32, 38],
        [0, 189, 216, 179, 179, 195, 195, 216, 135, 195, 254, 276, 32, 32, 0, 38, 44],
        [0, 152, 216, 189, 189, 195, 195, 206, 114, 168, 254, 265, 20, 20, 38, 0, 3],
        [0, 152, 211, 184, 184, 189, 189, 200, 103, 157, 249, 254, 38, 38, 44, 3, 0],

    ], 'time_windows': [
        (0, 3600),  # depot
        (0, 720),  # Local 1 - P-A3(P)
        (0, 720),  # Local 2 - P-A4(P)
        (0, 1440),  # Local 3 - P-A6(P)
        (0, 1440),  # Local 4 - P-A6(D)
        (0, 1440),  # Local 5 - P-A8(P)
        (0, 1440),  # Local 6 - P-A8(D)
        (0, 2880),  # Local 7 - P-A10(P)
        (0, 720),  # Local 8 - P-A11(D)
        (0, 1440),  # Local 9 - P-A13(P)
        (0, 720),  # Local 10 - P-A15(D)
        (0, 2880),  # Local 11 - P-A16(D)
        (0, 2160),  # Local 12 - P-A21(P)
        (0, 2160),  # Local 13 - P-A21(D)
        (0, 2160),  # Local 14 - P-A23(P)
        (0, 1440),  # Local 15 - P-A24(D)
        (0, 1440),  # Local 16 - P-A25(P)

    ], 'initial_routes': [
        [1, 2, 8, 10],  #
        [9, 5, 3, 4, 6, 7, 11],
        [14, 12, 13, 16, 15],  #

    ],
        'pickups_deliveries': [
            [1, 8],
            [2, 8],
            [3, 6],
            [5, 4],
            [9, 6],
            [7, 11],
            [2, 10],
            [12, 15],
            [14, 13],
            [16, 15],

        ], 'num_vehicles': 3, 'depot': 0}

    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time1 = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Rota da Embarcação {}:\n'.format(vehicle_id + 1)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Tempo({1},{2}) -> '.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var))
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Tempo({1},{2})\n'.format(manager.IndexToNode(index),
                                                     solution.Min(time_var),
                                                     solution.Max(time_var))
        plan_output += 'Tempo da rota: {} min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time1 += solution.Min(time_var)
    print('Total de minutos para todas as embarcações: {} min'.format(total_time1))


def main():
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.

    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        120,  # allow waiting time
        5000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))

    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <=
            time_dimension.CumulVar(delivery_index))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
    # search_parameters.time_limit.seconds = 30
    # search_parameters.log_search = True

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    initial_solution = routing.ReadAssignmentFromRoutes(data['initial_routes'], True)
    print('\n =================== Solução inicial por sugestão ================= \n')
    print_solution(data, manager, routing, initial_solution)

    if solution:
        print('\n ====================== Solução Proposta ======================= \n')
        print_solution(data, manager, routing, solution)


if __name__ == '__main__':
    main()