
import math
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import json

supply_need = [0, 100, 200, 50, 150, 1000]
p_coefficient = 0.3
d_coefficient = 0.7


def create_data():
    """Creates the data."""
    data = {'API_key': 'YOUR_API_KEY',
            # latitude, longitude
            'addresses': ['41.015137,28.979530',  # depot, Istanbul
                          '39.925533,32.866287',  # Ankara
                          '38.423733,27.142826',  # Izmir
                          '37.000000,35.321335',  # Adana
                          '37.575275,36.922821',  # Kahramanmaraş
                          '37.066666,37.383331',  # Gaziantep
                          ]}
    return data


def create_distance_matrix(data):
    addresses = data["addresses"]
    API_key = data["API_key"]
    # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
    max_elements = 100
    num_addresses = len(addresses)  # 16 in this example.
    # Maximum number of rows that can be computed per request.
    max_rows = max_elements // num_addresses
    # num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
    q, r = divmod(num_addresses, max_rows)
    dest_addresses = addresses
    distance_matrix = []
    # Send q requests, returning max_rows rows per request.
    for i in range(q):
        origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)

    # Get the remaining r rows, if necessary.
    if r > 0:
        origin_addresses = addresses[q * max_rows: q * max_rows + r]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)
    return distance_matrix


def send_request(origin_addresses, dest_addresses, API_key):
    """ Build and send request for the given origin and destination addresses."""

    def build_address_str(addresses):
        # Build a pipe-separated string of addresses
        address_str = ''
        for i in range(len(addresses) - 1):
            address_str += addresses[i] + '|'
        address_str += addresses[-1]
        return address_str

    request = 'https://maps.googleapis.com/maps/api/distancematrix/json?units=imperial'
    origin_address_str = build_address_str(origin_addresses)
    dest_address_str = build_address_str(dest_addresses)
    request = request + '&origins=' + origin_address_str + '&destinations=' + \
              dest_address_str + '&key=' + API_key

    jsonResult = requests.request("GET", request)
    response = json.loads(jsonResult.text)
    return response


def build_distance_matrix(response):
    distance_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
        distance_matrix.append(row_list)
    return distance_matrix


def construct_priority_vector():
    """Constructs priority vector using the supply need"""
    total_supply_need = 0
    priority_vector = []

    for i in supply_need:
        total_supply_need += i

    priority_vector.append(supply_need[0])

    for s in supply_need[1:]:
        priority_vector.append(math.ceil(total_supply_need * p_coefficient / s))

    return np.array(priority_vector)


def construct_prioritized_distance_matrix(distance_matrix, priority_vector):
    """Constructs the prioritized data matrix using priority vector and distance matrix"""
    current_row = 0
    prioritized_distance_matrix = []
    for row in distance_matrix:
        i = 0
        prioritized_row = []
        for elem in row:
            """Calculating the prioritized distance by adding initial distance, current location's priority and 
            target location's priority """
            if i == current_row:
                prioritized_row.append(0)
            else:
                prioritized_row.append(math.ceil(elem + (priority_vector[current_row] + priority_vector[i]) / 2))
            i += 1
        current_row += 1
        prioritized_distance_matrix.append(prioritized_row)

    return np.array(prioritized_distance_matrix)


def create_data_model():
    """Stores the data for the problem."""
    data = create_data()
    distance_matrix = np.array(create_distance_matrix(data))
    priority_vector = construct_priority_vector()
    prioritized_distance_matrix = construct_prioritized_distance_matrix(distance_matrix * d_coefficient,
                                                                        priority_vector)
    data = {'distance_matrix': prioritized_distance_matrix, 'num_vehicles': 4, 'depot': 0}
    return data


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
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
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
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
        60000000,  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print('No solution found !')


if __name__ == '__main__':
    main()
