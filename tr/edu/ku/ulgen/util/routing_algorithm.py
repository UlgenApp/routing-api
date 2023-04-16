import json
import math
import os

import numpy as np
import requests
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# Global declarations of the required configurations.
p_coefficient = 0.3
d_coefficient = 0.7
API_key = os.environ['GMAPS_API_KEY']


def create_data(data_points):
    """
    Create data for distance matrix.

    :param data_points: A list of strings containing the latitude and longitude of each location.
    :type data_points: list

    :return: A dictionary containing the API key and addresses.
    :rtype: dict
    """

    data = {'API_key': API_key,
            'addresses': data_points}
    return data


def create_distance_matrix(data):
    """
    Create a distance matrix using the Google Maps API.

    :param data: A dictionary containing the API key and addresses.
    :type data: dict

    :return: A list of lists representing the distance matrix, or -2 if an error occurs.
    :rtype: list or int
    """

    addresses = data["addresses"]
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
    try:
        for i in range(q):
            origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
            response = send_request(origin_addresses, dest_addresses)
            distance_matrix += build_distance_matrix(response)

        # Get the remaining r rows, if necessary.
        if r > 0:
            origin_addresses = addresses[q * max_rows: q * max_rows + r]
            response = send_request(origin_addresses, dest_addresses)
            distance_matrix += build_distance_matrix(response)
    except TypeError:
        return -2

    return distance_matrix


def send_request(origin_addresses, dest_addresses):
    """
    Send a request to the Google Maps Distance Matrix API and return the response.

    :param origin_addresses: A list of origin addresses.
    :type origin_addresses: list
    :param dest_addresses: A list of destination addresses.
    :type dest_addresses: list

    :return: A dictionary representing the JSON response from the API.
    :rtype: dict
    """

    def build_address_str(addresses):
        """
        Build a pipe-separated string of addresses.

        :param addresses: A list of addresses.
        :type addresses: list

        :return: A pipe-separated string of addresses.
        :rtype: str
        """

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
    """
    Build a distance matrix from the API response.

    :param response: A dictionary representing the JSON response from the API.
    :type response: dict

    :return: A list of lists representing the distance matrix, or -2 if an error occurs.
    :rtype: list
    """

    distance_matrix = []
    for row in response['rows']:
        try:
            row_list = [row['elements'][j]['distance']['value'] for j in range(len(row['elements']))]
            distance_matrix.append(row_list)
        except KeyError:
            return -2

    return distance_matrix


def construct_priority_vector(priority):
    """
    Construct a priority vector.

    :param priority: A list of integers representing the priority of each location.
    :type priority: list

    :return: A NumPy array representing the priority vector.
    :rtype: numpy.ndarray
    """

    total_supply_need = 0
    priority_vector = []

    for i in priority:
        total_supply_need += i

    priority_vector.append(priority[0])

    for s in priority[1:]:
        priority_vector.append(math.ceil(total_supply_need * p_coefficient / s))

    return np.array(priority_vector)


def construct_prioritized_distance_matrix(distance_matrix, priority_vector):
    """
    Construct a prioritized distance matrix.

    :param distance_matrix: A NumPy array representing the distance matrix.
    :type distance_matrix: numpy.ndarray
    :param priority_vector: A NumPy array representing the priority vector.
    :type priority_vector: numpy.ndarray

    :return: A NumPy array representing the prioritized distance matrix.
    :rtype: numpy.ndarray
    """

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


def create_data_model(priority, vehicle_count, data_points):
    """
    Create a data model for the vehicle routing problem.

    :param priority: A list of integers representing the priority of each location.
    :type priority: list
    :param vehicle_count: The number of vehicles.
    :type vehicle_count: int
    :param data_points: A list of strings containing the latitude and longitude of each location.
    :type data_points: list

    :return: A dictionary representing the data model, or -2 if an error occurs.
    :rtype: dict
    """

    data = create_data(data_points)
    distance_matrix = create_distance_matrix(data)
    if distance_matrix == -2:
        return distance_matrix
    else:
        distance_matrix = np.array(distance_matrix)

    priority_vector = construct_priority_vector(priority)
    prioritized_distance_matrix = construct_prioritized_distance_matrix(distance_matrix * d_coefficient,
                                                                        priority_vector)
    location_count = len(priority_vector)
    demands = [1] * location_count
    demands[0] = 0

    tmp_sum = location_count - 1
    tmp_vehicle_count = vehicle_count
    vehicle_capacities = []

    while tmp_vehicle_count > 0:
        tmp_capacity = math.floor(tmp_sum / tmp_vehicle_count)
        vehicle_capacities.append(tmp_capacity)
        tmp_sum -= tmp_capacity
        tmp_vehicle_count -= 1

    data = {'distance_matrix': prioritized_distance_matrix, 'demands': demands,
            'vehicle_capacities': vehicle_capacities, 'num_vehicles': vehicle_count, 'depot': 0}
    return data


def find_solution(data, manager, routing, solution):
    """
    Find the solution for the vehicle routing problem.

    :param data: A dictionary representing the data model.
    :type data: dict
    :param manager: The routing index manager.
    :type manager: pywrapcp.RoutingIndexManager
    :param routing: The routing model.
    :type routing: pywrapcp.RoutingModel
    :param solution: The solution of the routing problem.
    :type solution: pywrapcp.Assignment

    :return: A dictionary representing the solution, including routes and distances traveled.
    :rtype: dict
    """

    solution_dict = {}
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        route_distance = 0
        current_vehicle_route = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            current_vehicle_route.append(node)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        current_vehicle_route.append(manager.IndexToNode(index))

        solution_dict["vehicle_" + str(vehicle_id + 1)] = {"route": current_vehicle_route,
                                                           "distance_travelled": route_distance}

    return solution_dict


def run_algorithm(priority, vehicle_count, data_points):
    """
    Run the vehicle routing algorithm.

    :param priority: A list of integers representing the priority of each location.
    :type priority: list
    :param vehicle_count: The number of vehicles.
    :type vehicle_count: int
    :param data_points: A list of strings containing the latitude and longitude of each location.
    :type data_points: list

    :return: A dictionary representing the solution, including routes and distances traveled, or -1 if no solution is found,
    or -2 if an error occurs.
    :rtype: dict or int
    """

    # Instantiate the data problem.
    data = create_data_model(priority=priority, vehicle_count=vehicle_count, data_points=data_points)
    if data == -2:
        return data
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """
        Returns the distance between two nodes.

        :param from_index: The index of the origin node.
        :param to_index: The index of the destination node.
        :return: The distance between the two nodes.
        """

        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

        # Add Capacity constraint.

    def demand_callback(from_index):
        """
        Returns the demand of a node.

        :param from_index: The index of the node.
        :return: The demand of the node.
        """

        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    search_parameters.time_limit.FromSeconds(1)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Returns solution.
    if solution:
        return find_solution(data, manager, routing, solution)
    else:
        return -1


def read_centroid_data(centroid_data):
    """
    Read centroid data and extract priorities and data points.

    :param centroid_data: A list of dictionaries containing latitude, longitude, and priority of each location.
    :return: A tuple containing two lists: one with priorities and the other with data points (latitude and longitude).
    """

    priority_list = []
    data_points = []

    for elem in centroid_data:
        priority_list.append(elem["priority"])
        data_points.append(str(elem["latitude"]) + "," + str(elem["longitude"]))

    return priority_list, data_points


def calculate_routing_result(priority_coefficient, distance_coefficient, centroid_data, vehicle_count, depot):
    """
    Calculate the routing result using the vehicle routing algorithm.
    :param priority_coefficient: The weight given to priority in the objective function.
    :type priority_coefficient: float
    :param distance_coefficient: The weight given to distance in the objective function.
    :type distance_coefficient: float
    :param centroid_data: A list of dictionaries containing latitude, longitude, and priority of each location.
    :type centroid_data: list
    :param vehicle_count: The number of vehicles.
    :type vehicle_count: int
    :param depot: A dictionary containing the latitude and longitude of the depot.
    :type depot: dict

    :return: A dictionary representing the solution, including routes and distances traveled, or -1 if no solution is found.
    :rtype: dict or int
    """
    global p_coefficient, d_coefficient
    p_coefficient = priority_coefficient
    d_coefficient = distance_coefficient

    if centroid_data is None or vehicle_count is None:
        return -1
    else:
        priority, data_points = read_centroid_data(centroid_data)
        data_points_with_depot = [f"{depot['latitude']},{depot['longitude']}"]
        data_points_with_depot += data_points
        priority_with_depot = [0]
        priority_with_depot += priority
        return run_algorithm(priority=priority_with_depot, vehicle_count=vehicle_count,
                             data_points=data_points_with_depot)
