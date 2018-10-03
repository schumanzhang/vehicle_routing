
"""Capacitated Vehicle Routing Problem"""
from __future__ import print_function
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

import requests
import json
import os

from random import randint

'''
{'destination_addresses': ['Town Hall Station, Park St, Stand G, Sydney NSW 2000, Australia'], 
'origin_addresses': ['Office 427, Level 4, 95 Pitt Street, Plaza Building, Australian Square, Sydney NSW 2000, Australia'], 
'rows': [{'elements': [{'distance': {'text': '1.4 km', 'value': 1425}, 
'duration': {'text': '7 mins', 'value': 392}, 'status': 'OK'}]}], 
'status': 'OK'}
'''

def loadAPIKey():
    with open('vehicle_routing/cvrp/apiKey.json') as f:
        data = json.load(f)

    return str(data['key'])


def generateGPSCoordinates():

    locations = \
        [(-33.881656, 151.205913),
        (-33.873199, 151.208848), (-33.863333, 151.206831),
        (-33.875752, 151.218998), (-33.869785, 151.193664),
        (-33.891927, 151.211595), (-33.878716, 151.199230),
        (-33.892750, 151.203915), (-33.877291, 151.190818),
        (-33.873967, 151.236424), (-33.868141, 151.211221),
        (-33.836186, 151.207338), (-33.837004, 151.224960)
        ]
    '''
    locations = \
        [(-33.881656, 151.205913),
        (-33.873199, 151.208848), (-33.863333, 151.206831),
        (-33.875752, 151.218998), (-33.869785, 151.193664),
        (-33.891927, 151.211595), (-33.878716, 151.199230),
        (-33.892750, 151.203915), (-33.877291, 151.190818),
        (-33.873967, 151.236424), (-33.868141, 151.211221),
        (-33.836186, 151.207338), (-33.837004, 151.224960),
        (-33.826554, 151.207297), (-33.857084, 151.184688),
        (-33.875486, 151.173385), (-33.881491, 151.238136),
        (-33.866031, 151.249614), (-33.831431, 151.239843)
        ]
    '''

    return locations


def generateDemands(locations):
    demands = []
    for index, location in enumerate(locations):
        if index == 0:
            demands.append(0)
        else:
            demands.append(randint(1, 8))

    return demands


class Vehicle():

    def __init__(self):
        self._capacity = 15

    @property
    def capacity(self):
        return self._capacity


class DataProblem():

    def __init__(self, locations=generateGPSCoordinates(), demands=generateDemands(generateGPSCoordinates())):
        self._vehicle = Vehicle()
        self._num_vehicles = 3
        self._locations = locations
        self._depot = 0
        self._demands = demands
        
    @property
    def vehicle(self):
        return self._vehicle

    @property
    def num_vehicles(self):
        return self._num_vehicles

    @property
    def locations(self):
        return self._locations

    @property
    def num_locations(self):
        return len(self.locations)

    @property
    def depot(self):
        return self._depot

    @property
    def demands(self):
        return self._demands


def distance_coordinates(position_1, position_2):
    request_params = 'https://maps.googleapis.com/maps/api/distancematrix/json?&origins=' \
    + ','.join(map(str, position_1)).replace(' ', '') + \
    '&destinations=' + ','.join(map(str, position_2)).replace(' ', '') + '&key=' + loadAPIKey()

    response = requests.get(request_params).json()
    print(response)
    distance = response['rows'][0]['elements'][0]['distance']['value']
    return distance


class CreateDistanceEvaluator(object):

    def __init__(self, data):
        self._distances = {}

        for from_node in xrange(data.num_locations):
            self._distances[from_node] = {}
            for to_node in xrange(data.num_locations):
                if from_node == to_node:
                    self._distances[from_node][to_node] = 0
                else:
                    self._distances[from_node][to_node] = distance_coordinates(data.locations[from_node], data.locations[to_node])

    def distance_evaluator(self, from_node, to_node):
        return self._distances[from_node][to_node]


class CreateDemandEvaluator(object):

    def __init__(self, data):
        self._demands = data.demands

    def demand_evaluator(self, from_node, to_node):
        del to_node
        return self._demands[from_node]


def add_capacity_constraints(routing, data, demand_evaluator):
    capacity = 'Capacity'
    routing.AddDimension(
        demand_evaluator,
        0,
        data.vehicle.capacity,
        True,
        capacity
    )


def parseSolution(data, routing, assignment):
    print('parsing solution')
    output = {}
    total_dist = 0
    for vehicle_id in xrange(data.num_vehicles):
        index = routing.Start(vehicle_id)
        output[vehicle_id] = {}
        output[vehicle_id]['route'] = []
        output[vehicle_id]['load'] = []
        route_dist = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = routing.IndexToNode(index)
            next_node_index = routing.IndexToNode(assignment.Value(routing.NextVar(index)))
            route_dist += distance_coordinates(data.locations[node_index], data.locations[next_node_index])
            route_load += data.demands[node_index]
            output[vehicle_id]['route'].append(node_index)
            output[vehicle_id]['load'].append(route_load)
            index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        total_dist += route_dist
        output[vehicle_id]['total_route_distance'] = route_dist
        output[vehicle_id]['total_load'] = route_load
    
    output['total_distance'] = total_dist
    return output


def solveCVRPRoutingSolution():
    data = DataProblem()
    routing = pywrapcp.RoutingModel(data.num_locations, data.num_vehicles, data.depot)
    distance_evaluator = CreateDistanceEvaluator(data).distance_evaluator

    routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator)

    demand_evaluator = CreateDemandEvaluator(data).demand_evaluator
    add_capacity_constraints(routing, data, demand_evaluator)

    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    return parseSolution(data, routing, assignment)