from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status

from vehicle_routing.cvrp.algorithm import generateGPSCoordinates
from vehicle_routing.cvrp.algorithm import solveCVRPRoutingSolution


class GenerateData(APIView):

    def get(self, request):
        locations = generateGPSCoordinates()
        return Response({'coordinates': locations})


class GenerateCVRPSolution(APIView):

    def get(self, request):
        return Response(solveCVRPRoutingSolution())