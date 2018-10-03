from django.conf.urls import url
from django.contrib import admin

from vehicle_routing.cvrp import endpoints

urlpatterns = [
    url(r'^admin/', admin.site.urls),
    url(r'^routing/api/generate', endpoints.GenerateData.as_view()),
    url(r'^routing/api/cvrp', endpoints.GenerateCVRPSolution.as_view())
]
