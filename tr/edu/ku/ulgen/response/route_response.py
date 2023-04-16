from typing import List, Dict

from pydantic import BaseModel


class VehicleRoute(BaseModel):
    route: List[int]
    distance_travelled: int


class Location(BaseModel):
    priority: int
    latitude: float
    longitude: float


class Route(BaseModel):
    centroids: List[Location]
    route: Dict[str, VehicleRoute]


class RouteResponse(BaseModel):
    result: Route
