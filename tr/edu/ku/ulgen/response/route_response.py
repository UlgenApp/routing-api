from typing import List, Dict, Union

from pydantic import BaseModel


class VehicleRoute(BaseModel):
    route: List[int]
    distance_travelled: int


class Location(BaseModel):
    latitude: float
    longitude: float


class RouteResponse(BaseModel):
    result: Dict[str, Union[VehicleRoute, List[Location]]]
