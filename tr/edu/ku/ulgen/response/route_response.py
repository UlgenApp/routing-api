from typing import List, Dict
from pydantic import BaseModel


class VehicleRoute(BaseModel):
    route: List[int]
    distance_travelled: int


class RouteResponse(BaseModel):
    result: Dict[str, VehicleRoute]
