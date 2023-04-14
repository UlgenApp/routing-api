from typing import List

from pydantic import BaseModel


class RouteRequest(BaseModel):
    epsilon: float
    priority_coefficient: float
    distance_coefficient: float
    vehicle_count: int
    depot: dict
    location: List[dict]
