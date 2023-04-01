from pydantic import BaseModel
from typing import List


class RouteRequest(BaseModel):
    epsilon: float
    vehicle_count: int
    depot: dict
    location: List[dict]
