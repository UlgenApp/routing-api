from pydantic import BaseModel
from typing import List


class RouteRequest(BaseModel):
    priority: List[float]
    vehicle_count: int
    location: List[dict]
