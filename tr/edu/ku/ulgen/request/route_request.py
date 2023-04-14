from typing import List

from pydantic import BaseModel


class RouteRequest(BaseModel):
    epsilon: float
    p_c: float
    d_c: float
    vehicle_count: int
    depot: dict
    location: List[dict]
