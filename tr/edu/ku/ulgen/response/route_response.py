from pydantic import BaseModel
from typing import List


class RouteResponse(BaseModel):
    result: List[dict[str, float]]
