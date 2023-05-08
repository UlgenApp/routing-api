from typing import List

from pydantic import BaseModel


class HeatmapRequest(BaseModel):
    epsilon: float
    location: List[dict]
