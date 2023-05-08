from typing import List

from pydantic import BaseModel


class Location(BaseModel):
    priority: int
    latitude: float
    longitude: float


class Heatmap(BaseModel):
    centroids: List[Location]


class HeatmapResponse(BaseModel):
    result: Heatmap
