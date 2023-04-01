import uvicorn
from fastapi import FastAPI

from tr.edu.ku.ulgen.request.route_request import RouteRequest
from tr.edu.ku.ulgen.response.route_response import RouteResponse
from tr.edu.ku.ulgen.util.dbscan_model import dbscan_clustering
from tr.edu.ku.ulgen.util.routing_algorithm import calculate_routing_result

app = FastAPI()


@app.get("/")
def hello_world():
    return "Hello from Ulgen Team!"


@app.post("/api/v1/route")
async def calculate_root(body: RouteRequest):
    centroid_data = dbscan_clustering(body.location, body.epsilon)
    routing_response = {"result": calculate_routing_result(centroid_data, body.vehicle_count)}
    return RouteResponse(**routing_response)


if __name__ == '__main__':
    uvicorn.run("route_controller:app", reload=True)

