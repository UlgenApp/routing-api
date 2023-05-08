import uvicorn
from fastapi import FastAPI, HTTPException

from tr.edu.ku.ulgen.request.heatmap_request import HeatmapRequest
from tr.edu.ku.ulgen.request.route_request import RouteRequest
from tr.edu.ku.ulgen.response.heatmap_response import HeatmapResponse
from tr.edu.ku.ulgen.response.route_response import RouteResponse
from tr.edu.ku.ulgen.util.dbscan_model import dbscan_clustering
from tr.edu.ku.ulgen.util.routing_algorithm import calculate_routing_result

app = FastAPI()


@app.post("/api/v1/route")
async def calculate_route(body: RouteRequest):
    """
    Calculates the optimal route given the input parameters using the FastAPI framework.

    :param body: A RouteRequest object that contains information about the location of the points to be routed,
                 as well as other parameters such as the epsilon value and the number of vehicles.
    :type body: tr.edu.ku.ulgen.request.route_request.RouteRequest

    :return: A RouteResponse object that contains the calculated route as well as the centroid data.
    :rtype: tr.edu.ku.ulgen.response.route_response.RouteResponse
    """

    centroid_data = dbscan_clustering(body.location, body.epsilon)
    route = calculate_routing_result(body.priority_coefficient, body.distance_coefficient, centroid_data,
                                     body.vehicle_count, body.depot)

    if route == -2:
        raise HTTPException(status_code=400, detail="Bad request: Invalid input.")

    routing_response = {"result": {"centroids": centroid_data,
                                   "route": route}}

    return RouteResponse(**routing_response)


@app.post("/api/v1/heatmap")
async def calculate_heatmap(body: HeatmapRequest):
    """
    Calculates the information to draw a Heatmap of a location.

    :param body: A HeatmapRequest object that contains information about the location of the points to be shown,
                 as well as parameter such as the epsilon value.
    :type body: tr.edu.ku.ulgen.request.heatmap_request.HeatmapRequest

    :return: A HeatmapResponse object that contains the centroid data.
    :rtype: tr.edu.ku.ulgen.response.heatmap_response.HeatmapResponse
    """

    centroid_data = dbscan_clustering(body.location, body.epsilon)

    heatmap_response = {"result": {"centroids": centroid_data}}

    return HeatmapResponse(**heatmap_response)


if __name__ == '__main__':
    """
    Runs the application using the Uvicorn web server.
    """

    uvicorn.run("route_controller:app", reload=True)
