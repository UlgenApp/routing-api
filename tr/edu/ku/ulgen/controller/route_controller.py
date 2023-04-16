import uvicorn
from fastapi import FastAPI, HTTPException

from tr.edu.ku.ulgen.request.route_request import RouteRequest
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


if __name__ == '__main__':
    """
    Runs the application using the Uvicorn web server.
    """

    uvicorn.run("route_controller:app", reload=True)
