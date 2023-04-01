import uvicorn
from fastapi import FastAPI

from tr.edu.ku.ulgen.request.route_request import RouteRequest
from tr.edu.ku.ulgen.response.route_response import RouteResponse

app = FastAPI()


@app.get("/")
def hello_world():
    return "Hello from Ulgen Team!"


@app.post("/api/v1/route")
async def calculate_root(body: RouteRequest):
    print(body)
    return RouteResponse(result=[{"latitude": 1.2, "longitude": 2.4}, {"latitude": 2.2, "longitude": 2.6}])


if __name__ == '__main__':
    uvicorn.run("route_controller:app", reload=True)

