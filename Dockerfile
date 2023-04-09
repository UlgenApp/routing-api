FROM python:3.11

WORKDIR /app

EXPOSE 8000

COPY ./requirements.txt /app/requirements.txt

RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt

COPY ./tr/edu/ku/ulgen /app/tr/edu/ku/ulgen

CMD ["uvicorn", "tr.edu.ku.ulgen.controller.route_controller:app", "--host", "0.0.0.0", "--port", "8000"]

