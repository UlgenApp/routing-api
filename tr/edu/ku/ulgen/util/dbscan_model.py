import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN


def dbscan_clustering(location_coordinates_dict, eps):
    location_coordinates = np.array([[d['latitude'], d['longitude']] for d in location_coordinates_dict])

    dbscan = DBSCAN(eps=eps, min_samples=1)

    dbscan.fit(location_coordinates)
    labels = dbscan.labels_

    centroids, cluster_sizes = compute_centroids(location_coordinates, labels)

    cluster_data = []

    for cluster_id, centroid in centroids.items():
        if cluster_id != -1:
            cluster_data.append({
                "priority": cluster_sizes[cluster_id],
                "latitude": centroid[0],
                "longitude": centroid[1],
            })

    return cluster_data


def compute_centroids(X, labels):
    unique_labels = set(labels)
    centroids = {}
    cluster_sizes = {}

    for cluster_id in unique_labels:
        if cluster_id != -1:
            cluster_points = X[labels == cluster_id]
            centroids[cluster_id] = cluster_points.mean(axis=0)
            cluster_sizes[cluster_id] = len(cluster_points)

    return centroids, cluster_sizes
