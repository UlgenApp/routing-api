import numpy as np
from sklearn.cluster import DBSCAN


def dbscan_clustering(location_coordinates_dict, eps):
    """
    Perform DBSCAN clustering on location coordinates and return cluster centroids and sizes.

    :param location_coordinates_dict: A list of dictionaries containing latitude and longitude of locations.
    :type location_coordinates_dict: list
    :param eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
    :type eps: float

    :return: A list of dictionaries containing priority, latitude, and longitude for each cluster.
    :rtype: list
    """

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
    """
    Compute the centroids and sizes of the clusters.

    :param X: A 2D array containing the latitude and longitude of the locations.
    :type X: numpy.array
    :param labels: A 1D array containing the cluster labels for each location.
    :type labels: numpy.array

    :return: A tuple containing two dictionaries: one containing the cluster centroids, and one containing the sizes of each cluster.
    :rtype: tuple
    """

    unique_labels = set(labels)
    centroids = {}
    cluster_sizes = {}

    for cluster_id in unique_labels:
        if cluster_id != -1:
            cluster_points = X[labels == cluster_id]
            centroids[cluster_id] = cluster_points.mean(axis=0)
            cluster_sizes[cluster_id] = len(cluster_points)

    return centroids, cluster_sizes
