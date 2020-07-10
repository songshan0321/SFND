#include <chrono>
#include <string>
#include "kdtree.h"

void proximity(KdTree* tree, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, int index, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearbyPoints = tree->search(points[index], distanceTol);
	for (int id : nearbyPoints)
    {
    	if (!processed[id])
        	proximity(tree, points, cluster, processed, id, distanceTol);
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	for (int i = 0; i < points.size(); i++)
    {
    	if (!processed[i])
        {
        	std::vector<int> cluster;
        	proximity(tree, points, cluster, processed, i, distanceTol);
        	clusters.push_back(cluster);
        }
    }
	return clusters;
}