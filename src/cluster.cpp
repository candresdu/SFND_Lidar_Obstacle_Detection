/* \author Camilo Duarte */
// Submission to clustering using kd tree search algorithm
#include "render.h"
#include "box.h"
#include <chrono>
#include <string>
#include "kdtree.h"


void proximity(const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool> &proc_points, int id, KdTree* tree, float distanceTol){
	// Mark the point as processed
	proc_points[id] = true;
	// Add point to cluster
	cluster.push_back(id);
	// Find nearby points
	std::vector<int> nearby_points = tree->search(points[id], distanceTol);


	for (int nid : nearby_points){
		if (!proc_points[nid]){
			proximity(points, cluster, proc_points, nid, tree, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
	// Create a list of indices of points close to each other (or clusters)
	std::vector<std::vector<int>> clusters;

	// Create vector of processed booleans
	std::vector<bool> processed(points.size(), false);

	for (int id =0; id < points.size(); id++){
		// 	Check if the point has been processed
		if (!processed[id]){
			// Search the tree
			std::vector<int> cluster;
			proximity(points, cluster, processed, id, tree, distanceTol);
			if((cluster.size() >= minSize) && (cluster.size() <= maxSize)){
				clusters.push_back(cluster);
			}
		}
	}

 
	return clusters;

}
