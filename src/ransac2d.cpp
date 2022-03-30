/* \author Camilo Duarte */
// Submission to Ransac plane segmentation algorithm

#include "render.h"
#include <unordered_set>

#include <iostream>
#include <vector>
#include <stdlib.h> /* math functions */
#include <assert.h>

#include <experimental/filesystem>
using std::experimental::filesystem::current_path;

std::vector<float> crossProduct(std::vector<float> v1, std::vector<float> v2){
	// Cross product only valid for 3d vectors
	assert (v1.size() == 3 & v2.size() == 3);

	float x1, y1, z1, x2, y2, z2;
	x1 = v1[0];
	y1 = v1[1];
	z1 = v1[2];
	x2 = v2[0];
	y2 = v2[1];
	z2 = v2[2];
	std::vector<float> cross_out = {y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2};
	return cross_out;
}

std::vector<float> normalize(std::vector<float> vec){
	float norm;
	norm = sqrt(pow(vec[0], 2.) + pow(vec[1], 2.) + pow(vec[2], 2.));
	std::vector<float> norm_vec = {vec[0]/norm, vec[1]/norm, vec[2]/norm};
	return norm_vec;
}

std::vector<int> GetIndexes(int num_points, size_t cloud_size){
	std::vector<int> rand_vec;
	int rand_ind;
	bool repeated_sample = true;

	// Build a vector of indexes to get points from
	for (int i = 0; i<num_points; i++){
		repeated_sample = true;
		if (i==0){
			rand_vec.push_back((rand() % cloud_size + 1));
		}else{
			while (repeated_sample ){
				rand_ind = (rand() % cloud_size + 1);
				if (!(std::count(rand_vec.begin(), rand_vec.end(), rand_ind))){
					repeated_sample = false;
					rand_vec.push_back(rand_ind);
				}
			}
			
		}
	}

	return rand_vec;
}

std::vector<int> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> inliersResult;

	// Set a random seed and number of points to use
	srand(time(NULL));
	int num_points = 3;


	// TODO: Fill in this function
	// Points
	pcl::PointXYZI P, Q, R;

	// Plane params
	float A, B, C, D;

	// For max iterations 
	std::vector<int> rand_vec;
	std::vector<float> v3;
	int best_count = 0;
	for (int i = 0; i < maxIterations; i++){
		std::vector<int> inliers;
		// Randomly sample subset and fit line << std::endl
		rand_vec = GetIndexes(num_points, cloud->size());
		// print_vec(rand_vec);

		// This code only works for two points
		P = cloud->points[rand_vec[0]];
		Q = cloud->points[rand_vec[1]];
		R = cloud->points[rand_vec[2]];

		// Create the plane vectors
		std::vector<float> v1 = {Q.x-P.x, Q.y-P.y, Q.z-P.z};  
		std::vector<float> v2 = {R.x-P.x, R.y-P.y, R.z-P.z}; 
		v3 = crossProduct(normalize(v1), normalize(v2));

		// Measure distance between every point and fitted line
		A = v3[0];
		B = v3[1];
		C = v3[2];
		D = -(A*P.x + B*P.y + C*P.z);
		
		int count = 0;
		float d, x, y, z;
		// Check every point
		for (int j = 0; j < cloud->size(); j++){
			x = cloud->points[j].x;
			y = cloud->points[j].y;
			z = cloud->points[j].z;
			d = abs(A*x + B*y + C*z + D) / sqrt(pow(A, 2.) + pow(B, 2.) + pow(C, 2.));
			if (d < distanceTol){
				// If distance is smaller than threshold count it as inlier and save index
				count += 1;
				inliers.push_back(j);
			}
		}
		// If best, save indexes
		if (count > best_count){
			best_count = count;
			inliersResult = inliers;
		}
	}


	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}