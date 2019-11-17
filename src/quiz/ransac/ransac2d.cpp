/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <algorithm>
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <boost/program_options.hpp>
#include <boost/qvm/all.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::vector<pcl::PointXYZ> Sample(int num,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    std::vector<int> vec;
    for (int i = 0; i < cloud->size(); ++i) {
        vec.push_back(i);
    }
    std::random_shuffle(vec.begin(), vec.end());

    std::vector<pcl::PointXYZ> sampled;
    for (int i = 0; i < num; ++i) {
        sampled.push_back(cloud->at(vec.at(i)));
    }
    return sampled;
}

std::unordered_set<int> Ransac(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    int maxIterations,
    float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations
    while (maxIterations--) {
        // Randomly sample subset and fit line
        auto samples = Sample(3, cloud);

        // fit a plane
        boost::qvm::vec<float, 3> v1 = {
            samples.at(1).x - samples.at(0).x,
            samples.at(1).y - samples.at(0).y,
            samples.at(1).z - samples.at(0).z
        };
        boost::qvm::vec<float, 3> v2 = {
            samples.at(2).x - samples.at(0).x,
            samples.at(2).y - samples.at(0).y,
            samples.at(2).z - samples.at(0).z
        };

        auto cross = boost::qvm::cross(v1, v2);

        const float a = cross.a[0];
        const float b = cross.a[1];
        const float c = cross.a[2];
        const float d = -(a * samples.at(0).x + b * samples.at(0).y + c * samples.at(0).z);
        const float denom = std::sqrt(a * a + b * b + c * c);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        std::unordered_set<int> inliers;
        for (int i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->at(i);
            float d = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / denom;
            if (d <= distanceTol) {
                inliers.insert(i);
            }
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main (int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("max-iteration", po::value<int>()->default_value(100), "iteration")
        ("distance-threshold", po::value<float>()->default_value(0.2), "distance threshold")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(
        cloud,
        vm["max-iteration"].as<int>(),
        vm["distance-threshold"].as<float>());

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
