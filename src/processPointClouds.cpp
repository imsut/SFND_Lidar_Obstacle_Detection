// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kd_tree.h"
#include <unordered_set>
#include <boost/qvm/all.hpp>

const Eigen::Vector4f kRoofMinPoint(-1.5, -1.7, -1, 1);
const Eigen::Vector4f kRoofMaxPoint(2.6, 1.7, 0, 1);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud,
    float filterRes,
    Eigen::Vector4f minPoint,
    Eigen::Vector4f maxPoint) const
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_no_roof(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud_filtered);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.filter(*cloud_cropped);

    // remove "roof"
    auto indices = boost::make_shared<std::vector<int>>();
    pcl::CropBox<PointT> roof;
    roof.setInputCloud(cloud_cropped);
    roof.setMin(kRoofMinPoint);
    roof.setMax(kRoofMaxPoint);
    roof.filter(*indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud_no_roof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_no_roof;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers,
                                           typename pcl::PointCloud<PointT>::Ptr cloud) const
{
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr segmented(new pcl::PointCloud<PointT>);

    std::set<int> indices(inliers->indices.begin(), inliers->indices.end());
    for (int i = 0; i < cloud->size(); ++i) {
        if (indices.find(i) == indices.end()) {
            obstacles->push_back(cloud->at(i));
        } else {
            segmented->push_back(cloud->at(i));
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstacles, segmented);
}

template<typename Point>
static std::vector<int> Sample(size_t num, size_t out_of) {
    std::unordered_set<int> sampled;
    while (sampled.size() < num) {
        sampled.insert(rand() % out_of);
    }

    return std::vector<int>(sampled.begin(), sampled.end());
}

template<typename Point>
static pcl::PointIndicesPtr Ransac3d(typename pcl::PointCloud<Point>::Ptr cloud,
                                     int maxIterations,
                                     float distanceTol)
{
    srand(time(NULL));

    auto max_inliers = std::make_shared<std::unordered_set<int>>();

    // For max iterations
    while (maxIterations--) {
        // Randomly sample subset and fit line
        auto samples = Sample<Point>(3, cloud->points.size());
        const auto& pt0 = cloud->at(samples.at(0));
        const auto& pt1 = cloud->at(samples.at(1));
        const auto& pt2 = cloud->at(samples.at(2));

        // fit a plane
        boost::qvm::vec<float, 3> v1 = {pt1.x - pt0.x, pt1.y - pt0.y, pt1.z - pt0.z};
        boost::qvm::vec<float, 3> v2 = {pt2.x - pt0.x, pt2.y - pt0.y, pt2.z - pt0.z};
        auto cross = boost::qvm::cross(v1, v2);

        const float a = cross.a[0];
        const float b = cross.a[1];
        const float c = cross.a[2];
        const float d = -(a * pt0.x + b * pt0.y + c * pt0.z);
        const float denom = std::sqrt(a * a + b * b + c * c);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        auto inliers = std::make_shared<std::unordered_set<int>>();
        for (int i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->at(i);
            float dist = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / denom;
            if (dist <= distanceTol) {
                inliers->insert(i);
            }
        }

        if (inliers->size() > max_inliers->size()) {
            max_inliers = inliers;
        }
    }

    // Return indicies of inliers from fitted line with most inliers
    pcl::PointIndicesPtr plane(new pcl::PointIndices());
    std::copy(max_inliers->begin(), max_inliers->end(), std::back_inserter(plane->indices));
    return plane;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                         int maxIterations,
                                         float distanceThreshold) const
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers = Ransac3d<PointT>(cloud, maxIterations, distanceThreshold);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return SeparateClouds(inliers, cloud);
}

template<typename PointT>
static std::vector<std::vector<int>> EuclideanCluster(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        const KdTree<PointT>& tree,
        float distanceTol,
        int minSize,
        int maxSize) {
    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> processed;

    for (size_t id = 0; id < cloud->points.size(); ++id) {
        const auto& point = cloud->points.at(id);

        if (processed.find(id) == processed.cend()) {
            processed.insert(id);

            std::vector<int> cluster;
            cluster.push_back(id);

            for (size_t idx = 0; idx < cluster.size() and cluster.size() <= maxSize; ++idx) {
                const auto neighbor = tree.search(
                    cloud->points.at(cluster.at(idx)), distanceTol);
                for (const auto& nid : neighbor) {
                    if (processed.find(nid) == processed.cend()) {
                        processed.insert(nid);
                        cluster.push_back(nid);
                    }
                }
            }

            if (cluster.size() >= minSize) {
                clusters.push_back(cluster);
            }
        }
    }

    return clusters;
}

static void printElapsedTime(
    const std::chrono::time_point<std::chrono::steady_clock>& start,
    const std::string& action) {
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << action << " took " << elapsed.count() << " milliseconds." << std::endl;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       float clusterTolerance,
                                       int minSize,
                                       int maxSize) const
{
    // Time clustering process
    auto start0 = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    KdTree<PointT> kdtree;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& point = cloud->points.at(i);
        kdtree.insert(point, i);
    }

    printElapsedTime(start0, "creating KDtree");
    auto start1 = std::chrono::steady_clock::now();

    std::vector<std::vector<int>> cluster_indices = EuclideanCluster<PointT>(
        cloud, kdtree, clusterTolerance, minSize, maxSize);

    printElapsedTime(start1, "euclidean clustering");
    auto start2 = std::chrono::steady_clock::now();

    for (const auto& indices : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (const auto& i : indices) {
            cluster->points.push_back(cloud->points[i]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    printElapsedTime(start2, "post processing");

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) const
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) const
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) const
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) const
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
