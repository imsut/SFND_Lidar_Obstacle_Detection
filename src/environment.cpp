/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <chrono>
#include <thread>

#include <memory>
#include <boost/program_options.hpp>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>& pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
               const boost::program_options::variables_map& vm) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(
        inputCloud,
        0.1,
        Eigen::Vector4f(-10.0, -10.0, -3.0, 1.0),
        Eigen::Vector4f(20.0, 10.0, 3.0, 1.0));
    //renderPointCloud(viewer, filterCloud, "filterCloud");

    auto segmentCloud = pointProcessorI.SegmentPlane(
        filterCloud,
        vm["max-iteration"].as<int>(),
        vm["distance-threshold"].as<double>());
    renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));
    //renderPointCloud(viewer, segmentCloud.first, "obst", Color(1, 1, 1));

    auto clusters = pointProcessorI.Clustering(
        segmentCloud.first,
        vm["cluster-tolerance"].as<float>(),
        vm["cluster-minsize"].as<int>(),
        vm["cluster-maxsize"].as<int>());

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};

    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters.at(i);
        std::cout << "cluster " << i << ": size = " << cluster->points.size() << std::endl;
        renderPointCloud(
            viewer,
            cluster,
            "obstCloud" + std::to_string(i),
            colors[i % colors.size()]);

        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, i);
    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer,
                   const boost::program_options::variables_map& vm)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);

    // TODO:: Create point processor
    auto cloud = lidar->scan();
#if 0
    renderRays(viewer, lidar->position, cloud);
    renderPointCloud(viewer, cloud, "pcd");
#endif

    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    auto segmentCloud = pointProcessor.SegmentPlane(
        cloud,
        vm["max-iteration"].as<int>(),
        vm["distance-threshold"].as<double>());
#if 0
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
#endif

    auto clusters = pointProcessor.Clustering(
        segmentCloud.first,
        vm["cluster-tolerance"].as<float>(),
        vm["cluster-minsize"].as<int>(),
        vm["cluster-maxsize"].as<int>());

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (int i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters.at(i);
        std::cout << "cluster size " << cluster->points.size() << std::endl;
        renderPointCloud(
            viewer,
            cluster,
            "obstCloud" + std::to_string(i),
            colors[i % colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, i);
    }

    delete lidar;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("max-iteration", po::value<int>()->default_value(100), "iteration")
        ("distance-threshold", po::value<double>()->default_value(0.2), "distance threshold")
        ("cluster-tolerance", po::value<float>()->default_value(1.0), "cluster tolerance")
        ("cluster-minsize", po::value<int>()->default_value(3), "minimum cluster size")
        ("cluster-maxsize", po::value<int>()->default_value(3000), "maximum cluster size")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //CameraAngle setAngle = XY;
    CameraAngle setAngle = TopDown;
    initCamera(setAngle, viewer);

    //simpleHighway(viewer, vm);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //renderPointCloud(viewer, inputCloud,"inputCloud");

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI, vm);

        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
