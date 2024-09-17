#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <vector>
#include <ctime>


int main(int argc, char** argv)
{
    srand(time(NULL)); // 使用当前时间初始化随机数生成器

    // 创建一个指向点云的智能指针，PointCloud 数据结构用来存储点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 随机生成点云，点的数量为1000，点云的高度为1
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height); // 设置点云的大小

    // 遍历每一个点，为每个点的 x、y、z 赋随机值
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    // 设置八叉树的分辨率为128，创建八叉树对象，用于存储和查询点云
    float resolution = 128.f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    // 将生成的点云添加到八叉树中
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // 随机生成一个搜索点
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // === 体素内近邻搜索 ===
    std::vector<int> pointindex; // 存储搜索到的点的索引
    if (octree.voxelSearch(searchPoint, pointindex)) // 体素内搜索，检查搜索点所在体素内的邻居点
    {
        std::cout << "Neighbors within voxel search at ("
            << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << std::endl;

        // 输出体素内所有邻居点的坐标
        for (size_t i = 0; i < pointindex.size(); ++i)
            std::cout << "    " << cloud->points[pointindex[i]].x
            << " " << cloud->points[pointindex[i]].y
            << " " << cloud->points[pointindex[i]].z << std::endl;
    }

    // === K 近邻搜索 ===
    int K = 10; // 设置搜索的近邻点数为 10
    std::vector<int> pointIdxNKNSearch(K); // 用于存储 K 近邻点的索引
    std::vector<float> pointNKNSquaredDistance(K); // 用于存储近邻点的距离平方值

    std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y << " " << searchPoint.z
        << ") with K=" << K << std::endl;

    // 进行 K 近邻搜索，返回找到的点数
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // 输出 K 个近邻点的坐标及其距离
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
            << " " << cloud->points[pointIdxNKNSearch[i]].y
            << " " << cloud->points[pointIdxNKNSearch[i]].z
            << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // === 半径搜索 ===
    std::vector<int> pointIdxRadiusSearch; // 存储半径搜索到的点的索引
    std::vector<float> pointRadiusSquaredDistance; // 存储搜索到的点的距离平方值
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f); // 随机生成搜索半径

    std::cout << "Neighbors within radius search at (" << searchPoint.x
        << " " << searchPoint.y << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;

    // 进行半径搜索，返回找到的点数
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        // 输出所有在该半径内的邻居点的坐标及其距离
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
            << " " << cloud->points[pointIdxRadiusSearch[i]].y
            << " " << cloud->points[pointIdxRadiusSearch[i]].z
            << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
    std::cin.get();
    return 0; // 结束程序
}
