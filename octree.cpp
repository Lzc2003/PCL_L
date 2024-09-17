#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <vector>
#include <ctime>


int main(int argc, char** argv)
{
    srand(time(NULL)); // ʹ�õ�ǰʱ���ʼ�������������

    // ����һ��ָ����Ƶ�����ָ�룬PointCloud ���ݽṹ�����洢����
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ������ɵ��ƣ��������Ϊ1000�����Ƶĸ߶�Ϊ1
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height); // ���õ��ƵĴ�С

    // ����ÿһ���㣬Ϊÿ����� x��y��z �����ֵ
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    // ���ð˲����ķֱ���Ϊ128�������˲����������ڴ洢�Ͳ�ѯ����
    float resolution = 128.f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

    // �����ɵĵ�����ӵ��˲�����
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // �������һ��������
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // === �����ڽ������� ===
    std::vector<int> pointindex; // �洢�������ĵ������
    if (octree.voxelSearch(searchPoint, pointindex)) // ������������������������������ڵ��ھӵ�
    {
        std::cout << "Neighbors within voxel search at ("
            << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << ")" << std::endl;

        // ��������������ھӵ������
        for (size_t i = 0; i < pointindex.size(); ++i)
            std::cout << "    " << cloud->points[pointindex[i]].x
            << " " << cloud->points[pointindex[i]].y
            << " " << cloud->points[pointindex[i]].z << std::endl;
    }

    // === K �������� ===
    int K = 10; // ���������Ľ��ڵ���Ϊ 10
    std::vector<int> pointIdxNKNSearch(K); // ���ڴ洢 K ���ڵ������
    std::vector<float> pointNKNSquaredDistance(K); // ���ڴ洢���ڵ�ľ���ƽ��ֵ

    std::cout << "K nearest neighbor search at (" << searchPoint.x
        << " " << searchPoint.y << " " << searchPoint.z
        << ") with K=" << K << std::endl;

    // ���� K ���������������ҵ��ĵ���
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        // ��� K �����ڵ�����꼰�����
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
            << " " << cloud->points[pointIdxNKNSearch[i]].y
            << " " << cloud->points[pointIdxNKNSearch[i]].z
            << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }

    // === �뾶���� ===
    std::vector<int> pointIdxRadiusSearch; // �洢�뾶�������ĵ������
    std::vector<float> pointRadiusSquaredDistance; // �洢�������ĵ�ľ���ƽ��ֵ
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f); // ������������뾶

    std::cout << "Neighbors within radius search at (" << searchPoint.x
        << " " << searchPoint.y << " " << searchPoint.z
        << ") with radius=" << radius << std::endl;

    // ���а뾶�����������ҵ��ĵ���
    if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        // ��������ڸð뾶�ڵ��ھӵ�����꼰�����
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
            << " " << cloud->points[pointIdxRadiusSearch[i]].y
            << " " << cloud->points[pointIdxRadiusSearch[i]].z
            << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }
    std::cin.get();
    return 0; // ��������
}
