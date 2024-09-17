#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;

int main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));

	// 八叉树分辨率 即体素的大小
	float resolution = 16.0f;

	// 1、初始化空间变化检测对象
	octree::OctreePointCloudChangeDetector<PointXYZ>octree(resolution);

	// 2、为cloudA创建点云 ，添加点云到八叉树，建立八叉树
	PointCloud<PointXYZ>::Ptr cloudA(new PointCloud<PointXYZ>);
	cloudA->width = 128;
	cloudA->height = 1;
	cloudA->points.resize(cloudA->width * cloudA->height);
	for (size_t i = 0; i < cloudA->points.size(); ++i)
	{
		cloudA->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudA->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}
	octree.setInputCloud(cloudA);
	octree.addPointsFromInputCloud();

	//  3、交换八叉树缓存，但是cloudA对应的八叉树仍在内存中
	octree.switchBuffers();

	// 4、为cloudB创建点云，添加 cloudB到八叉树
	PointCloud<PointXYZ>::Ptr cloudB(new PointCloud<PointXYZ>);
	cloudB->width = 128;
	cloudB->height = 1;
	cloudB->points.resize(cloudB->width * cloudB->height);
	for (size_t i = 0; i < cloudB->points.size(); ++i)
	{
		cloudB->points[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
		cloudB->points[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
	}
	octree.setInputCloud(cloudB); // cloudA cloudB
	octree.addPointsFromInputCloud();


	// 5、点云B ，相对于点云A，没有的体素
	std::vector<int>newPointIdxVector; // 后添加进来的点云的索引
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	//打印输出点
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size(); ++i)
	{
		std::cout << i << "# Index:" << newPointIdxVector[i]
			<< "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
			<< cloudB->points[newPointIdxVector[i]].y << " "
			<< cloudB->points[newPointIdxVector[i]].z << std::endl;
	}
}


