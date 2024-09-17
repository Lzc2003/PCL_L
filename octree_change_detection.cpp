#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;

int main(int argc, char** argv)
{
	srand((unsigned int)time(NULL));

	// �˲����ֱ��� �����صĴ�С
	float resolution = 16.0f;

	// 1����ʼ���ռ�仯������
	octree::OctreePointCloudChangeDetector<PointXYZ>octree(resolution);

	// 2��ΪcloudA�������� ����ӵ��Ƶ��˲����������˲���
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

	//  3�������˲������棬����cloudA��Ӧ�İ˲��������ڴ���
	octree.switchBuffers();

	// 4��ΪcloudB�������ƣ���� cloudB���˲���
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


	// 5������B ������ڵ���A��û�е�����
	std::vector<int>newPointIdxVector; // ����ӽ����ĵ��Ƶ�����
	octree.getPointIndicesFromNewVoxels(newPointIdxVector);

	//��ӡ�����
	std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
	for (size_t i = 0; i < newPointIdxVector.size(); ++i)
	{
		std::cout << i << "# Index:" << newPointIdxVector[i]
			<< "  Point:" << cloudB->points[newPointIdxVector[i]].x << " "
			<< cloudB->points[newPointIdxVector[i]].y << " "
			<< cloudB->points[newPointIdxVector[i]].z << std::endl;
	}
}


