#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
	srand(time(NULL));
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>); // cloud ָ��
	//�������� �������
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	// k-d tree
	KdTreeFLANN<PointXYZ>kdtree;
	kdtree.setInputCloud(cloud); //ָ������

	// �����
	PointXYZ searchPoint;   // �� ���
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// k��������
	int K = 10;
	std::vector<int>pointIdxNKNSearch(K);				// ����������Ե��Ƶ�����ֵ
	std::vector<float>pointNKNSquaredDistance(K);		// ��ž������ֵ
	std::cout << "K nearest neighbor search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	// kd tree ���� nearestKSearch
	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}


	// �ڰ뾶r����������
	std::vector<int> pointIdxRadiusSearch;               // ����������Ե��Ƶ�����ֵ
	std::vector<float> pointRadiusSquaredDistance;       // ��Ű뾶����ֵ
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;

	// kd tree ���� radiusSearch
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	return 0;
}
