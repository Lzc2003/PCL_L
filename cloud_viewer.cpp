#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
int user_data;
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
    //FIXME: possible race condition here:
    user_data++;
}


int main()
{
    PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>);
    pcl::io::loadPCDFile("maize.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    //showCloud������ͬ���ģ��ڴ˴��ȴ�ֱ����Ⱦ��ʾΪֹ
    viewer.showCloud(cloud);
    //��ע�ắ���ڿ��ӻ�ʱֻ����һ��
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //��ע�ắ������Ⱦ���ʱÿ�ζ�����
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped())
    {
        //�ڴ˴����������������
        user_data++;
    }
    return 0;
}

