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
    //showCloud函数是同步的，在此处等待直到渲染显示为止
    viewer.showCloud(cloud);
    //该注册函数在可视化时只调用一次
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
    //该注册函数在渲染输出时每次都调用
    viewer.runOnVisualizationThread(viewerPsycho);
    while (!viewer.wasStopped())
    {
        //在此处可以添加其他处理
        user_data++;
    }
    return 0;
}

