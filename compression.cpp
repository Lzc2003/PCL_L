#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>//pcl官方文档给出的是openni_grabber.h，但我们安装的版本里没有了openni，只有openni2
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>//使用八叉树对点云数据进行压缩

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include<functional>
using std::placeholders::_1;
#ifdef WIN32
# define sleep(x) Sleep((x)*1000)
#endif

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer() :
        viewer("点云压缩实例")
    {
    }

    // 当抓取到点云数据后，执行该函数，cloud为传进来的点云数据
    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
    {
        if (!viewer.wasStopped())
        {
            // 存储待压缩点云的字节流对象
            std::stringstream compressedData;
            // 存储输出点云的指针
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

            // 压缩点云、
            PointCloudEncoder->encodePointCloud(cloud, compressedData);

            // 传输 -> 解压 -> 处理

            // 解压缩点云到输出点云对象
            PointCloudDecoder->decodePointCloud(compressedData, cloudOut);


            //可视化输出点云
            viewer.showCloud(cloudOut);
        }
    }

    void
        run()
    {

        bool showStatistics = true;//是否在标准设备上打印出压缩结果信息

        // 压缩选项详情在: /io/include/pcl/compression/compression_profiles.h

        //设置压缩选项为：分辨率 5mm立方，有颜色，快速在线编码
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
        /*
         LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,    // 分辨率 1cm立方，无颜色，快速在线编码
         LOW_RES_ONLINE_COMPRESSION_WITH_COLOR,       // 分辨率 1cm立方，有颜色，快速在线编码

         MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,    // 分辨率 5mm立方，无颜色，快速在线编码
         MED_RES_ONLINE_COMPRESSION_WITH_COLOR,       // 分辨率 5mm立方，有颜色，快速在线编码

         HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,   // 分辨率 1mm立方，无颜色，快速在线编码
         HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR,      // 分辨率 1mm立方，有颜色，快速在线编码

         LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,   // 分辨率 1cm立方，无颜色，高效离线编码
         LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR,      // 分辨率 1cm立方，有颜色，高效离线编码

         MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,   // 分辨率 5mm立方，无颜色，高效离线编码
         MED_RES_OFFLINE_COMPRESSION_WITH_COLOR,      // 分辨率 5mm立方，有颜色，高效离线编码

         HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,  // 分辨率 1mm立方，无颜色，高效离线编码
         HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR,     // 分辨率 1mm立方，有颜色，高效离线编码

         COMPRESSION_PROFILE_COUNT,
         MANUAL_CONFIGURATION                         // 允许高级参数化进行手工配置
        */

        //高级参数配置
        /*
        * pcl::io::OctreePointCloudCompression
        * OctreePointCloudCompression (
                               compression_Profiles_e compressionProfile_arg,  // 为了启用高级参数化，该参数设置成 MANUAL_CONFIGURATION
                               bool showStatistics_arg = false,                // 是否将压缩相关的统计信息打印到标准输出
                               const double pointResolution_arg = 0.001,       // 定义了点坐标的编码精度，该参数设置应该小于传感器精度的值
                               const double octreeResolution_arg = 0.01,       // 定义了八叉树的体素大小，较大的体素压缩更快，但压缩质量下降，在较高的帧率和压缩效率中间进行了折中设置
                               bool doVoxelGridDownDownSampling_arg = false,   // 是否在点云压缩期间进行下采样，达到较高的压缩性能
                               const unsigned int iFrameRate_arg = 30,         // 编码率
                               bool doColorEncoding_arg = true,                // 是否启用彩色纹理成分编码压缩
                               const unsigned char colorBitResolution_arg = 6  // 颜色分辨率，每一个彩色成分编码后所占比特率
                               )
        */


        //初始化压缩和解压缩对象  其中压缩对象需要设定压缩参数选项，解压缩按照数据源自行判断
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

        //为openni2设备创建一个新的采样器对象
        pcl::Grabber* interfaceg = new pcl::io::OpenNI2Grabber();

        //循环回调接口，每从设备获取一帧数据就回调函数一次，这里的回调函数就是实现数据压缩和可视化解压缩结果。
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

        //建立回调函数和回调信息的绑定
        boost::signals2::connection c = interfaceg->registerCallback(f);

        //开始接受点云的数据流
        interfaceg->start();

        while (!viewer.wasStopped())
        {
            sleep(1);
        }

        interfaceg->stop();

        //删除压缩与解压缩的实例
        delete (PointCloudEncoder);
        delete (PointCloudDecoder);

    }

    pcl::visualization::CloudViewer viewer;

    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;

};



int main(int argc, char** argv)
{
    SimpleOpenNIViewer v;
    v.run();

    return (0);
}

