#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>//pcl�ٷ��ĵ���������openni_grabber.h�������ǰ�װ�İ汾��û����openni��ֻ��openni2
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/compression/octree_pointcloud_compression.h>//ʹ�ð˲����Ե������ݽ���ѹ��

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
        viewer("����ѹ��ʵ��")
    {
    }

    // ��ץȡ���������ݺ�ִ�иú�����cloudΪ�������ĵ�������
    void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
    {
        if (!viewer.wasStopped())
        {
            // �洢��ѹ�����Ƶ��ֽ�������
            std::stringstream compressedData;
            // �洢������Ƶ�ָ��
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

            // ѹ�����ơ�
            PointCloudEncoder->encodePointCloud(cloud, compressedData);

            // ���� -> ��ѹ -> ����

            // ��ѹ�����Ƶ�������ƶ���
            PointCloudDecoder->decodePointCloud(compressedData, cloudOut);


            //���ӻ��������
            viewer.showCloud(cloudOut);
        }
    }

    void
        run()
    {

        bool showStatistics = true;//�Ƿ��ڱ�׼�豸�ϴ�ӡ��ѹ�������Ϣ

        // ѹ��ѡ��������: /io/include/pcl/compression/compression_profiles.h

        //����ѹ��ѡ��Ϊ���ֱ��� 5mm����������ɫ���������߱���
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
        /*
         LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,    // �ֱ��� 1cm����������ɫ���������߱���
         LOW_RES_ONLINE_COMPRESSION_WITH_COLOR,       // �ֱ��� 1cm����������ɫ���������߱���

         MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,    // �ֱ��� 5mm����������ɫ���������߱���
         MED_RES_ONLINE_COMPRESSION_WITH_COLOR,       // �ֱ��� 5mm����������ɫ���������߱���

         HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,   // �ֱ��� 1mm����������ɫ���������߱���
         HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR,      // �ֱ��� 1mm����������ɫ���������߱���

         LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,   // �ֱ��� 1cm����������ɫ����Ч���߱���
         LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR,      // �ֱ��� 1cm����������ɫ����Ч���߱���

         MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,   // �ֱ��� 5mm����������ɫ����Ч���߱���
         MED_RES_OFFLINE_COMPRESSION_WITH_COLOR,      // �ֱ��� 5mm����������ɫ����Ч���߱���

         HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR,  // �ֱ��� 1mm����������ɫ����Ч���߱���
         HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR,     // �ֱ��� 1mm����������ɫ����Ч���߱���

         COMPRESSION_PROFILE_COUNT,
         MANUAL_CONFIGURATION                         // ����߼������������ֹ�����
        */

        //�߼���������
        /*
        * pcl::io::OctreePointCloudCompression
        * OctreePointCloudCompression (
                               compression_Profiles_e compressionProfile_arg,  // Ϊ�����ø߼����������ò������ó� MANUAL_CONFIGURATION
                               bool showStatistics_arg = false,                // �Ƿ�ѹ����ص�ͳ����Ϣ��ӡ����׼���
                               const double pointResolution_arg = 0.001,       // �����˵�����ı��뾫�ȣ��ò�������Ӧ��С�ڴ��������ȵ�ֵ
                               const double octreeResolution_arg = 0.01,       // �����˰˲��������ش�С���ϴ������ѹ�����죬��ѹ�������½����ڽϸߵ�֡�ʺ�ѹ��Ч���м��������������
                               bool doVoxelGridDownDownSampling_arg = false,   // �Ƿ��ڵ���ѹ���ڼ�����²������ﵽ�ϸߵ�ѹ������
                               const unsigned int iFrameRate_arg = 30,         // ������
                               bool doColorEncoding_arg = true,                // �Ƿ����ò�ɫ����ɷֱ���ѹ��
                               const unsigned char colorBitResolution_arg = 6  // ��ɫ�ֱ��ʣ�ÿһ����ɫ�ɷֱ������ռ������
                               )
        */


        //��ʼ��ѹ���ͽ�ѹ������  ����ѹ��������Ҫ�趨ѹ������ѡ���ѹ����������Դ�����ж�
        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

        //Ϊopenni2�豸����һ���µĲ���������
        pcl::Grabber* interfaceg = new pcl::io::OpenNI2Grabber();

        //ѭ���ص��ӿڣ�ÿ���豸��ȡһ֡���ݾͻص�����һ�Σ�����Ļص���������ʵ������ѹ���Ϳ��ӻ���ѹ�������
        boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
            boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

        //�����ص������ͻص���Ϣ�İ�
        boost::signals2::connection c = interfaceg->registerCallback(f);

        //��ʼ���ܵ��Ƶ�������
        interfaceg->start();

        while (!viewer.wasStopped())
        {
            sleep(1);
        }

        interfaceg->stop();

        //ɾ��ѹ�����ѹ����ʵ��
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

