#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define VISUALIZATION 1

using namespace std;

string data_root = "/media/philip/Samsung_T5/合创/路侧数据集/黑夜/普通场景/银色SUV+MPV+盒子2+行人2/pcd/";

int main(int argc, char const *argv[])
{
    if (data_root.empty())
        std::cout << "file open error" << std::endl;
    //文件读取接口初始化
    DIR *dir;
    struct dirent *ptr;
    std::vector<std::string> file_list;
    FILE *pFd = NULL;
    const char *p = data_root.c_str();
    dir = opendir(p);
    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
        {
            continue;
        }
        if (strcmp(ptr->d_name, "pcd") == 0)
        {
            continue;
        }
        file_list.push_back(ptr->d_name);
    }
    closedir(dir);
    std::vector<uint64_t> file_list_num;
    for (int i = 0; i < file_list.size(); i++)
    {
        file_list[i] = file_list[i].substr(0, file_list[i].rfind("."));

        uint64_t temp;
        std::istringstream iss(file_list[i].c_str());
        iss >> temp;
        file_list_num.push_back(temp);
    }
    std::sort(file_list_num.begin(), file_list_num.end());

#if VISUALIZATION
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
#endif

    for (int num_index = 0; num_index < file_list.size(); num_index++)
    {
        printf("num index = %d \n", num_index + 1);
        std::string string_input(data_root);
        string pcd_name = string_input + std::to_string(file_list_num[num_index]) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr initial_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_name, *initial_cloud) == -1)
        {
            PCL_ERROR("Couldn't read the pcd file!\n");
            return (-1);
        }
        /*
        TODO: OPERATION
        */
#if VISUALIZATION
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> init_color(initial_cloud, 255, 255, 255);
        viewer->addPointCloud<pcl::PointXYZI>(initial_cloud, init_color, "initial cloud");
        viewer->spinOnce(1);
#endif
    }
    return 1;
}
