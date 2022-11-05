#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>

#define VISUALIZATION 1

using namespace std;

std::vector<uint64_t> read_pcd_dir(string &data_root)
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
        // cout << temp << endl;
        file_list_num.push_back(temp);
    }
    std::sort(file_list_num.begin(), file_list_num.end());
    return file_list_num;
}

int main(int argc, char const *argv[])
{
    string common_path = "/media/philip/Samsung_T5/合创/路侧数据集/白天/普通场景/银色SUV+轿车+盒子2+行人2/";
    int ratio = 2; //激光雷达原始数据10Hz，静态场景除以5（2Hz），动态场景除以2(5Hz)

    std::vector<uint64_t> file_low_num;
    string data_low_root = common_path + "low/";
    file_low_num = read_pcd_dir(data_low_root);

    cout <<"Size of low lidar pcd files=" <<file_low_num.size()<< endl;

    std::vector<uint64_t> file_high_num;
    string data_high_root = common_path + "high/";

    file_high_num = read_pcd_dir(data_high_root);

    cout << "Size of high lidar pcd files=" << file_high_num.size() << endl;

    // if (file_low_num.size() != file_high_num.size())
    // {
    //     return (-1);
    // }

    cout << "Starting Match!" << endl;

    std::vector<uint64_t> diff_num;
    for (int i = 0; i < file_low_num.size(); ++i)
    {
        // if (file_high_num[i + 1] - file_high_num[i] <90000000)
        // {
        //     cout << file_high_num[i] << endl;
        // }

        // if (file_low_num[i + 1] - file_low_num[i] < 90000000)
        // {
        //     cout << file_low_num[i] << endl;
        // }

        if (i % ratio == 0 )
        {
            std::string string_low_input(data_low_root);
            string pcd_low_name = string_low_input + std::to_string(file_low_num[i]) + ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_low(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_low_name, *cloud_low) == -1)
            {
                PCL_ERROR("Couldn't read the pcd file!\n");
                return (-1);
            }

            std::string string_high_input(data_high_root);
            string pcd_high_name = string_high_input + std::to_string(file_high_num[i]) + ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_high(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_high_name, *cloud_high) == -1)
            {
                PCL_ERROR("Couldn't read the pcd file!\n");
                return (-1);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

            *cloud = *cloud_low + *cloud_high;

            string pcd_name = common_path +"pcd/"+ std::to_string(file_low_num[i]) + ".pcd";
            pcl::io::savePCDFileASCII(pcd_name, *cloud);
            cout << "Number : " << i / ratio + 1 << endl;
        }
    }

    cout << "Finish!" << endl;

    return 1;
}
