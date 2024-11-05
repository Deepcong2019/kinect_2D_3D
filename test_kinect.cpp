#pragma warning(disable : 4996)
#include <iostream>
#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
#include <io.h>
#include <vector>
#include <direct.h>
#include <math.h>
#include <sstream>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 //Kinect DK
#include <k4a/k4a.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>

//定义点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace cv;
using namespace std;

PointT point;
PointT point_center;


int main(int argc, char* argv[]) {
    /*
    找到并打开 Azure Kinect 设备
*/
// 发现已连接的设备数

    const uint32_t device_count = k4a::device::get_installed_count();


    if (0 == device_count) {
        std::cout << "Error: no K4A devices found. " << std::endl;
        return -1;
    }
    else {
        std::cout << "Found " << device_count << " connected devices. " << std::endl;
        if (1 != device_count)// 超过1个设备，也输出错误信息。
        {
            std::cout << "Error: more than one K4A devices found. " << std::endl;
            return -1;
        }
        else// 该示例代码仅限对1个设备操作
        {
            std::cout << "Done: found 1 K4A device. " << std::endl;
        }
    }

    // 打开（默认）设备
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    std::cout << "Done: open device. " << std::endl;

    /*
        检索并保存 Azure Kinect 图像数据
    */
    // 配置并启动设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    //config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    //config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
    device.start_cameras(&config);
    std::cout << "Done: start camera." << std::endl;

    //写入txt文件流
    ofstream rgb_out;
    ofstream d_out;

    rgb_out.open("./rgb.txt");
    d_out.open("./depth.txt");

    rgb_out << "#  color images" << endl;
    rgb_out << "#  file: rgbd_dataset" << endl;
    rgb_out << "#  timestamp" << "    " << "filename" << endl;

    d_out << "#  depth images" << endl;
    d_out << "#  file: rgbd_dataset" << endl;
    d_out << "#  timestamp" << "    " << "filename" << endl;

    rgb_out << flush;
    d_out << flush;
    // 稳定化
    k4a::capture capture;
    int iAuto = 0;//用来稳定，类似自动曝光
    int iAutoError = 0;// 统计自动曝光的失败次数
    while (true) {
        if (device.get_capture(&capture)) {
            std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;

            // 跳过前 n 个（成功的数据采集）循环，用来稳定
            if (iAuto != 30) {
                iAuto++;
                continue;
            }
            else {
                std::cout << "Done: auto-exposure" << std::endl;
                break;// 跳出该循环，完成相机的稳定过程
            }

        }
        else {
            std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
            if (iAutoError != 30) {
                iAutoError++;
                continue;
            }
            else {
                std::cout << "Error: failed to give auto-exposure. " << std::endl;
                return -1;
            }
        }
    }
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "----- Have Started Kinect DK. -----" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    // 从设备获取捕获
    k4a::image rgbImage;
    k4a::image depthImage;
    //k4a::image irImage;
    k4a::image transformed_depthImage;

    cv::Mat cv_rgbImage_with_alpha;
    cv::Mat cv_rgbImage_no_alpha;
    cv::Mat cv_depth;
    cv::Mat cv_depth_8U;

    // 随机指定一个目标框,用于切割出点云（x,y,w,h）
    cv::Rect object1(448,388,197,264);
    // 计算出中心点坐标
    int center_x = int(object1.x + object1.width / 2);
    int center_y = int(object1.y + object1.height / 2);
    int size_x = object1.width;
    int size_y = object1.height;
    

    int index = 0;
    while (index < 1) {
        if (device.get_capture(&capture)) {
            // rgb
            // * Each pixel of BGRA32 data is four bytes. The first three bytes represent Blue, Green,
            // * and Red data. The fourth byte is the alpha channel and is unused in the Azure Kinect APIs.
            rgbImage = capture.get_color_image();

            std::cout << "[rgb] " << "\n"
            << "format: " << rgbImage.get_format() << "\n"
            << "device_timestamp: " << rgbImage.get_device_timestamp().count() << "\n"
            << "system_timestamp: " << rgbImage.get_system_timestamp().count() << "\n"
            << "height*width: " << rgbImage.get_height_pixels() << ", " << rgbImage.get_width_pixels()
            << std::endl;

            uint8_t* color_buffer = rgbImage.get_buffer(); // 获取颜色图像数据的指针
            int width = rgbImage.get_width_pixels();
            int height = rgbImage.get_height_pixels();
            std::cout << "width: " << std::endl;

            for (int i = 0; i < 10; i++) // 只打印前10个像素
            {
                int index = i * 4; // 每个像素占用4个字节（R、G、B 和 Alpha通道）
                std::cout << "Pixel[" << i << "]: ("
                    << (int)color_buffer[index] << ", "  // R
                    << (int)color_buffer[index + 1] << ", " // G
                    << (int)color_buffer[index + 2] << ") " // B
                    << std::endl;
            }
            cv::Mat cv_rgbImage_with_alpha = cv::Mat(height, width, CV_8UC4, color_buffer, cv::Mat::AUTO_STEP);
            cv::Mat cv_image_no_alpha;
            cv::cvtColor(cv_rgbImage_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
            cv::imwrite("a.jpg", cv_image_no_alpha);
            cv::imshow("color", cv_image_no_alpha);
            cv::waitKey(1);
            // depth
            // * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
            // * millimeters from the origin of the camera.
            depthImage = capture.get_depth_image();

            std::cout << "[depth] " << "\n"
                << "format: " << depthImage.get_format() << "\n"
                << "device_timestamp: " << depthImage.get_device_timestamp().count() << "\n"
                << "system_timestamp: " << depthImage.get_system_timestamp().count() << "\n"
                << "height*width: " << depthImage.get_height_pixels() << ", " << depthImage.get_width_pixels()
                << std::endl;

            //获取彩色点云
            k4a::calibration k4aCalibration = device.get_calibration(config.depth_mode, config.color_resolution);
            float fx = k4aCalibration.color_camera_calibration.intrinsics.parameters.param.fx;
            float fy = k4aCalibration.color_camera_calibration.intrinsics.parameters.param.fy;
            std::cout << "fx: " << fx << "fy: " << fy << std::endl;
            k4a::transformation k4aTransformation = k4a::transformation(k4aCalibration);

            //PointCloud::Ptr cloud(new PointCloud);
            int color_image_width_pixels = rgbImage.get_width_pixels();
            int color_image_height_pixels = rgbImage.get_height_pixels();
            transformed_depthImage = NULL;
            transformed_depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * (int)sizeof(uint16_t));
            k4a::image point_cloud_image = NULL;
            point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                color_image_width_pixels,
                color_image_height_pixels,
                color_image_width_pixels * 3 * (int)sizeof(int16_t));

            if (depthImage.get_width_pixels() == rgbImage.get_width_pixels() && depthImage.get_height_pixels() == rgbImage.get_height_pixels()) {
                std::copy(depthImage.get_buffer(), depthImage.get_buffer() + depthImage.get_height_pixels() * depthImage.get_width_pixels() * (int)sizeof(uint16_t), transformed_depthImage.get_buffer());
            }
            else {
                k4aTransformation.depth_image_to_color_camera(depthImage, &transformed_depthImage);
            }
            k4aTransformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            cloud->width = color_image_width_pixels;
            cloud->height = color_image_height_pixels;
            cloud->is_dense = false;
            cloud->resize(static_cast<size_t>(color_image_width_pixels) * color_image_height_pixels);


            const int16_t* point_cloud_image_data = reinterpret_cast<const int16_t*>(point_cloud_image.get_buffer());
            const uint8_t* color_image_data = rgbImage.get_buffer();


            //// 创建一个 PCLVisualizer 对象
            pcl::visualization::PCLVisualizer viewer("3D Viewer");


            //// 创建表示框的模型系数
            pcl::ModelCoefficients coefficients;
            coefficients.values.resize(6); // 6个值分别表示 x_min, x_max, y_min, y_max, z_min, z_max

            for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {

                if (i == center_x + object1.width * center_y) {
                    point_center.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
                    point_center.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
                    point_center.z = point_cloud_image_data[3 * i + 2] / 1000.0f;
                    std::cout << "point_center.x:" << point_center.x << "  point_center.y: "<< point_center.y<<"  point_center.z: " <<point_center.z << std::endl;
                    float D = point_center.z;
                    std::cout << "D * (size_x / fx);" << D * (size_x / fx) << std::endl;
                    float W = D * (size_x / fx);
                    float H = D * (size_y / fy);
                    std::cout << "W:  " << W << "H: " << H << "D: " << D << std::endl;
                    //// 定义框的最大和最小坐标
                    coefficients.values[0] = -(point_center.x + W / 2);  // x_min
                    coefficients.values[1] = -(point_center.x - W / 2);  // x_max
                    coefficients.values[2] = (point_center.y - H/2);  // y_min
                    coefficients.values[3] = (point_center.y + H/2);  // y_max
                    std::cout << "y_min: " << coefficients.values[2] << "y_max: " << coefficients.values[3] << std::endl;
                    coefficients.values[4] = -point_center.z-0.5 ;  // z_min
                    coefficients.values[5] = -point_center.z  ;  // z_max


                }
                point.x = -point_cloud_image_data[3 * i + 0] / 1000.0f;
                point.y = -point_cloud_image_data[3 * i + 1] / 1000.0f;
                point.z = -point_cloud_image_data[3 * i + 2] / 1000.0f;

                point.b = color_image_data[4 * i + 0];
                point.g = color_image_data[4 * i + 1];
                point.r = color_image_data[4 * i + 2];
                uint8_t alpha = color_image_data[4 * i + 3];
                if (point.x == 0 && point.y == 0 && point.z == 0 && alpha == 0)
                    continue;
                cloud->points[i] = point;
            }
            





            pcl::io::savePLYFile("4.ply", *cloud);   //将点云数据保存为ply文件
          





            

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);

            viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");

            //// 在可视化窗口中添加框
            viewer.addCube(coefficients.values[0], coefficients.values[1],
                coefficients.values[2], coefficients.values[3],
                coefficients.values[4], coefficients.values[5],
                1.0, 0.0, 0.0, "cube"); // 使用红色绘制框
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, "cube"); // 设置线宽
            //// 循环直到关闭窗口
            while (!viewer.wasStopped()) {
                viewer.spinOnce(100);
            }


        }
        else {
            std::cout << "false: K4A_WAIT_RESULT_TIMEOUT." << std::endl;
        }
        index++;
    }
    cv::destroyAllWindows();
    rgb_out << flush;
    d_out << flush;
    rgb_out.close();
    d_out.close();

    // 释放，关闭设备
    rgbImage.reset();
    depthImage.reset();
    capture.reset();
    device.close();

    return 1;
}

