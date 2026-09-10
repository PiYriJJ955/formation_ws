#include <ros/ros.h>
#include <nlink_parser/IotFrame0.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <map>
#include <vector>

std::ofstream csv_file;
std_msgs::Float32 msg1,msg2,msg3,msg4,msg5,msg6;

float distance3_1 ;
float f_3_1       ;
float fz_3_1      ;

float distance3_2 ;
float f_3_2       ;
float fz_3_2      ;

const std::vector<uint32_t> TARGET_UIDS = {352336640, 805334528 ,805319168, 335564288};

struct NodeData
{
    uint32_t uid     = 0;
    float    dis     = 0.0f;
    float    aoa_h   = 0.0f;
    float    aoa_v   = 0.0f;
    float    fp_rssi = 0.0f;
    float    rx_rssi = 0.0f;
    std::string user_data;
};

std::map<uint32_t, NodeData> g_node_map;

NodeData node_a, node_b, node_c;

/* ---------- 回调 ---------- */
void uwb0Callback(const nlink_parser::IotFrame0::ConstPtr& msg)
{
    for (const auto& n : msg->nodes)
    {
        NodeData d;
        d.uid     = n.uid;
        d.dis     = n.dis;
        d.aoa_h   = n.aoa_angle_horizontal;
        d.aoa_v   = n.aoa_angle_vertical;
        d.fp_rssi = n.fp_rssi;
        d.rx_rssi = n.rx_rssi;
        d.user_data = n.user_data;

        g_node_map[n.uid] = d;
    }

    /* 用统一的 find/assign，避免重复代码 */
    auto get_node = [](uint32_t id) -> NodeData {
        auto it = g_node_map.find(id);
        return (it != g_node_map.end()) ? it->second : NodeData{};
    };

    node_a = get_node(TARGET_UIDS[0]);
    node_b = get_node(TARGET_UIDS[1]);
    node_c = get_node(TARGET_UIDS[2]);

}


/* ---------- 主函数 ---------- */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "UWBdata");
    ros::NodeHandle nh;

    /* 打开 CSV，检查是否成功 */
    const char* csv_path = "/home/wheeltec/wheeltec_robot/data_UWB.csv";
    csv_file.open(csv_path, std::ios::out | std::ios::app);
    if (!csv_file.is_open())
    {
        ROS_FATAL("Cannot open %s", csv_path);
        return 1;
    }

    ros::Subscriber sub0 = nh.subscribe("nlink_iot_frame0", 1, uwb0Callback);
    ros::Publisher d31_pub = nh.advertise<std_msgs::Float32>("/distance3_1", 1);
    ros::Publisher f31_pub = nh.advertise<std_msgs::Float32>("/f_3_1", 1);
    ros::Publisher fz31_pub = nh.advertise<std_msgs::Float32>("/fz_3_1", 1);   
    ros::Publisher d32_pub = nh.advertise<std_msgs::Float32>("/distance3_2", 1);
    ros::Publisher f32_pub = nh.advertise<std_msgs::Float32>("/f_3_2", 1);
    ros::Publisher fz32_pub = nh.advertise<std_msgs::Float32>("/fz_3_2", 1);       

    ros::Rate loop(10);
    while (ros::ok())
    {
        distance3_1 = node_c.dis;   
        f_3_1       = node_c.aoa_h; 
        fz_3_1      = node_c.aoa_v; 

        msg1.data = distance3_1;
        msg2.data = f_3_1;        
        msg3.data = fz_3_1;  

        d31_pub.publish(msg1);
        f31_pub.publish(msg2);
        fz31_pub.publish(msg3);

        distance3_2 = node_a.dis;
        f_3_2       = node_a.aoa_h;
        fz_3_2      = node_a.aoa_v;

        msg4.data = distance3_2;
        msg5.data = f_3_2;        
        msg6.data = fz_3_2;   

        d32_pub.publish(msg4);
        f32_pub.publish(msg5);
        fz32_pub.publish(msg6);               

        // /* 终端打印 */
        // ROS_INFO("d31=%.3f, f31=%.3f, fz31=%.3f",distance3_1, f_3_1, fz_3_1);
        // ROS_INFO("d32=%.3f, f32=%.3f, fz32=%.3f",distance3_2, f_3_2, fz_3_2);


        /* 写 CSV */
        csv_file << distance3_1 << ',' << f_3_1 << ',' << fz_3_1 <<  ','  << distance3_2 << ',' << f_3_2 << ',' << fz_3_2 <<  '\n';

        ros::spinOnce();
        loop.sleep();
    }

    csv_file.close();
    return 0;
}