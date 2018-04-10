//--------------------------------------------------
//物体数量计算的ROS节点
//
//author: Vance Wu
//date: 17/08/04
//--------------------------------------------------

#include <roch_gpsr/common_include.h>
#include <roch_gpsr/common_function.h>

//--------------------------------------------------
// 计算物体数量的类
//--------------------------------------------------
class ObjDetect{
private:
    ros::NodeHandle nh_;

   
    ros::Subscriber sub_dep_img_;
    ros::Publisher pub_det_point_;
    ros::Publisher pub_det_plane_;
    ros::Publisher pub_det_objs_;

    ros::Publisher pub_down;
    CommonFunction* CommonFunction_;

public:
    ObjDetect();
    ~ObjDetect();

    void sub_CB(const sensor_msgs::PointCloud2ConstPtr& dep_point_ptr);
};

ObjDetect::ObjDetect(){
    ROS_INFO("Detecting objects...");

    sub_dep_img_    = nh_.subscribe("/camera/depth_registered/points", 1, &ObjDetect::sub_CB, this);

    pub_det_point_  = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/area", 1);         
    pub_det_plane_  = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/plane", 1);
    pub_det_objs_   = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/objs", 1);
    pub_down        = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/down", 1);
    ros::Duration(2).sleep();
    ROS_INFO("Constuctor done...");
}

ObjDetect::~ObjDetect(){
}

void ObjDetect::sub_CB(const sensor_msgs::PointCloud2ConstPtr& dep_point_ptr){ 

    // ROS msg 转到 pcl data 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*dep_point_ptr, *i_pc);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*i_pc));//指针指向pcl格式的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objs_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


    //下采样
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(i_pc_ptr);
    vg.setLeafSize(0.01, 0.01, 0.01); 
    vg.filter(*plane_pc_ptr);

    //直通滤波
    ROS_INFO("Passing through...");
    pcl::PassThrough<pcl::PointXYZRGB> pass_th;
    pass_th.setInputCloud(plane_pc_ptr);
    pass_th.setFilterFieldName("x");
    pass_th.setFilterLimits(-0.3, 0.3);
    pass_th.setFilterLimitsNegative(false);
    pass_th.filter(*plane_pc_ptr);

    // pass_th.setInputCloud(plane_pc_ptr);
    // pass_th.setFilterFieldName("y");
    // pass_th.setFilterLimits(-0.1, 0.2);
    // pass_th.setFilterLimitsNegative(false);
    // pass_th.filter(*plane_pc_ptr);

    pass_th.setInputCloud(plane_pc_ptr);
    pass_th.setFilterFieldName("z");
    pass_th.setFilterLimits(0.3, 2);
    pass_th.setFilterLimitsNegative(false);
    pass_th.filter(*plane_pc_ptr);

    //转成ROS msg以发布
    sensor_msgs::PointCloud2 det_area;
    pcl::toROSMsg(*plane_pc_ptr, det_area);
    det_area.header.frame_id = "camera_depth_optical_frame";
    pub_det_point_.publish(det_area);


    //复制点云
    pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*plane_pc_ptr, *objs_pc_ptr);  

    //检测平面
    ROS_INFO("Detecting the plane...");
    pcl::ModelCoefficients::Ptr model_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    //分割对象平面
    pcl::SACSegmentation<pcl::PointXYZRGB> sac_seg;
    sac_seg.setModelType(pcl::SACMODEL_PLANE);
    sac_seg.setMethodType(pcl::SAC_RANSAC);
    sac_seg.setDistanceThreshold(0.01); 
    sac_seg.setOptimizeCoefficients(true);
    sac_seg.setInputCloud(objs_pc_ptr);
    sac_seg.segment (*indices, *model_coef);

    //平面没有检测的情况
    if(indices->indices.size() == 0){
        ROS_INFO("[obj_detect]: There is no plane.");
        return;
    }

    //平面上色
    for(size_t i = 0; i < indices->indices.size(); ++i){
        plane_pc_ptr->points[indices->indices[i]].r = 0;
        plane_pc_ptr->points[indices->indices[i]].g = 0;
        plane_pc_ptr->points[indices->indices[i]].b = 255;
    }

    //发布蓝色平面topic
    ROS_INFO("Publishing the plane...");
    sensor_msgs::PointCloud2 det_plane;
    pcl::toROSMsg(*plane_pc_ptr, det_plane);
    det_plane.header.frame_id = "camera_depth_optical_frame";
    pub_det_plane_.publish(det_plane);


    //平面除去
    pcl::ExtractIndices<pcl::PointXYZRGB> ext;
    ext.setInputCloud(objs_pc_ptr);
    ext.setIndices(indices);
    ext.setNegative(true); 
    ext.filter(*objs_pc_ptr);

    //发布具有多个物体的点云
    sensor_msgs::PointCloud2 det_objs;
    pcl::toROSMsg(*objs_pc_ptr, det_objs);
    det_objs.header.frame_id = "camera_depth_optical_frame";
    pub_det_objs_.publish(det_objs);
}









//--------------------------------------------------
//主函数
//--------------------------------------------------
int main (int argc, char** argv){
    ros::init (argc, argv, "object_detect");

    ObjDetect ObjDetect;

    while(ros::ok()){
        ros::spinOnce(); 
    }
}
