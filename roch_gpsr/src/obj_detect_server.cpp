//--------------------------------------------------
//物体数量计算的ROS节点
//
//author: Vance Wu
//date: 17/08/04
//--------------------------------------------------

#include <roch_gpsr/common_include.h>
#include <roch_gpsr/common_function.h>

#include <dynamic_reconfigure/server.h>
#include <roch_gpsr/ObjDetectConfig.h>
#include <boost/thread.hpp> 
//--------------------------------------------------
// 计算物体数量的类
//--------------------------------------------------
class ObjDetect{
protected:
    dynamic_reconfigure::Server<roch_gpsr::ObjDetectConfig> dynamic_server_;
    dynamic_reconfigure::Server<roch_gpsr::ObjDetectConfig>::CallbackType f_;
    ros::Subscriber sub_dep_img_; 
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_det_point_;
    ros::Publisher pub_det_plane_;
    ros::Publisher pub_det_objs_;
    ros::Publisher pub_down_point_;
    ros::Rate rate_;
    



    // roch_gpsr::ObjDetectConfig config_;

    double  voxel_leafsize_;
    double  pass_th_x_;
    bool    pass_th_y_;
    bool    pass_th_z_;
    double  pass_th_x_min_;
    double  pass_th_x_max_;
    double  pass_th_y_min_;
    double  pass_th_y_max_;
    double  pass_th_z_min_;
    double  pass_th_z_max_;
    bool    plane_seg_optimize_;
    double  plane_seg_threshold_;
    double  obj_extrac_tolerance_;
    double  obj_extrac_size_min_;
    double  obj_extrac_size_max_;



    // int sub_dep_img_cnt_;
    int pills_cnt_;

    vector< vector<float> > objs_pos_; //物体在map上的位置
    vector<int> objs_poll_; //投票数

    // CommonFunction* CommonFunction_;


public:
    ObjDetect();
    ~ObjDetect();
    void dyCB(roch_gpsr::ObjDetectConfig &config, uint32_t level);
    void sub_CB(const sensor_msgs::PointCloud2ConstPtr& dep_point_ptr);
};

//--------------------------------------------------
// 构造函数
//--------------------------------------------------
ObjDetect::ObjDetect():rate_(10)
{
    ROS_INFO("Detecting objects...");
    
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    f_ = boost::bind(&ObjDetect::dyCB, this, _1, _2);
    dynamic_server_.setCallback(f_);

    sub_dep_img_    = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &ObjDetect::sub_CB, this);

    pub_det_point_  = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/area", 1);         
    pub_det_plane_  = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/plane", 1);
    pub_det_objs_   = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/objs", 1);
    pub_down_point_ = nh_.advertise<sensor_msgs::PointCloud2>("/gpsr/obj_detect/down", 1);

    // CommonFunction_ = new CommonFunction(nh_);
    // CommonFunction_->commonf_actionf_speech_multi(nh_, "正在寻找物体");
    // CommonFunction_->commonf_actionf_cam_lift(nh_, iarm_obj_height_ - 0.28);
    // CommonFunction_->commonf_pubf_cam_tilt(nh_, 0.785);
    pills_cnt_ = 0;
    ros::Duration(0.5).sleep();
    
    voxel_leafsize_ = 0.01;
    pass_th_x_      = true;
    pass_th_y_      = true;
    pass_th_z_      = true;
    pass_th_x_min_  = -0.3;
    pass_th_x_max_  = 0.3;
    pass_th_y_min_  = -0.3;
    pass_th_y_max_  = 0.3;
    pass_th_z_min_  = 0.1;
    pass_th_z_max_  = 2.0;
    plane_seg_optimize_     = true;
    plane_seg_threshold_    = 0.01;
    obj_extrac_tolerance_   = 0.02;
    obj_extrac_size_min_    = 100;
    obj_extrac_size_max_    = 1000;

    ros::waitForShutdown();
}


//--------------------------------------------------
//析构函数
//--------------------------------------------------
ObjDetect::~ObjDetect(){
}


//--------------------------------------------------
// 点云订阅回调函数
//--------------------------------------------------
void ObjDetect::sub_CB(const sensor_msgs::PointCloud2ConstPtr& dep_point_ptr){ 
    ROS_INFO("finding  objects...");
    

    // ROS msg 转到 pcl data 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg (*dep_point_ptr, *i_pc);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(*i_pc));//指针指向pcl格式的点云

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objs_pc_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    ROS_INFO("Reconfigure Real are : %f, %s, %s, %s, %f, %f, %f, %f, %f, %f, %s, %f, %f, %f, %f.", 
        voxel_leafsize_,
        pass_th_x_?"True":"False",
        pass_th_y_?"True":"False",
        pass_th_z_?"True":"False",
        pass_th_x_min_,
        pass_th_x_max_,
        pass_th_y_min_,
        pass_th_y_max_,
        pass_th_z_min_,
        pass_th_z_max_,
        plane_seg_optimize_?"True":"False",
        plane_seg_threshold_,
        obj_extrac_tolerance_,
        obj_extrac_size_min_,
        obj_extrac_size_max_);

    //下采样
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(i_pc_ptr);
    // vg.setLeafSize(0.01, 0.01, 0.01); 
    vg.setLeafSize(voxel_leafsize_, voxel_leafsize_, voxel_leafsize_);
    vg.filter(*plane_pc_ptr);
    ROS_INFO("downing sample...");
    sensor_msgs::PointCloud2 down;
    pcl::toROSMsg(*plane_pc_ptr, down);
    down.header.frame_id = "camera_depth_optical_frame";
    pub_down_point_.publish(down);
    ROS_INFO("publishing the downing sample points...");

    //直通滤波
    pcl::PassThrough<pcl::PointXYZRGB> pass_th;
    if (pass_th_x_){
        ROS_INFO("pasing through x");
        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("x");
        // pass_th.setFilterLimits(-0.3, 0.3);
        pass_th.setFilterLimits(pass_th_x_min_, pass_th_x_max_);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);
        ROS_INFO("Passing through x ...");
    }
    if (pass_th_y_){
        ROS_INFO("pasing through y");
        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("y");
        pass_th.setFilterLimits(pass_th_y_min_, pass_th_y_max_);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);
        ROS_INFO("Passing through y ...");
    }
    if (pass_th_z_){
        ROS_INFO("pasing through z");
        pass_th.setInputCloud(plane_pc_ptr);
        pass_th.setFilterFieldName("z");
        pass_th.setFilterLimits(pass_th_z_min_, pass_th_z_max_);
        pass_th.setFilterLimitsNegative(false);
        pass_th.filter(*plane_pc_ptr);
        ROS_INFO("Passing through z ...");
    }
    //转成ROS msg以发布
    sensor_msgs::PointCloud2 det_area;
    pcl::toROSMsg(*plane_pc_ptr, det_area);
    det_area.header.frame_id = "camera_depth_optical_frame";
    pub_det_point_.publish(det_area);
    ROS_INFO("publishing the passthrough points...");

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
    sac_seg.setDistanceThreshold(plane_seg_threshold_); 
    sac_seg.setOptimizeCoefficients(plane_seg_optimize_); // true or false
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


    //聚类
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(objs_pc_ptr);
    std::vector<pcl::PointIndices> cluster_indices; //聚类索引

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec_ext;
    ec_ext.setClusterTolerance(obj_extrac_tolerance_); //分隔阈值[m]
    ec_ext.setMinClusterSize(obj_extrac_size_min_); 
    ec_ext.setMaxClusterSize(obj_extrac_size_max_); 
    ec_ext.setSearchMethod(kdtree); //搜索方法
    ec_ext.setInputCloud(objs_pc_ptr); //设置输入点云
    ec_ext.extract(cluster_indices); //提取索引

    //逐一提取点云(根据索引)
    for(std::vector<pcl::PointIndices>::const_iterator it1 = cluster_indices.begin(); it1 != cluster_indices.end(); ++it1)
    {
        //每个物体的最小、最大値
        double obj_pc_x_min = objs_pc_ptr->points[*it1->indices.begin()].x;
        double obj_pc_x_max = objs_pc_ptr->points[*it1->indices.begin()].x;
        double obj_pc_y_min = objs_pc_ptr->points[*it1->indices.begin()].y;
        double obj_pc_y_max = objs_pc_ptr->points[*it1->indices.begin()].y;
        double obj_pc_z_min = objs_pc_ptr->points[*it1->indices.begin()].z;
        double obj_pc_z_max = objs_pc_ptr->points[*it1->indices.begin()].z;
    
        for(std::vector<int>::const_iterator it2 = it1->indices.begin(); it2 != it1->indices.end(); it2++){
            if(obj_pc_x_min > objs_pc_ptr->points[*it2].x) obj_pc_x_min = objs_pc_ptr->points[*it2].x;
            if(obj_pc_x_max < objs_pc_ptr->points[*it2].x) obj_pc_x_max = objs_pc_ptr->points[*it2].x;
            if(obj_pc_y_min > objs_pc_ptr->points[*it2].y) obj_pc_y_min = objs_pc_ptr->points[*it2].y;
            if(obj_pc_y_max < objs_pc_ptr->points[*it2].y) obj_pc_y_max = objs_pc_ptr->points[*it2].y;
            if(obj_pc_z_min > objs_pc_ptr->points[*it2].z) obj_pc_z_min = objs_pc_ptr->points[*it2].z;
            if(obj_pc_z_max < objs_pc_ptr->points[*it2].z) obj_pc_z_max = objs_pc_ptr->points[*it2].z;            
        }

        //各物体的中心坐标(由三个维度中的点的中间值表示)
        double obj_pc_x_ave = (obj_pc_x_min + obj_pc_x_max) / 2.0;
        double obj_pc_y_ave = (obj_pc_y_min + obj_pc_y_max) / 2.0;
        double obj_pc_z_ave = (obj_pc_z_min + obj_pc_z_max) / 2.0;

        ROS_INFO("Got the position of object:%f, %f, %f", obj_pc_x_ave, obj_pc_y_ave, obj_pc_z_ave);

        // 获取tf关系
        tf::TransformListener tf_listener;
        tf::StampedTransform tf_stamped_transform;                            

        tf::TransformBroadcaster tf_broadcaster;
        tf::Transform tf_transform;
        tf_transform.setOrigin(tf::Vector3(obj_pc_x_ave , obj_pc_y_ave, obj_pc_z_ave));
        tf::Quaternion tf_quaternion;
        tf_quaternion.setRPY(0, 0, 0);
        tf_transform.setRotation(tf_quaternion);

        while(ros::ok()){
            tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "camera_depth_optical_frame", "obj"));
            try {
                tf_listener.lookupTransform("map", "obj", ros::Time(0), tf_stamped_transform);
            }
            catch(tf::TransformException){
                continue;
            }
            break;
        }

        //物体数量计算
        bool flag_poll = false; //Check if the voting was done
        for(size_t i = 0; i < objs_pos_.size(); i++){
            //中心坐标确认
            if(abs(objs_pos_[i][0] - tf_stamped_transform.getOrigin().x()) < 0.1 &&
                abs(objs_pos_[i][1] - tf_stamped_transform.getOrigin().y()) < 0.1 &&
                abs(objs_pos_[i][2] - tf_stamped_transform.getOrigin().z()) < 0.1){
                objs_poll_[i]++;
                flag_poll = true;

                //If you voted three times
                if(objs_poll_[i] == 3){
                    pills_cnt_++;
                    nh_.setParam("/param/pills/cnt", pills_cnt_);
                    ROS_INFO("There are %d pills", pills_cnt_);

                    int pills_target;
                    nh_.getParam("/param/pills/target", pills_target);
                    if(pills_cnt_ == pills_target){
                        // CommonFunction_->commonf_actionf_speech_single(nh_, "我发现了一个东西。");

                        double target_pos[3] = {0, 0, 0};
                        
                        for(size_t j = 0; j < objs_pos_.size(); j++){
                            if(objs_poll_[j] >= 3){
                                tf_transform.setOrigin(tf::Vector3(objs_pos_[j][0] , objs_pos_[j][1], objs_pos_[j][2]));

                                while(ros::ok()){
                                    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "map", "obj"));
                                    try {
                                        tf_listener.lookupTransform("base_link", "obj", ros::Time(0), tf_stamped_transform);
                                    }
                                    catch(tf::TransformException){
                                        continue;
                                    }
                                    break;
                                }

                                if((target_pos[0] == 0 and target_pos[1] == 0 and target_pos[2] == 0) or
                                    tf_stamped_transform.getOrigin().y() < target_pos[1]){
                                    target_pos[0] = tf_stamped_transform.getOrigin().x();
                                    target_pos[1] = tf_stamped_transform.getOrigin().y();
                                    target_pos[2] = tf_stamped_transform.getOrigin().z();
                                }
                            }
                        }

                        // nh_.setParam("/param/iarm/obj/pos/x", target_pos[0]);
                        // nh_.setParam("/param/iarm/obj/pos/y", target_pos[1]);
                        // nh_.setParam("/param/iarm/obj/pos/z", iarm_obj_height_ + 0.05); //tf_stamped_transform.getOrigin().z());

                        exit(0);
                    }
                }

                break;
            }
        }

        //When not voted, new voting
        if(flag_poll == false){
            vector<float> buf_pos(3);
            buf_pos[0] = tf_stamped_transform.getOrigin().x();
            buf_pos[1] = tf_stamped_transform.getOrigin().y();
            buf_pos[2] = tf_stamped_transform.getOrigin().z();
            objs_pos_.push_back(buf_pos);
            objs_poll_.push_back(1);
        }

    } // 
    // rate_.sleep();  
    // ros::Duration(0.1).sleep();
}


void ObjDetect::dyCB(roch_gpsr::ObjDetectConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %f, %s, %s, %s, %f, %f, %f, %f, %f, %f, %s, %f, %f, %f, %f.", 
            config.voxel_leafsize,
            config.pass_th_x?"True":"False",
            config.pass_th_y?"True":"False",
            config.pass_th_z?"True":"False",
            config.pass_th_x_min,
            config.pass_th_x_max,
            config.pass_th_y_min,
            config.pass_th_y_max,
            config.pass_th_z_min,
            config.pass_th_z_max,
            config.plane_seg_optimize?"True":"False",
            config.plane_seg_threshold,
            config.obj_extrac_tolerance,
            config.obj_extrac_size_min,
            config.obj_extrac_size_max);
    
    voxel_leafsize_ = config.voxel_leafsize;
    pass_th_x_      = config.pass_th_x;
    pass_th_y_      = config.pass_th_y;
    pass_th_z_      = config.pass_th_z;
    pass_th_x_min_  = config.pass_th_x_min;
    pass_th_x_max_  = config.pass_th_x_max;
    pass_th_y_min_  = config.pass_th_y_min;
    pass_th_y_max_  = config.pass_th_y_max;
    pass_th_z_min_  = config.pass_th_z_min;
    pass_th_z_max_  = config.pass_th_z_max;
    plane_seg_optimize_     = config.plane_seg_optimize;
    plane_seg_threshold_    = config.plane_seg_threshold;
    obj_extrac_tolerance_   = config.obj_extrac_tolerance;
    obj_extrac_size_min_    = config.obj_extrac_size_min;
    obj_extrac_size_max_    = config.obj_extrac_size_max;
}



//--------------------------------------------------
//主函数
//--------------------------------------------------
int main (int argc, char** argv){
    ros::init (argc, argv, "object_detect");

   

    ObjDetect ObjDetect;

    ros::spin();
    

    return 0;
}
