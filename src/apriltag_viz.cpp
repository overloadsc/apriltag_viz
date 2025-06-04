// ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>  
#include <opencv2/calib3d.hpp>

class AprilVizNode : public rclcpp::Node {
public:
    AprilVizNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions())
    : Node("apriltag_viz", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        get_parameter_or<std::string>("overlay_mode", overlay_mode, "tri");
        get_parameter_or<double>("tag_size", tag_size, 0.161); // 默认标签尺寸为16.6厘米

        std::string image_transport;
        get_parameter_or<std::string>("image_transport", image_transport, "raw");

        pub_tags = image_transport::create_publisher(this, "tag_detections_image");

        sub_img = image_transport::create_subscription(this, "image_raw",
            std::bind(&AprilVizNode::onImage, this, std::placeholders::_1),
            image_transport);

        sub_tag = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "detections", rclcpp::QoS(1),
            std::bind(&AprilVizNode::onTags, this, std::placeholders::_1));

        // 初始化相机内参
        initCameraParameters();
    }

private:
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub_tags;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr sub_tag;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    cv::Mat img;
    cv::Mat merged;
    cv::Mat overlay;
    std::string overlay_mode;
    double tag_size; // AprilTag的物理尺寸(米)
    
    // 相机内参
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    cv::Mat rectification_matrix;
    cv::Mat projection_matrix;

    static const std::array<cv::Scalar, 4> colours;

    static std::array<double, 2> project(const std::array<double,9> H, // homography matrix
                                         const std::array<double,2> pc) // point in camera
    {
        std::array<double,2> pi;    // point in image
        const auto z = H[3*2+0] * pc[0] + H[3*2+1] * pc[1] + H[3*2+2];
        for(uint i(0); i<2; i++) {
            pi[i] = (H[3*i+0] * pc[0] + H[3*i+1] * pc[1] + H[3*i+2]) / z;
        }
        return pi;
    }

    // 初始化相机参数
    void initCameraParameters() {
        // 根据提供的相机内参数据初始化
        camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0, 0) = 499.835235; // fx
        camera_matrix.at<double>(0, 2) = 330.364862; // cx
        camera_matrix.at<double>(1, 1) = 500.632092; // fy
        camera_matrix.at<double>(1, 2) = 228.013527; // cy
        
        distortion_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        distortion_coeffs.at<double>(0, 0) = 0.044355; // k1
        distortion_coeffs.at<double>(0, 1) = -0.129307; // k2
        distortion_coeffs.at<double>(0, 2) = -0.004065;  // p1
        distortion_coeffs.at<double>(0, 3) = 0.006937;  // p2
        distortion_coeffs.at<double>(0, 4) = 0.000000;  // k3
        
        rectification_matrix = cv::Mat::eye(3, 3, CV_64F);
        
        projection_matrix = cv::Mat::zeros(3, 4, CV_64F);
        projection_matrix.at<double>(0, 0) = 496.396423;
        projection_matrix.at<double>(0, 2) = 335.015725;
        projection_matrix.at<double>(1, 1) = 501.993439;
        projection_matrix.at<double>(1, 2) = 225.839952;
        projection_matrix.at<double>(2, 2) = 1.0;
    }

    void publishTF(const std::string& parent_frame, const std::string& child_frame,
        const geometry_msgs::msg::Transform& transform) {
        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = parent_frame;
        transform_stamped.child_frame_id = child_frame;
        transform_stamped.transform = transform;

        tf_broadcaster_->sendTransform(transform_stamped);
    }

    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg_img) {
        img = cv_bridge::toCvCopy(msg_img)->image;

        if(overlay.empty()) {
            merged = img;
        }
        else {
            // blend overlay and image
            double alpha;
            get_parameter_or("alpha", alpha, 0.5);
            cv::addWeighted(img, 1, overlay, alpha, 0, merged, -1);
        }

        pub_tags.publish(cv_bridge::CvImage(msg_img->header, msg_img->encoding, merged).toImageMsg());
    }

    void onTags(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg_tag) {
        if(img.empty())
            return;

        // overlay with transparent background
        overlay = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0,0,0,0));

        for(const auto& d : msg_tag->detections) {
            if(overlay_mode=="axes") {
                // axes
                const auto c = project(d.homography, {{0,0}});
                const auto x = project(d.homography, {{1,0}});
                const auto y = project(d.homography, {{0,1}});
                cv::line(overlay, cv::Point2d(c[0], c[1]), cv::Point2d(x[0],x[1]), cv::Scalar(0,0,255,255), 3);
                cv::line(overlay, cv::Point2d(c[0], c[1]), cv::Point2d(y[0],y[1]), cv::Scalar(0,255,0,255), 3);
            }
            else if(overlay_mode=="tri") {
                // triangle patches
                std::array<cv::Point,3> points;
                points[0].x = d.centre.x;
                points[0].y = d.centre.y;

                for(uint i(0); i<4; i++) {
                    points[1].x = d.corners[i%4].x;
                    points[1].y = d.corners[i%4].y;
                    points[2].x = d.corners[(i+1)%4].x;
                    points[2].y = d.corners[(i+1)%4].y;

                    cv::fillConvexPoly(overlay, points.data(), 3, colours[i]);
                }
            }
            else {
                throw std::runtime_error("unknown overlay mode");
            }

            for(uint i(0); i<4; i++) {
                cv::circle(overlay, cv::Point(d.corners[i].x, d.corners[i].y), 5, colours[i], 2);
            }

            // 改进的位姿估计
            geometry_msgs::msg::Transform transform = estimatePose(d);
            
            // 发布变换
            publishTF("camera", "tag_" + std::to_string(d.id), transform);
            // 计算并打印两个坐标系之间的距离
            double distance = std::sqrt(
                std::pow(transform.translation.x, 2) +
                std::pow(transform.translation.y, 2) +
                std::pow(transform.translation.z, 2)
            );
            RCLCPP_INFO(this->get_logger(), "Distance between camera and tag_%d: %.3f meters", d.id, distance);
            RCLCPP_INFO(this->get_logger(), "x : %.3f y: %.3f z: %.3f", transform.translation.x, transform.translation.y, transform.translation.z);
        }
    }

    // 基于相机内参和AprilTag角点估计位姿
    geometry_msgs::msg::Transform estimatePose(const apriltag_msgs::msg::AprilTagDetection& detection) {
        // 定义AprilTag四个角点在其自身坐标系中的3D坐标(单位:米)
        // 假设AprilTag中心为原点，Z轴垂直于标签平面向外
        double half_size = tag_size / 2.0;
        std::vector<cv::Point3f> object_points;
        object_points.push_back(cv::Point3f(-half_size,  half_size, 0)); // 左上
        object_points.push_back(cv::Point3f( half_size,  half_size, 0)); // 右上
        object_points.push_back(cv::Point3f( half_size, -half_size, 0)); // 右下
        object_points.push_back(cv::Point3f(-half_size, -half_size, 0)); // 左下

        // 提取检测到的AprilTag四个角点的像素坐标
        std::vector<cv::Point2f> image_points;
        for (const auto& corner : detection.corners) {
            image_points.push_back(cv::Point2f(corner.x, corner.y));
        }

        // 求解位姿
        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, image_points, camera_matrix, distortion_coeffs, rvec, tvec);

        // 转换为四元数表示旋转
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        
        // 转换为TF变换
        geometry_msgs::msg::Transform transform;
        
        // 设置平移
        transform.translation.x = tvec.at<double>(0, 0);
        transform.translation.y = tvec.at<double>(1, 0);
        transform.translation.z = tvec.at<double>(2, 0);
        
        // 计算四元数 (Foxy版本兼容写法)
        tf2::Matrix3x3 tf_rotation;
        tf_rotation.setValue(
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)
        );
        
        tf2::Quaternion tf_quaternion;
        double roll, pitch, yaw;
        tf_rotation.getRPY(roll, pitch, yaw);
        tf_quaternion.setRPY(roll, pitch, yaw);
        
        // 设置旋转
        transform.rotation.x = tf_quaternion.x();
        transform.rotation.y = tf_quaternion.y();
        transform.rotation.z = tf_quaternion.z();
        transform.rotation.w = tf_quaternion.w();
        
        return transform;
    }
};


const std::array<cv::Scalar, 4> AprilVizNode::colours = {{
    cv::Scalar(0,0,255,255),    // red
    cv::Scalar(0,255,0,255),    // green
    cv::Scalar(255,0,0,255),    // blue
    cv::Scalar(0,255,255,255)   // yellow
}};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilVizNode)