
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>

#include <boost/program_options.hpp>

std::string m_target_frame;
std::string m_depth_frame_id;

void parse_command_line(int argc, char ** argv, std::string &m_target_frame, std::string &m_depth_frame_id)
{
  std::string target_frame;
  std::string depth_frame_id;
  boost::program_options::options_description desc("Configuration");
  desc.add_options()
    ("help", "show this help message")
    ("target_frame", boost::program_options::value<std::string>(&target_frame)->default_value(m_target_frame),
     "target_frame")
    ("depth_frame_id", boost::program_options::value<std::string>(&depth_frame_id)->default_value(m_depth_frame_id),
     "depth_frame_id")
    ;
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);
  m_target_frame = vm["target_frame"].as<std::string>();
  m_depth_frame_id = vm["depth_frame_id"].as<std::string>();
  ROS_DEBUG_STREAM("target_frame is " << m_target_frame);
  ROS_DEBUG_STREAM("depth_frame_id is " << m_depth_frame_id);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return ;
  }
}

class ObjectFilter {
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;
    ros::Publisher object_moveit_pub_, pub_obj_pose, pub_obj_poses;
    tf::TransformListener listener_;
    std::string object_topic_;
    geometry_msgs::PoseStamped msg_obj_cam_, msg_obj_pose;
    geometry_msgs::PoseArray msg_obj_poses;
    object_recognition_msgs::RecognizedObjectArray::_objects_type::const_iterator it;
    shape_msgs::SolidPrimitive msg_sphere_, msg_box_, msg_cylinder_;

    void obj_fake(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
    {
      msg_obj_poses.header.frame_id = m_target_frame;
      msg_obj_poses.poses.clear();

      moveit_msgs::CollisionObject msg_obj_moveit;
      msg_obj_moveit.header.frame_id = m_target_frame;
      //msg_obj_moveit.operation = moveit_msgs::CollisionObject::ADD;
      msg_obj_moveit.primitives.push_back(msg_cylinder_); //msg_box_); //
      //msg_obj_moveit.primitives.push_back(msg_box_);

      //for the left hand
      //- Rotation best [-0.707, 0.000, -0.000, 0.707],
      msg_obj_pose.pose.orientation.x = -1;//-0.707;//-1;
      msg_obj_pose.pose.orientation.y = 0.0;
      msg_obj_pose.pose.orientation.z = 0.0;
      msg_obj_pose.pose.orientation.w = 0.000000253; //0.0;//0.707; //0.0;
      msg_obj_pose.header.frame_id = m_target_frame;

      //for the left hand romeo
      //position best: 0.47; 0.21;-0.11;
      msg_obj_pose.pose.position.x = 0.5;//0.47; //0.5;//
      msg_obj_pose.pose.position.y = 0.2;//0.2
      msg_obj_pose.pose.position.z = -0.13;//-0.11; //-0.13;//
      msg_obj_moveit.primitive_poses.clear();
      msg_obj_moveit.primitive_poses.push_back(msg_obj_pose.pose);
      msg_obj_moveit.type.key = "0_l_romeo";
      msg_obj_moveit.id = "0_l_romeo";
      object_moveit_pub_.publish(msg_obj_moveit);
      pub_obj_pose.publish(msg_obj_pose);
      msg_obj_poses.poses.push_back(msg_obj_pose.pose);
      std::cout << msg_obj_moveit.id << std::endl << msg_obj_pose.pose << std::endl;

      //---------------------right
      //0.47; -0.18; -0.11; //best
      msg_obj_pose.pose.position.y = -msg_obj_pose.pose.position.y;
      msg_obj_moveit.primitive_poses.clear();
      msg_obj_moveit.primitive_poses.push_back(msg_obj_pose.pose);
      msg_obj_moveit.type.key = "1_r_romeo";
      msg_obj_moveit.id = "1_r_romeo";
      object_moveit_pub_.publish(msg_obj_moveit);
      pub_obj_pose.publish(msg_obj_pose);
      msg_obj_poses.poses.push_back(msg_obj_pose.pose);
      pub_obj_poses.publish(msg_obj_poses);
      std::cout << msg_obj_moveit.id << std::endl << msg_obj_pose.pose << std::endl;
    }

    void obj_cb(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
    {
      try {
        int obj_id = 0;
        for (it = msg->objects.begin(); it != msg->objects.end(); ++it) {
          if (it->confidence > 0.80) {
            tf::StampedTransform transform;
            listener_.waitForTransform(m_target_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(3.0));
            //listener_.lookupTransform(m_target_frame, msg->header.frame_id, msg->header.stamp, transform);
            msg_obj_cam_.header = msg->header;
            msg_obj_cam_.pose = it->pose.pose.pose;
            listener_.transformPose(m_target_frame, msg->header.stamp, msg_obj_cam_, m_depth_frame_id, msg_obj_pose);
            //ROS_INFO_STREAM("object " << it->type.key);

            //object_recognition_msgs::RecognizedObject msg_obj_ork(*it);
            //ROS_INFO_STREAM(msg_obj_pose.pose);
            //ROS_INFO_STREAM("frames: " << m_target_frame << " " << m_depth_frame_id << " " << msg->header.frame_id ); // << " " << msg_obj_base_.pose);

            moveit_msgs::CollisionObject msg_obj_moveit;
            msg_obj_moveit.header = msg->header;
            msg_obj_moveit.header.frame_id = m_target_frame;
            msg_obj_moveit.operation = moveit_msgs::CollisionObject::ADD;
            msg_obj_moveit.primitive_poses.push_back(msg_obj_pose.pose);
            msg_obj_moveit.primitives.push_back(msg_sphere_);
            msg_obj_moveit.type = it->type;
            std::stringstream ss;
            ss << obj_id; //<< "object"
            msg_obj_moveit.type.key = ss.str();
            ss << "_" << it->type.key;
            msg_obj_moveit.id = ss.str();
            ++obj_id;

            object_moveit_pub_.publish(msg_obj_moveit);

            /*msg_obj_moveit.mesh_poses.push_back(msg_obj_pose.pose);
            msg_obj_moveit.meshes.push_back(it->bounding_mesh);*/
          }
        }
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    public:
        ObjectFilter() : nh_("~") {
            nh_.param("object_topic", object_topic_, std::string("/recognized_object_array"));
            object_sub_ = nh_.subscribe<object_recognition_msgs::RecognizedObjectArray>(object_topic_, 10, &ObjectFilter::obj_fake, this);
            object_moveit_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 1000);
            pub_obj_pose = nh_.advertise<geometry_msgs::PoseStamped>("/obj_pose", 1000);
            pub_obj_poses = nh_.advertise<geometry_msgs::PoseArray>("/obj_poses", 100);
            init();
        }

        void init()
        {
          static const double BLOCK_SIZE = 0.03; //0.03; //0.03;
          msg_sphere_.type = shape_msgs::SolidPrimitive::SPHERE;
          msg_sphere_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
          msg_sphere_.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = BLOCK_SIZE; //0.015;

          msg_cylinder_.type = shape_msgs::SolidPrimitive::CYLINDER;
          msg_cylinder_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
          msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = BLOCK_SIZE;
          msg_cylinder_.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = BLOCK_SIZE*3.0;

          msg_box_.type = shape_msgs::SolidPrimitive::BOX;
          msg_box_.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
          msg_box_.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.015;
          msg_box_.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.015;
          msg_box_.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.04;
        }
};

int main(int argc, char **argv) {
  m_target_frame = "base_link";
  m_depth_frame_id = "CameraDepth_frame";
  parse_command_line(argc, argv, m_target_frame, m_depth_frame_id);
  ROS_INFO_STREAM("Object frame: " << m_target_frame << " " << m_depth_frame_id);

  ros::init(argc,argv,"object_filter");
  ObjectFilter fm;
  fm.init();
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
