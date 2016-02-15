
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit_msgs/CollisionObject.h>

#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>

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
  ROS_INFO_STREAM("target_frame is " << m_target_frame);
  ROS_INFO_STREAM("depth_frame_id is " << m_depth_frame_id);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return ;
  }
}

class ObjectFilter {
  protected:
    ros::NodeHandle nh_;
    ros::Subscriber object_sub_;
    ros::Publisher object_moveit_pub_;
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

      moveit_msgs::CollisionObject msg_obj_collision;
      msg_obj_collision.header.stamp = ros::Time::now();
      msg_obj_collision.header.frame_id = m_target_frame;
      msg_obj_collision.operation = moveit_msgs::CollisionObject::ADD;
      msg_obj_collision.primitives.push_back(msg_cylinder_); //msg_box_); //

      //for the left hand
      //- Rotation best [-0.707, 0.000, -0.000, 0.707],
      msg_obj_pose.header.frame_id = m_target_frame;
      msg_obj_pose.pose.orientation.x = -1;//-0.707;//-1;
      msg_obj_pose.pose.orientation.y = 0.0;
      msg_obj_pose.pose.orientation.z = 0.0;
      msg_obj_pose.pose.orientation.w = 0.000000253; //0.0;//0.707; //0.0;

      //position best: 0.47; 0.21;-0.11;
      msg_obj_pose.pose.position.x = 0.5;//0.47; //0.5;//
      msg_obj_pose.pose.position.y = 0.2;//0.2
      msg_obj_pose.pose.position.z = -0.13;//-0.11; //-0.13;//

      /*msg_obj_pose.pose.position.x = 0.419093;
      msg_obj_pose.pose.position.y = 0.2;
      msg_obj_pose.pose.position.z = -0.0640153;
      msg_obj_pose.pose.orientation.x = 0.000000199;
      msg_obj_pose.pose.orientation.y = 0.195092;
      msg_obj_pose.pose.orientation.z = -0.000000199;
      msg_obj_pose.pose.orientation.w = 0.980785;*/

      msg_obj_collision.primitive_poses.clear();
      msg_obj_collision.primitive_poses.push_back(msg_obj_pose.pose);
      msg_obj_collision.type.key = "1";
      msg_obj_collision.id = "block1";
      object_moveit_pub_.publish(msg_obj_collision);
      msg_obj_poses.poses.push_back(msg_obj_pose.pose);

      //---------------------right
      //0.47; -0.18; -0.11; //best
      msg_obj_pose.pose.position.y = -msg_obj_pose.pose.position.y;
      msg_obj_collision.primitive_poses.clear();
      msg_obj_collision.primitive_poses.push_back(msg_obj_pose.pose);
      msg_obj_collision.type.key = "2";
      msg_obj_collision.id = "block2";
      object_moveit_pub_.publish(msg_obj_collision);
      msg_obj_poses.poses.push_back(msg_obj_pose.pose);
    }

    bool getMeshFromDB(object_recognition_msgs::GetObjectInformation &obj_info)
    {
      //! Client for getting the mesh for a database object
      ros::ServiceClient get_model_mesh_srv_;
      std::string get_model_mesh_srv_name("get_object_info");
      ros::Time start_time = ros::Time::now();
      while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) )
      {
        ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
        if (!nh_.ok() || ros::Time::now() - start_time >= ros::Duration(5.0))
          return false;
      }

      get_model_mesh_srv_ = nh_.serviceClient<object_recognition_msgs::GetObjectInformation>
        (get_model_mesh_srv_name, false);

      if ( !get_model_mesh_srv_.call(obj_info) )
      {
        ROS_ERROR("Get model mesh service service call failed altogether");
        return false;
      }
      return true;
    }

    void obj_real(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
    {
      try {
        int obj_id = 0;
        for (it = msg->objects.begin(); it != msg->objects.end(); ++it)
        {
            msg_obj_cam_.header = msg->header;
            msg_obj_cam_.pose = it->pose.pose.pose;

            ros::Time t = ros::Time(0);
            listener_.waitForTransform(m_target_frame, msg->header.frame_id, t, ros::Duration(3.0));
            listener_.transformPose(m_target_frame, t, msg_obj_cam_, msg->header.frame_id, msg_obj_pose);

            moveit_msgs::CollisionObject msg_obj_collision;
            msg_obj_collision.header = msg->header;
            msg_obj_collision.header.frame_id = m_target_frame;

            object_recognition_msgs::GetObjectInformation obj_info;
            obj_info.request.type = it->type;

            if (getMeshFromDB(obj_info))
            {
              msg_obj_collision.meshes.push_back(obj_info.response.information.ground_truth_mesh);
              msg_obj_collision.mesh_poses.push_back(msg_obj_pose.pose);
            }
            else
            {
              msg_obj_collision.primitive_poses.push_back(msg_obj_pose.pose);
              msg_obj_collision.primitives.push_back(msg_cylinder_);
            }

            msg_obj_collision.operation = moveit_msgs::CollisionObject::ADD;

            msg_obj_collision.type = it->type;
            std::stringstream ss;
            ss << obj_id; //<< "object"
            //msg_obj_collision.type.key = ss.str();
            ss << "_" << it->type.key;
            msg_obj_collision.id = ss.str();
            ++obj_id;

            object_moveit_pub_.publish(msg_obj_collision);
        }
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    public:
        ObjectFilter(bool virt) :
          nh_("")
        {
            nh_.param("object_topic", object_topic_, std::string("/recognized_object_array"));
            object_sub_ = nh_.subscribe<object_recognition_msgs::RecognizedObjectArray>(object_topic_, 10, virt?&ObjectFilter::obj_fake:&ObjectFilter::obj_real, this);
            object_moveit_pub_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
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
  bool virt = false; //true;
  m_target_frame = "base_link";
  m_depth_frame_id = "CameraDepth_frame";
  parse_command_line(argc, argv, m_target_frame, m_depth_frame_id);

  ros::init(argc,argv,"orkobj_tomoveit");
  ObjectFilter *fm = new ObjectFilter(virt);
  fm->init();

  /*ros::AsyncSpinner spinner(1);
  spinner.start();*/

  ros::Rate rate(10);
  while(ros::ok())
    ros::spinOnce();

  return 0;
}
