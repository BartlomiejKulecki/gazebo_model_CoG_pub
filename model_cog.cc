#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace gazebo
{
  class ModelCoG : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelCoG::OnUpdate, this));

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;

        ros::init(argc, argv, "model_cog_node",
            ros::init_options::NoSigintHandler);
      }

      // Create ROS node.
      this->rosNode.reset(new ros::NodeHandle( "model_cog_node" ));
      this->rosPub = this->rosNode->advertise<geometry_msgs::PoseStamped>(this->model->GetName()+"_pose",1);
    }

    // Called by the world update start event
    public: void OnUpdate()
    {  
      this->cog = this->model->GetLink("link")->WorldCoGPose();
      //std::cout << this->cog << std::endl;
      this->pose.header.stamp = ros::Time::now();
	this->pose.header.frame_id = "world";
	this->pose.pose.position.x = this->cog.Pos().X();
	this->pose.pose.position.y = this->cog.Pos().Y();
	this->pose.pose.position.z = this->cog.Pos().Z();
	this->pose.pose.orientation.x = this->cog.Rot().X();
	this->pose.pose.orientation.y = this->cog.Rot().Y();
	this->pose.pose.orientation.z = this->cog.Rot().Z();
	this->pose.pose.orientation.w = this->cog.Rot().W();
	this->rosPub.publish(this->pose);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Publisher rosPub;
    private: ignition::math::Pose3d cog;
    private: geometry_msgs::PoseStamped pose;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelCoG)
}
