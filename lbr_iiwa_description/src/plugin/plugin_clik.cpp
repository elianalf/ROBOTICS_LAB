#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include "nav_msgs/Path.h"
#include "boost/thread.hpp"


using namespace std;

/*The client choose the trajecty and this plugin take and perform it*/

namespace gazebo
{
   class CLIK_traj : public ModelPlugin {
  
    private:
          ros::NodeHandle* nh;
	       physics::ModelPtr model;
	       
	       ros::Subscriber traj_sub;
	       ros::Publisher *cmd_pub; 
	       ros::Publisher cartpose_pub;
	       
          event::ConnectionPtr updateConnection;
          
          int joint_num;
          KDL::Tree iiwa_tree;
          KDL::Chain k_chain;
          
          KDL::ChainFkSolverPos_recursive *fksolver;
          KDL::ChainIkSolverVel_pinv *ik_solver_vel;   	//Inverse velocity solver
          KDL::ChainIkSolverPos_NR *ik_solver_pos;
          
          KDL::JntArray *q_in;
          KDL::JntArray *q_out;
          KDL::Frame des_frame;
          KDL::Frame p_out;
          physics::JointPtr joint_handler[7]; //we need it to get directly the joint  position from gazebo model
          
          bool initIK();
          bool new_traj;
          nav_msgs::PathPtr path;
	       
	  public:
	        //Free memory
	        ~CLIK_traj();       
	        
	        //Load the plugin: The Load FUNCTION can receive the SDF element. Function load is called when the plugin is loaded in simulation
	        void Load(physics::ModelPtr parent, sdf::ElementPtr _sdf);
	        
	        // Called by the world update start event
	        void OnUpdate();
	       
	       //Retrive desired position
	        void traj_cb(nav_msgs::PathPtr nav_path);
	        void perform_traj();
	
   };
    
    // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CLIK_traj)
  
  
  CLIK_traj::~CLIK_traj(){
    delete[] cmd_pub;
    delete q_in;
    delete ik_solver_pos;
    delete ik_solver_vel;
    delete nh;  
   }
   
   
   void CLIK_traj::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf){
        //setup plugin
        nh = new ros::NodeHandle();
        model=parent;
        
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CLIK_traj::OnUpdate, this));
        //setup inverse kinamatics solver
        if(!initIK())
            exit(1);
        
        ROS_INFO("robot tree correctly loaded!");
        
        cmd_pub = new ros::Publisher[joint_num];
        
        cmd_pub[0] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint1_position_controller/command", 0);
        cmd_pub[1] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint2_position_controller/command", 0);
        cmd_pub[2] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint3_position_controller/command", 0);
        cmd_pub[3] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint4_position_controller/command", 0);
        cmd_pub[4] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint5_position_controller/command", 0);
        cmd_pub[5] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint6_position_controller/command", 0);
        cmd_pub[6] = nh->advertise< std_msgs::Float64 > ("/lbr_iiwa/joint7_position_controller/command", 0);
        
        cartpose_pub = nh->advertise<geometry_msgs::Pose>("/lbr_iiwa/eef_pose",0);
        
        traj_sub = nh->subscribe("/nav/path",0 , &CLIK_traj::traj_cb, this);
        
        for(int i=0;i<7; i++){
            joint_handler[i]= this->model->GetJoint("lbr_iiwa_joint" + std::to_string(i+1) ); //initialize joint_handlers with the name of joint in the gazebo model to directly get the joints position from gazebo model 
        }
        new_traj=false;
   }
   
   
   
   bool CLIK_traj::initIK(){
        std::string robot_desc_string;
        
        nh->param("robot_description", robot_desc_string, std::string());
        if(!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
            ROS_ERROR("FAILED to construct kdl tree ");
            return false;
        }
        if(!iiwa_tree.getChain("lbr_iiwa_link_0", "lbr_iiwa_link_7", k_chain)){
        ROS_ERROR("fAILED TO CONSTRUCT KDL CHAIN");
        return false;
        }
        
        fksolver = new KDL::ChainFkSolverPos_recursive(k_chain);
        ik_solver_vel =new KDL::ChainIkSolverVel_pinv(k_chain);
        ik_solver_pos =new KDL::ChainIkSolverPos_NR(k_chain, *fksolver, *ik_solver_vel,100, 1e-6);
        
        joint_num = k_chain.getNrOfJoints();
        q_in = new KDL::JntArray(joint_num);
        q_out = new KDL::JntArray(joint_num);
        
        return true;
   
   }
   
   void CLIK_traj::traj_cb(nav_msgs::PathPtr t){
    path =t;
    new_traj=true;
   }
   
    
   void CLIK_traj::perform_traj(){
        KDL::JntArray *q;
        q=new KDL::JntArray(joint_num);
        q = q_in;
        std_msgs::Float64 cmd;
        std::vector< vector< float > > q_cmd_stack; //it's a matrix: each column is a i-th joint and each row is a configuration in a different istant 
        vector< float > q_cmd;
        q_cmd.resize(7);
        ros::Rate r(300);
        
        KDL::JntArray q_out(7);
        for(int i=0; i<path->poses.size();i++){
            des_frame.p = KDL::Vector(path->poses[i].pose.position.x,path->poses[i].pose.position.y,path->poses[i].pose.position.z);
            des_frame.M = p_out.M;
            int result = ik_solver_pos->CartToJnt(*q, des_frame, *q);//(current q , desired point to reach, q output in the same variable of the previous q)
            for(int i=0; i<7;i++){
                q_cmd[i] = q->data[i];
            }
            q_cmd_stack.push_back(q_cmd);
         }  
            
        cout<< "Stack size: " <<q_cmd_stack.size() <<endl;
         
        for(int i=0; i<q_cmd_stack.size();i++){ 
            for(int j; j<7;j++){
                cmd.data = q_cmd_stack[i][j];
                cmd_pub[j].publish (cmd); //REMEMBER that why we have to use cmd topic if we are inside gazebo? because we are using the ROS controller (that are an implementation of the controllers in ROS not in Gazebo) so we have to use ROS topic command. In gazebo I can send directly the position but it's something static, here I want to use the controllers
            }
            cout<<"Stream: "<< i<<endl;
            r.sleep();
        }
        
        
    }   
    
     
    void CLIK_traj::OnUpdate(){
        for(int i=0; i<7; i++){
            q_in->data[i]=joint_handler[i]->Position(0);
        }
        fksolver->JntToCart(*q_in, p_out);
        
        geometry_msgs::Pose cpose;
        
        cpose.position.x = p_out.p.x();
        cpose.position.y = p_out.p.y();
        cpose.position.z = p_out.p.z();
        
        double qx,qy,qz,qw;
        p_out.M.GetQuaternion(qx, qy, qz, qw);
        cpose.orientation.w = qw;
        cpose.orientation.x = qx;
        cpose.orientation.y = qy;
        cpose.orientation.z = qz;
        
        cartpose_pub.publish(cpose);
        
        if(new_traj){  
            boost::thread perform_traj_t(&CLIK_traj::perform_traj,this);
            new_traj=false;
        }
    
    
    
    
    }
   
}
