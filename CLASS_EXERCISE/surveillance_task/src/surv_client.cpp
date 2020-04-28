#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/client/simple_action_client.h>
#include "surveillance_task/navAction.h"

using namespace std;

#define LIN_VEL 0.3
#define ANG_VEL 0.5


class SURV_CLIENT {
    public:
        SURV_CLIENT();
        void laser_cb( sensor_msgs::LaserScanConstPtr );
        void odom_cb( nav_msgs::OdometryConstPtr );
        void run();
			void main_loop();
			int teaching_cb(int);
				
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _laser_sub;
        ros::Publisher vel_pub;
        bool obstacles;	
        geometry_msgs::Point odom_pos;
        std::vector< geometry_msgs::Point > point_list;
        
};


SURV_CLIENT::SURV_CLIENT() {

   //_human_cb = _nh.subscribe("/cmd_vel/key", 0, &SURV_CLIENT::keyabord_cb,this);
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV_CLIENT::laser_cb,this);
    _odom_sub = _nh.subscribe("/odom", 0, &SURV_CLIENT::odom_cb, this);
   obstacles=false;
   vel_pub = _nh.advertise< geometry_msgs::Twist >("/cmd_vel", 0);  //decleare the topic 
}




int SURV_CLIENT::teaching_cb( int index ) {

	string input;
   bool teaching_done = false;
   int first=0;
   geometry_msgs::Twist cmd_vel;
   float _fv; //Forward velocity	
	float _rv; //Rotational velocity
	
	std::vector<geometry_msgs::Point >::iterator itp;
	
	//activate keyboard mode
	cout << "Keyboard Input: " << endl;
	cout << "[w]: Forward direction velocity" << endl;
	cout << "[x]: Backward direction velocity" << endl;
	cout << "[a]: Left angular velocity" << endl;
	cout << "[d]: Right angular velocity" << endl;
	cout << "[s]: Stop the robot and save position" << endl;
   cout << "[f]: Teaching completed!" << endl;
   
   ros::Rate r(10);
   while(!teaching_done){
        
		getline( cin, input);

		if( input == "w" ) 
			_fv = (_fv < 0.0 ) ? 0.0 : LIN_VEL;
		else if( input == "x" ) 
			_fv = (_fv > 0.0 ) ? 0.0 : -LIN_VEL;
		else if( input == "a" ) 
			_rv = (_rv > 0.0 ) ? 0.0 : -ANG_VEL;
		else if( input == "d" )
			_rv = (_rv < 0.0 ) ? 0.0 : ANG_VEL;
		else if( input == "s" ){
		      _fv = _rv = 0.0;
		      itp = point_list.begin() + index ;
            point_list.insert(itp,1, odom_pos);
             //cout<<"added new point"<<endl;
            index++;
		}
      else if(input == "f"){
            teaching_done = true;
       }  
    
     cmd_vel.linear.x = _fv;
	  cmd_vel.angular.z = _rv;
	  vel_pub.publish( cmd_vel );
	  
	  r.sleep();
   }
   
   index = index-1;
   return index;
}



void SURV_CLIENT::laser_cb( sensor_msgs::LaserScanConstPtr laser_data) {
   //  cout << "Laser!" << endl;   
    float threshold = 0.7;
    int first_index = int( (( 90-20 )/180.0*M_PI) / laser_data->angle_increment   );
    int last_index  = int( (( 90+20 )/180.0*M_PI) / laser_data->angle_increment   );
	//cout << laser_data->ranges[last_index] << endl;
	int i = first_index;
	while(i <= last_index && !obstacles){
	      
         obstacles = (laser_data->ranges[i]<=threshold);  //if () is true, set obstacles = true 
         i++;
   }
   
   
}
 
 
void SURV_CLIENT::odom_cb( nav_msgs::OdometryConstPtr odom_ptr) {
    //cout << "Odom!" << endl;
    odom_pos.x = odom_ptr->pose.pose.position.x;
    odom_pos.y = odom_ptr->pose.pose.position.y;
}
 
 
//main loop! 
void SURV_CLIENT::main_loop() {

  actionlib::SimpleActionClient<surveillance_task::navAction> ac("auto_nav_server", true);
  ac.waitForServer(); //will wait for infinite time
  cout << "Connected with server" << endl;
  surveillance_task::navGoal g;
  
  point_list.resize(4);
  
  point_list[0].x = 6.0;
  point_list[0].y = 0.0;
  
  point_list[1].x = 4.0;
  point_list[1].y = 5.0; 
  
  point_list[2].x = -2.0;
  point_list[2].y = 2.0;
  
  point_list[3].x = 0.0;
  point_list[3].y = 0.0;

   while(ros::ok()){
	   cout<<"ros is ok"<< endl;
	   surveillance_task::navGoal g;

	   ros::Rate r(10);
	   bool done = false;
	   
	   
	   //motion logic
      for(int i=0; i<point_list.size(); i++)
      {     done = false;
            g.x_dest = point_list[i].x;
            g.y_dest = point_list[i].y;
            ac.sendGoal(g);
            cout << "Send " <<g.x_dest<< " " << g.y_dest<<endl;

	          while ( !done && !obstacles) {
	                r.sleep();
		            if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
		                cout << "Target position reached!" << endl;
			            done = true;
			           r.sleep();
		             }
             }
             	//sent to the robot the human generated velocity						
					//until human operator terminates with an additional input            
	          if( obstacles ) {
                        //cout << "Send cancel goal" << endl;
								ac.cancelGoal();		
								cout << "Keyboard_teleoperation mode on" << endl;
								i=teaching_cb(i);
								cout << "Teaching done" << endl;
								obstacles = false;
								               					
               }
	       
        }
         
       
   }
}

 
 void SURV_CLIENT::run() {
   	boost::thread ctrl_loop_t( &SURV_CLIENT::main_loop, this );
    ros::spin();
}



int main( int argc, char** argv) {

    ros::init(argc, argv, "surv_task_client" );
    SURV_CLIENT s;
    s.run();
    
    return 0;

}
