/*******************************************************************************************************************
*   
*  Assignment 2: Implementation of the divide-and-conquer go-to-position algorithm (goto1) 
*                and the MIMO go-to-position algorithm (goto2)
*
*   This is the implementation file.
*   For documentation, please see the application file
*
*   David Vernon
*   7 August 2020
*
*   Audit Trail
*   -----------
*
*******************************************************************************************************************/

#include <assignment2/studentid.h> 

//Loop rate 
geometry_msgs::Twist msg;
/* global variables with the current turtle pose */

extern float         current_x; 
extern float         current_y; 
extern float         current_theta;

/* Callback function, executed each time a new pose message arrives */

void poseMessageReceived(const turtlesim::Pose& msg) {
  bool debug = false;

   if (debug) {ROS_INFO_STREAM(std::setprecision(2) << std::fixed <<
	                       "position=(" << msg.x << "," << msg.y << ")" <<
		               " direction=" << msg.theta);
   }
   
   current_x     = msg.x;
   current_y     = msg.y;
   current_theta = msg.theta;
}

/**
 * 
 * 
 * */

void devidConquerLocomotion(float current_x, float current_y, float current_theta, float goal_x, float goal_y, const ros::Publisher pub){
         // Initialization of values
         float xr = current_x;
         float yr = current_y;
         float dx = goal_x - xr;
         float dy = goal_y - yr;
         float pos_err = sqrt((dx*dx)+(dy*dy));
         float head_err = atan2(dy,dx)-current_theta;
         ros::Rate loop_rate(5);


         do{
               // compute the current position of R
               xr = current_x;
               yr = current_y;
               //  compute the distsnce from R to Goal
               dx = goal_x - xr;
               dy = goal_y - yr;
               // Compute errors
               pos_err = sqrt((dx*dx)+(dy*dy));
               head_err = atan2(dy,dx) - current_theta;
               // correct the heading first
               if(abs(head_err) >0.14){
                  msg.linear.x = 0;
                  msg.angular.z = - abs(head_err * 0.4);
               }else{
                  msg.linear.x = abs(pos_err * 2);
                  msg.angular.z = 0;
               }
               // Publish the w and v velocity to the simulator
               pub.publish(msg);
               ros::spinOnce();
               loop_rate.sleep();

         }while(abs(pos_err) > 2);
}


/**
 * 
 * 
 * */

void mimoLocomotion(float current_x, float current_y, float current_theta, float goal_x, float goal_y, const ros::Publisher pub){

}

/*=======================================================*/
/* Utility functions                                     */ 
/*=======================================================*/


void display_error_and_exit(char error_message[]) {
   printf("%s\n", error_message);
   printf("Hit any key to continue >>");
   getchar();
   exit(1);
}

void prompt_and_exit(int status) {
   printf("Press any key to terminate the program ... \n");
   getchar();
   exit(status);
}

void prompt_and_continue() {
   printf("Press any key to continue ... \n");
   getchar();
}


void print_message_to_file(FILE *fp, char message[]) {
   fprintf(fp,"The message is: %s\n", message);
}

