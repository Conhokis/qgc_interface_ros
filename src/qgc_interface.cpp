#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <stdint.h>
#include <iostream>
#include <qgc_interface/Errors.h>

//my_libs
#include "commUtils.h"
#include "GoalToError.h"

//tf2
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>


#include <cmath>
enum states {
	INIT_GPS,
	INIT_MAVLINK,
	OK_STATE,
	ERROR_STATE,
} state;

enum events {
	NONE,
	GPS_STARTED,
	QGC_STARTED,
	GPS_ERROR,
	QGC_ERROR,
	ERROR_RECOVERED,
} event;

double starting_latitude = 0, starting_longitude = 0, starting_altitude = 0, starting_yaw = 0;

void navSatFixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	//Codigo para ver o estado vai aqui
	static int8_t nav_count = 20;
	static bool fixed = false;
    switch(state) {
    	case INIT_GPS:
    	if(msg->status.status == 0 && !fixed) {
    		if(nav_count == 0) { 
    			event = GPS_STARTED;
    			starting_latitude = starting_latitude / 20;
    			starting_longitude = starting_longitude / 20;
    			starting_altitude = starting_altitude / 20;
    			fixed = true;
    		}
    		else {
    			starting_latitude += msg->latitude;
    			starting_longitude += msg->longitude;
    			nav_count--;
    		}
    	}
    	break;

    	case OK_STATE:
    		starting_latitude = msg->latitude;
    		starting_longitude = msg->longitude;
    	break;
    }
}

void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	static bool done = false;
	if(done) return;

	geometry_msgs::Quaternion quat = msg->orientation;
	double siny_cosp = +2.0 * (quat.w * quat.z + quat.x * quat.y);
 	double cosy_cosp = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
	starting_yaw = atan2(siny_cosp, cosy_cosp); 
	done = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qgc_node");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(100);

	//TODO: Criar o subscriber para 

	missions mission;
	commUtils commonUtils(&mission);
	GoalToError* goalToError;

	event = NONE;
	state = INIT_GPS;
	ros::Subscriber sub_nav = n.subscribe("/fix", 10, navSatFixCallback);
	ros::Subscriber sub_fix = n.subscribe("/imu_bosch/data", 10, ImuCallback);
	ros::Publisher pub_errors = n.advertise<qgc_interface::Errors>("/errors", 10);

	std::cout << "Trying to connect to GPS..." << std::endl;
	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);
  	geometry_msgs::TransformStamped transformStamped;
  	Waypoint current_waypoint;
	bool qgcIsConnected;
	qgc_interface::Errors errors_msg;
	errors_msg.distance_error = 0;
	errors_msg.angle_error = 0;
	while(ros::ok()) {
		ros::spinOnce();
		//Main

		try {
  			transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
 		}

		catch (tf2::TransformException &ex) {
   			//ROS_WARN("%s",ex.what());
   			continue;
 		}

		qgcIsConnected = commonUtils.getConnection();
		//Acrescentar aqui talvez a verificação do GPS
		switch(state) {
			case INIT_GPS:
				switch(event) {
					case GPS_STARTED:
						state = INIT_MAVLINK;
						commonUtils.init();
						goalToError = new GoalToError(starting_latitude, starting_longitude, starting_altitude, starting_yaw);
						std::cout << std::setprecision(15) << "Initial position set: " << starting_latitude << " " << starting_longitude << std::endl;
						std::cout << "starting_yaw : " << (starting_yaw * 180 / M_PI) << std::endl;
					break;
					//No default here
				}
			break;

			case INIT_MAVLINK:
				if(qgcIsConnected) event = QGC_STARTED;

				switch(event) {
					case QGC_STARTED:
						state = OK_STATE;
						std::cout << "QGroundControl connected" << std::endl;
					break;
					//No default here
				}
			break;

			case OK_STATE:
				if(!qgcIsConnected) event = QGC_ERROR;	//Fazer o outro if do gps depois

				switch(event) {
					case QGC_ERROR:
						state = ERROR_STATE;
						std::cout << "[ERROR] QGroundControl disconnected" << std::endl;
					break;

					case GPS_ERROR:
						state = ERROR_STATE;
						std::cout << "[ERROR] GPS disconnected" << std::endl;
					break;

					default:
						//Codigo a fazer default na main
						goalToError->updatePosition(transformStamped);
						commonUtils.setLatLonYaw(goalToError->getCurrentLatitude(), goalToError->getCurrentLongitude(), goalToError->getCurrentYaw());
						//commonUtils.setLatLon(starting_latitude, starting_longitude);
						if(mission.hasWaypoints()) {
							current_waypoint = mission.getNextWaypoint();
							goalToError->updateErrors(current_waypoint.lat, current_waypoint.lon, transformStamped);
							errors_msg.distance_error = goalToError->getDistanceError();
							errors_msg.angle_error = goalToError->getAngleError();

							if(goalToError->getDistanceError() < current_waypoint.acceptanceRadius) {
								commonUtils.waypoint_cleared(current_waypoint.seq);
								mission.clearWaypoint(current_waypoint.seq);
								std::cout << "Mission Completed!" << std::endl;
								std::cout << mission.hasWaypoints() << std::endl;
								errors_msg.distance_error = 0;
								errors_msg.angle_error = 0;
							}
						}

						else { //NO MISSION
							//std::cout << "NO MISSION" << std::endl;
						}
						
					break;
				}
			break;

			case ERROR_STATE:
				//if(qgcIsConnected && ...) and navstatus whatever happens
				//	event = ERROR_RECOVERED;
				switch(event) {
					case ERROR_RECOVERED:
						state = OK_STATE;
						std::cout << "Recovered connection! Resuming mission." << std::endl;
					break;

					default:
						//Enviar erro para a missão
						errors_msg.distance_error = 0;
						errors_msg.angle_error = 0;

						if(qgcIsConnected) event = ERROR_RECOVERED;
					break;
				}
			break;
		}

		pub_errors.publish(errors_msg);
		loop_rate.sleep();
	}

	ros::shutdown();
}