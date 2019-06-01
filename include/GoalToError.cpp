#include <cmath>
#include "GoalToError.h"
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <iostream>
#include <geodesy/utm.h>

GoalToError::GoalToError(double start_latitude, double start_longitude, double start_altitude, double start_yaw) {
 	altitude = start_altitude;
 	theta_start = start_yaw;
	latLonToXY(start_latitude, start_longitude, &x_start, &y_start);
	std::cout << std::setprecision(10) << "Start: " << x_start << " " << y_start << std::endl;
}

void GoalToError::latLonToXY(double latitude, double longitude, double* x_return, double* y_return) {
	geographic_msgs::GeoPoint geo_pt;
	geo_pt.latitude = latitude;
	geo_pt.longitude = longitude;
	geo_pt.altitude = altitude;
	geodesy::UTMPoint utm_pt(geo_pt);
	*x_return = utm_pt.easting;
	*y_return = utm_pt.northing;
	zone = utm_pt.zone;
	band = utm_pt.band;
}

double GoalToError::degreesToRadians(double degrees) {
	return (degrees * M_PI / 180);
}

double GoalToError::getDistanceError() {
	return distance_error;
}

double GoalToError::getAngleError() {
	return angle_error;
}

double GoalToError::getCurrentLatitude() {
	return current_latitude;
}

double GoalToError::getCurrentLongitude() {
	return current_longitude;
}

double GoalToError::quaternionToYaw(geometry_msgs::Quaternion quat) {
 	double siny_cosp = +2.0 * (quat.w * quat.z + quat.x * quat.y);
 	double cosy_cosp = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
 	return atan2(siny_cosp, cosy_cosp);
}

void GoalToError::updateErrors(double latitude_goal, double longitude_goal, geometry_msgs::TransformStamped current_tf) {
	geometry_msgs::Vector3 current_translation = current_tf.transform.translation;
	geometry_msgs::Quaternion current_quat = current_tf.transform.rotation;

	//Goal
	double x_goal;
	double y_goal;
	latLonToXY(latitude_goal, longitude_goal, &x_goal, &y_goal);

	double x_current = x_start + current_translation.x * cos(-theta_start) - current_translation.y * sin(-theta_start);
 	double y_current = y_start + current_translation.y * cos(-theta_start) + current_translation.x * sin(-theta_start);

 	double theta_goal = atan2(y_goal - y_current, x_goal - x_current);
 	double tf_yaw= quaternionToYaw(current_quat);

 	distance_error = sqrt(pow((x_goal - x_current),2.0) + pow((y_goal - y_current),2.0));
 	angle_error = theta_goal - (tf_yaw + theta_start);
}

void GoalToError::updatePosition(geometry_msgs::TransformStamped current_tf) {
	geometry_msgs::Vector3 current_translation = current_tf.transform.translation;

	double x_current = x_start + current_translation.x * cos(-theta_start) - current_translation.y * sin(-theta_start);
 	double y_current = y_start + current_translation.y * cos(-theta_start) + current_translation.x * sin(-theta_start);

 	geodesy::UTMPoint utm_pt(x_current, y_current, altitude, zone, band);
 	geographic_msgs::GeoPoint geo_pt;
 	geo_pt = geodesy::toMsg(utm_pt);

 	current_latitude = geo_pt.latitude;
 	current_longitude = geo_pt.longitude;
}