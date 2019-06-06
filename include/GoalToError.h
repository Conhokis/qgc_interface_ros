#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

class GoalToError {
public:
	GoalToError(double start_latitude, double start_longitude, double start_altitude, double start_yaw);
	void updateErrors(double latitude_goal, double longitude_goal, geometry_msgs::TransformStamped current_tf);
	void updatePosition(geometry_msgs::TransformStamped current_tf);
	double getDistanceError();
	double getAngleError();
	double getCurrentLatitude();
	double getCurrentLongitude();
	float getCurrentYaw();

private:
	double x_start, y_start, theta_start, altitude, zone, band;
	double distance_error, angle_error;
	double current_latitude, current_longitude;
	float current_yaw;

	void latLonToXY(double latitude, double longitude, double* x_return, double* y_return);
	double degreesToRadians(double degrees);
	double quaternionToYaw(geometry_msgs::Quaternion);
};