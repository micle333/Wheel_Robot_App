#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>  // gyro
#include <geometry_msgs/TwistStamped.h> // gnss vel
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h> // navigation
#include <geometry_msgs/TransformStamped.h>
#include <ublox_msgs/NavPVT.h> // gnss pos

#include "platform_velocity_and_pose/threewheel.h"
#include "platform_velocity_and_pose/threewheel_types.h"
#include "platform_controller_comm/WheelRotation.h" // encoder
//#include <bondcpp/bond.h>

/* GLOBAL  VARIABLES */
int platformID = 0; // platform's ID isn used to define correct platform parameters
					// like wheel radius, wheel axis length and maybe others
					// ID = 1 - roko_1 platform (white, egg-shaped)
					// ID = 2 - denis platform (promo bot, small black chasis, miracle-car)
// global Kalman state and parameters structures
static StateStruct ekfState;
static ParametersStruct ekfParam;
// this publisher is global because two callback
// functions need access to it.
ros::Publisher odom_pub;
tf2_ros::TransformBroadcaster* tf_broadcaster;
// used to calculate dT on every update
unsigned int last_timestamp = 0;
// PI constant
const double pi = 3.141592653589793;
// these are used to calculate lat0 and lon0 ???
// in case when gps update is not available from the start. (NED frame) ???
// Also these are the last calculated values for navigation solution.
float posX = 0.0F;
float posY = 0.0F;
float posT = 0.0F;
// Heading offset defines rotation of local frame from NORTH direction.
// It is entered manualy with _heading ROS parameter. (Degrees)
float heading_offset = 0.0F;
// Initial geo reference point (lat0, lon0). These are in RADIANS!
float referenceLat_rad = 0.0F;
float referenceLon_rad = 0.0F;
// Initial position in local frame. (meters)
// These are also local coordinates of the geo reference point.
float referenceNED_x = 0.0F;
float referenceNED_y = 0.0F;
// used to store last geo coordinates and velocities
// these values will be consumed in the closest locomotion_callback()
float lastLat = 0.0F;
float lastLon = 0.0F;
float lastNspeed = 0.0F;
float lastEspeed = 0.0F;
// used to store last odometer measurements
// these values will be consumed in the closest locomotion_callback()
float odoLeft = 0.0F;
float odoRight = 0.0F;
// Gps correction stationary ZUPT variables
float stationaryHeading = 0.0F;
bool stationary = true;
bool notMovedYet = true;
// Heading correction from SLAM variable
bool slamupdate_ENABLED = false; // defines is slam update is used ('false' will disable it completly)
bool slamUpdate = false;
float lastQuat[4] = {0, 0, 0, 1};
float newQuat[4] = {0, 0, 0, 1};
float lastLocalHeading = 0.0F;
// SLAM data callback variables
//tf::TransformListener tf_listener;
//tf::StampedTransform tf_transform;
int navigationCounter = 0;
ros::Time last_msg_time;


/* Function Declarations */
static void KalmanParametersInit(ParametersStruct *result);
void motors_callback(const platform_controller_comm::WheelRotation msg);
void locomotion_callback(const sensor_msgs::Imu msg);
void gnss_pos_callback(const ublox_msgs::NavPVT msg);
void gnss_vel_callback(const geometry_msgs::TwistWithCovarianceStamped msg);
void keyboard_callback(const std_msgs::String msg);
int main(int argc, char** argv);
float pi_to_pi(float angle, int metric = 0);
void ned_to_local(float nedCoord[3]);
float dt_calc(unsigned int timestamp);
static void KalmanInit(void);
static SensorsStruct sensorsInit();
static void stateInit(StateStruct *result);
static float quat2angle(float quat[4]);
static void odomPublish(float x, float y, float heading, float lx, float ly, float lheading, float dt);

/* Function Definitions */

// Initialize ParametersStruct
// All Kalman settings are defined here.
static void KalmanParametersInit(ParametersStruct *result)
{
	// Odometer sample rate
	result->dt = 2.0F/100.0F;
	// Odometer parameters
	struct0_T odoTemp;
	odoTemp.lr = 2*pi*0.1; // right wheel length
	odoTemp.ll = 2*pi*0.1; // left wheel length
	odoTemp.d = 1.2F;	// wheel axis length
	odoTemp.thr = 0.005F;// Threshold for odometer angular rate
	result->odo = odoTemp;
	// GPS parameters
	ParametersGPSStruct gpsTemp;
	gpsTemp.update = 0; // enable(1)/disable(0) gps update
	gpsTemp.lb[0] = 0.85F; // gps lever length (position offset), in meters
	gpsTemp.lb[1] = 0.1F;
	gpsTemp.lb[2] = 0.1F;
	gpsTemp.pos_noise = 0.8F; // gps position noise in meters (3.0 = 1 sigma)
	gpsTemp.vel_noise = 0.4F; // gps velocity noise in m/s (0.1 = 1 sigma)
	gpsTemp.lat0 = referenceLat_rad; // NED frame origin, rad
	gpsTemp.lon0 = referenceLon_rad; // NED frame origin, rad
	gpsTemp.a = 6378137.0F; // ellipsoid radius (WGS-84)
	gpsTemp.e = 0.081819190842621F; // ellipsoid eccentricity (WGS-84)
	result->gps = gpsTemp;
	// Gyroscope parameters
	ParametersGYRStruct gyroTemp;
	gyroTemp.update = 0; // enable(1)/disable(0) gyro update
	gyroTemp.noise = 0.001F; // gyroscope measurement noise
	result->gyr = gyroTemp;
	// Initial erros and parameters estimation (?)
	ParametersINIStruct iniTemp;
	iniTemp.Rn[0] = referenceNED_x; // NED coordinate system
	iniTemp.Rn[1] = referenceNED_y; // reference point + initial offset
	// Initial heading with respect to the NED frame
	iniTemp.heading = heading_offset;
	iniTemp.dw = 0.0F; // initial value for gyro bias
	iniTemp.dlr = 0.0F; // initial right wheel length error
	iniTemp.dll = 0.0F; // initial left wheel length error
	result->ini = iniTemp;
	// EFK parameters
	ParametersEKFStruct ekfTemp;
	ekfTemp.nlr = 0.001F; // noise for right wheel length
	ekfTemp.nll = 0.001F; // noise for left wheel length
	ekfTemp.nw = 0.0005F; // 1e-4
	ekfTemp.nlre = 0.0000000001F;
	ekfTemp.nlle = 0.0000000001F; // 1e-10
	ekfTemp.nwe = 0.00000001F; // 1e-8
	result->ekf = ekfTemp;
	// SLAM parameters
	ParametersSLAMStruct slamTemp;
	slamTemp.update = 0; // noise for right wheel length
	slamTemp.pos_noise = 0.1F; // noise for left wheel length
	slamTemp.heading_noise = 0.8F; // 0.8
	result->slam = slamTemp;
	result->reset = 0; // 1 for reset, 0 for no reset
}

// Process incoming motors data - odometers mostly
void motors_callback(const platform_controller_comm::WheelRotation msg) {
	// Odometer values have some noise, so we cut it
	// out if value is too low.
	if (fabs(msg.frontLeft) > 0.2F) // odo[0] is LEFT
		odoLeft = msg.frontLeft;
	else
		odoLeft = 0.0F;
    if (fabs(msg.frontRight) > 0.2F) // odo[1] is RIGHT
		odoRight = msg.frontRight;
	else
		odoRight = 0.0F;
}

float fround(float var)
{
	int c = (int)(var * 100 + .5);
    float b = c / 100.0;
    return b;
}

// Process incoming locomotion data
void locomotion_callback(const sensor_msgs::Imu msg) {
	// Try to hook up slam update data
	// static tf::TransformListener tf_listener;
	// static tf::StampedTransform tf_transform;
	// if (slamupdate_ENABLED)
	// {
	// 	try
	// 	{
	// 		tf_listener.lookupTransform("/map", "/base_footprint", ros::Time(0), tf_transform);
	// 		newQuat[0] = tf_transform.getRotation().getAxis().getX();
	// 		newQuat[1] = tf_transform.getRotation().getAxis().getY();
	// 		newQuat[2] = tf_transform.getRotation().getAxis().getZ();
	// 		newQuat[3] = tf_transform.getRotation().getW();
	// 		slamUpdate = true;
	// 	}
	// 	catch (tf::TransformException &ex) {
	// 		//ROS_ERROR("%s",ex.what());
	// 		//ros::Duration(1.0).sleep();
	// 		//continue;
	// 	}
	// }
	OutStruct Out;
	SensorsStruct measurement = sensorsInit();

	// TODO: gyro update is acting weird. I disabled it. 09.02.2024
    ekfParam.gyr.update = 0;
    ekfParam.gps.update = 0;
    ekfParam.slam.update = 0;

	// GPS measurement
	if ((lastLon != 0) && (lastLat != 0) && ((lastNspeed != 0) || (lastEspeed != 0)))
	{
		measurement.gps.pos[0] = lastLat;
		measurement.gps.pos[1] = lastLon;
		measurement.gps.vel[0] = lastNspeed;
		measurement.gps.vel[1] = lastEspeed;
		ekfParam.gps.update = 1;
		//lastLat = 0.0F;
		//lastLon = 0.0F;
		lastNspeed = 0.0F;
		lastEspeed = 0.0F;
		float gpsx = (lastLat - ekfParam.gps.lat0) * 180.0 / pi * 111111 ;
		float gpsy = (lastLon - ekfParam.gps.lon0) * 180.0 / pi * 111111 * cos(lastLat);
		//printf("GPS: %.2f %.2f \n", gpsx, gpsy);
	}

	// SLAM measurement
	// if (slamupdate_ENABLED && slamUpdate)
	// {
	// 	if ( (newQuat[3] != 1) && (newQuat[3] != 0) && (newQuat[3] != lastQuat[3]) )
	// 	{
	// 		float dthe_slam = lastLocalHeading - quat2angle(newQuat);
	// 		dthe_slam = pi_to_pi(dthe_slam, 0);
	// 		measurement.slam.dthe = dthe_slam;
	// 		ekfParam.slam.update = 1;
	// 		for (int ins = 0; ins < 4; ins++)
	// 			lastQuat[ins] = newQuat[ins];
	// 	}
	// 	slamUpdate = false;
	// }

	// Odometer measurement
	measurement.odo.fil = odoLeft;
	measurement.odo.fir = odoRight;

    // calculate time difference between measurements
	float dT = msg.header.stamp.toSec() - last_msg_time.toSec();
	last_msg_time = msg.header.stamp;
	ekfParam.dt = dT;

    // Gyro measurement
	// Angle is negative because axis is pointing the wrong way
    //measurement.gyr.dthe = -msg.skew * dT;
	// Convert gyro measurement to Kalman's frame (which has weird axes)
    measurement.gyr.dthe = msg.angular_velocity.z * dT;
	// float odo_rate = (measurement.odo.fir - measurement.odo.fil) * 2*pi*0.2 / 2 / pi / 1.4;
	// printf("Gyro: %.2f odo: %.2f \n", measurement.gyr.dthe / dT, odo_rate);
    // Zero update
    if ((measurement.odo.fil == 0) && (measurement.odo.fir == 0))
    {
		if (!stationary)
		{
			stationary = true;
			stationaryHeading = posT;
		}
		measurement.gyr.dthe = stationaryHeading - posT;
	}
	else
	{
		if (notMovedYet) notMovedYet = false;
		if (stationary) stationary = false;
	}

    // Actual Kalman filter call
	threewheel(&measurement, &ekfState, &ekfParam, &Out);

	if (ekfParam.reset != 0)
		ekfParam.reset = 0;
	else
	{
		float posX1 = posX;
		float posY1 = posY;
		float posT1 = posT;
		// replace last geo coordinates
		posX = Out.Rn[0];
		posY = Out.Rn[1];
		posT = Out.Heading;
		// transform navigation solution from NED to local frame
		float coordTransform[] = {Out.Rn[0], Out.Rn[1], 0};
		ned_to_local(coordTransform);
		//coordTransform[1] =  -coordTransform[1];
		float localHeading = -pi_to_pi(heading_offset*pi/180.0F - Out.Heading);
		float localHeading1 = pi_to_pi(heading_offset*pi/180.0F - posT1);
		float coordTransform1[] = {posX1, posY1, 0};
		// ned_to_local(coordTransform1);
		//coordTransform1[1] =  -coordTransform1[1];

		if (++navigationCounter == 1)
		{
			navigationCounter = 0;
			odomPublish(coordTransform[0], coordTransform[1], localHeading, coordTransform1[0], coordTransform1[1], localHeading1, dT);
		}

		if ((lastLon != 0) && (lastLat != 0) && ((lastNspeed != 0) || (lastEspeed != 0)))
		{
			lastLat = 0.0F;
			lastLon = 0.0F;
		}

	}
}

// Process incoming GNSS position data
void gnss_pos_callback(const ublox_msgs::NavPVT msg) {
	// GPS measurement
	// measurements come in degrees. But filter works with radians.
	float newLat = msg.lat * pi / 180.0F * 1e-7;
	float newLon = msg.lon * pi / 180.0F * 1e-7;

	if ((fabs(msg.velN / 1000.0) < 20) && (fabs(msg.velE / 1000.0) < 20))
	{
		lastNspeed = msg.velN / 1000.0;
		// Alright so the Kalman Y axis is inverted. Thats why we invert gnss velocity measurement
		// Don't look at me, I do not like this
		lastEspeed = -msg.velE / 1000.0;
	}
	//printf("gnss measurements: %.2f %.2f %.2f %.2f \n", newLat, newLon, lastNspeed, lastEspeed);

    bool referenceNeeded = false;
    bool validCoords = false;
    // This section contains hard coded geo coordinates limits.
    // This was added to avoid HUGE jumps in geo positioning due to GPS errors.
    // If using robot far away from Moscow - you might want to disable this.
    if ((msg.lat * 1e-7 > 53) && (msg.lat * 1e-7 < 58) &&
			(msg.lon * 1e-7 > 35) && (msg.lon * 1e-7 < 40)) {
		validCoords = true;
	}
    if (validCoords && (msg.fixType >= 0) &&
			(msg.lat != 0) && (msg.lon != 0)) {
		// if lat0 or lon0 hasn't been set - reference point calculation
		// is needed
		if ((ekfParam.gps.lat0 == 0)||(ekfParam.gps.lon0 == 0)) {
			referenceNeeded = true;
		}
		if (referenceNeeded) {
			// distance between lat0 and the last local frame coordinate (rad)
			float shiftLat = posX * pi / 111111 / 180.0F;
			// distance between lon0 and the last local frame coordinate (rad)
			float shiftLon = posY * pi / (111111*cos(newLat)) / 180.0F;
			// 1 geo degree equals to 111111 meters (approx.)
			ekfParam.gps.lon0 = newLon - shiftLon;
			ekfParam.gps.lat0 = newLat - shiftLat;
		}

		// replace last GEO coordinates
		if (notMovedYet)
		{
			ekfParam.gps.lon0 = newLon;
			ekfParam.gps.lat0 = newLat;
		}
		else
		{
			if (msg.fixType > 0)
			{
				lastLat = newLat;
				// Correction for weird Kalman frame.
				// Basicly inverting Y axis in NED frame
				lastLon = 2 * ekfParam.gps.lon0 - newLon;
			}
		}
	}

	// TODO: Adjust errors covariance based on GNSS status
	//if (msg.fixType == 2)
	//{
	//	ekfParam.gps.pos_noise = 0.1F;
	//	ekfParam.gps.vel_noise = 0.1F;
	//}
	//else
	//{
	//	ekfParam.gps.pos_noise = 1.0F;
	//	ekfParam.gps.vel_noise = 0.2F;
	//}
}

// Process incoming GNSS velocity data
void gnss_vel_callback(const geometry_msgs::TwistWithCovarianceStamped msg)
{
	// NOTE: not needed after transition
	// if ((fabs(msg.twist.twist.linear.x) < 20) && (fabs(msg.twist.twist.linear.y) < 20))
	// {
	// 	lastNspeed = msg.twist.twist.linear.y;
	// 	lastEspeed = msg.twist.twist.linear.x;
	// }

}

// Process incoming keyboard commands
void keyboard_callback(const std_msgs::String msg)
{
	if (msg.data.compare("nr") == 0)
	{
		ekfParam.reset = 1;
		posX = 0.0F;
		posY = 0.0F;
		posT = 0.0F;
		ekfParam.gps.lon0 = 0.0F;
		ekfParam.gps.lat0 = 0.0F;
		KalmanInit();
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "navigation_node");
	ros::NodeHandle nh;
	last_msg_time = ros::Time::now();

	ros::Subscriber motors_sub = nh.subscribe<platform_controller_comm::WheelRotation>("odo_converted", 3, motors_callback);
	ros::Subscriber locomotion_sub = nh.subscribe<sensor_msgs::Imu>("main_imu_data", 3, locomotion_callback);
	ros::Subscriber gnss_sub = nh.subscribe<ublox_msgs::NavPVT>("sc/navpvt", 3, gnss_pos_callback);


	// ros::Subscriber gnss_vel_sub =
	// 	nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>
	// 	("gps_node/fix_velocity", 10, gnss_vel_callback);
	// ros::Subscriber keyboard_sub =
	// 	nh.subscribe<std_msgs::String>("keyboard_commands", 100, keyboard_callback);

	// NOTE: PROBABLY not needed after transition
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
	tf2_ros::TransformBroadcaster tf_broadcaster_instance;
	tf_broadcaster = &tf_broadcaster_instance;


	//tf_listener = tf::TransformListener(nh);

	ros::Rate loop_rate(100);
	// separate hande to HANDLE (ahaha) parameter input
	ros::NodeHandle nh_private("~");

    // Get heading parameter
    float init_heading = 0.0F;
    nh_private.getParam("heading", init_heading);
	// Initial heading is inverted to adjust for weird Kalman frame 09/02/2024
    heading_offset = pi_to_pi(-init_heading, 1); // second parameter is set to 'degrees'
    printf("Heading offset: %f degrees.\n", heading_offset);
    printf("Use _heading:=[value] to change it.\n");

	// NOTE: Supporting multiple platform setting is probably not need after transition
    // nh_private.getParam("platform", platformID);
    // printf("Platform ID: %d \n", platformID);
	// if (platformID == 0)
	// {
	// 	printf("Platform ID is not set. Please use _platrform:=[id] parameter to set the ID.\n");
	// 	//return 1;
	// }
	// Receives id from A using a service or action
  //bond::Bond bond("sensor_checker_bonds", "Gyroscope");
  //bond.start();

	KalmanInit();

    ros::spin();
	//bond.breakBond();
	return 0;
}

// Limit angle to [-pi,pi) interval. Metric system is defined by
// metric argument.
// metric = 0 (for radians, default)
// metric = 1 (for degrees)
float pi_to_pi(float angle, int metric)
{
	double edgeValue = pi;
	if (metric == 1)
		edgeValue = 180.0;
	while ((angle <= -edgeValue) || (angle > edgeValue))
	{
		if (angle <= -edgeValue)
			angle = angle + 2*edgeValue;
		if (angle > edgeValue)
			angle = angle - 2*edgeValue;
	}
	return angle;
}

// Transform coordinates from NED to localframe, using cosine matrix
// based on heading_offset parameter
void ned_to_local(float nedCoord[3])
{
	float localCoord[] = {0, 0, 0};
	float offset_rad =  -heading_offset*pi/180.0F;
	localCoord[0] = nedCoord[0]*cos(offset_rad) - nedCoord[1]*sin(offset_rad);
	localCoord[1] = nedCoord[0]*sin(offset_rad) + nedCoord[1]*cos(offset_rad);
	nedCoord[0] = localCoord[0];
	nedCoord[1] = localCoord[1];
}

// Calculate timestamp difference
float dt_calc(unsigned int timestamp)
{
	float dT = 0.0F;
	if (last_timestamp == 0)
		dT = 0.01;
	else
		dT = (float)(timestamp-last_timestamp)/1000;
	last_timestamp = timestamp;
	return dT;
}

// This function is used to setup and start Kalman filter.
static void KalmanInit(void)
{
  SensorsStruct sens0 = sensorsInit();
  OutStruct Out;

  // initial values for heading ZUPT
  stationaryHeading = heading_offset*pi/180.0F;
  posT = heading_offset*pi/180;

  stateInit(&ekfState); // Fill state with zeros
  KalmanParametersInit(&ekfParam); // Set EKF parameters
  threewheel(&sens0, &ekfState, &ekfParam, &Out); // Start Kalman filter
}

// Creates zeros-filled SensorsStruct
static SensorsStruct sensorsInit()
{
	SensorsStruct measurement;
	// GPS measurement
	SensorsGPSStruct gpsMeas;
	gpsMeas.pos[0] = 0.0F;
	gpsMeas.pos[1] = 0.0F;
	gpsMeas.vel[0] = 0.0F;
	gpsMeas.vel[1] = 0.0F;
	measurement.gps = gpsMeas;
	// Gyro measurement
	SensorsGYRStruct gyrMeas;
	gyrMeas.dthe = 0.0F;
	measurement.gyr = gyrMeas;
	// Odometer measurement
	SensorsODOStruct odoMeas;
	odoMeas.fir = 0.0F;
	odoMeas.fil = 0.0F;
	measurement.odo = odoMeas;
	// SLAM measurement
	SensorsSLAMStruct slamMeas;
	slamMeas.pos[0] = 0.0F;
	slamMeas.pos[1] = 0.0F;
	slamMeas.dthe = 0.0F;
	measurement.slam = slamMeas;

	return measurement;
}

// Fills StateStruct with zeros
static void stateInit(StateStruct *result)
{
	// covariance matrix
	for (int ir = 0; ir < 36; ir++)
		result->P[ir] = 0.0F;
	// NED coordinates
	result->Rn[0] = 0.0F;
	result->Rn[1] = 0.0F;
	// cosine matrix
	for (int ir = 0; ir < 9; ir++)
		result->Cbn[ir] = 0.0F;
	// gyro bias
	result->dw = 0.0F;
	// wheel length errors
	result->dlr = 0.0F;
	result->dll = 0.0F;
	// state (0 - Kalman initialization required)
	result->state = 0;
}


// Transform quaternion to angles
static float quat2angle(float quat[4])
{
	float result = 0.0F;
	// Normalize quaternion
	float norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
	quat[0] = quat[0] / norm;
	quat[1] = quat[1] / norm;
	quat[2] = quat[2] / norm;
	quat[3] = quat[3] / norm;

	// DCM elements
	float c21 = 2 * (quat[1] * quat[2] + quat[0] * quat[3]);
	float c11 = quat[0]*quat[0] + quat[1]*quat[1] - quat[2]*quat[2] - quat[3]*quat[3];
	float c31 = 2 * (quat[1] * quat[3] - quat[0] * quat[2]);
	float c32 = 2 * (quat[2] * quat[3] + quat[0] * quat[1]);
	float c33 = quat[0]*quat[0] - quat[1]*quat[1] - quat[2]*quat[2] + quat[3]*quat[3];

	// Angles from DCM
	float ang1 = atan2( c21, c11 );
	float ang2 = asin( -c31 );
	float ang3 = atan2( c32, c33 );

	result = ang3;
	return result;
}

static void odomPublish(float x, float y, float heading, float lx, float ly, float lheading, float dt) {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;//0.155;
    msg.pose.pose.orientation.z = sin(heading / 2.0);
    msg.pose.pose.orientation.w = cos(heading / 2.0);
    msg.pose.covariance[0] = 0.00001;
    msg.pose.covariance[7] = 0.00001;
    msg.pose.covariance[14] = 1000000000000.0;
    msg.pose.covariance[21] = 1000000000000.0;
    msg.pose.covariance[28] = 1000000000000.0;
    msg.pose.covariance[35] = 0.001;
    odom_pub.publish(msg);

    geometry_msgs::TransformStamped tfmsg;
    tfmsg.header.stamp = ros::Time::now();
    tfmsg.child_frame_id = "base_link";
    tfmsg.header.frame_id = "odom";
    tfmsg.transform.translation.x = x;
    tfmsg.transform.translation.y = y;
    tfmsg.transform.translation.z = 0.0;//0.155;
    tfmsg.transform.rotation.z = sin(heading / 2.0);
    tfmsg.transform.rotation.w = cos(heading / 2.0);
    tf_broadcaster->sendTransform(tfmsg);

    geometry_msgs::TransformStamped map_tfmsg;
    map_tfmsg.header.stamp = ros::Time::now();
    map_tfmsg.child_frame_id = "odom";
    map_tfmsg.header.frame_id = "map";
    map_tfmsg.transform.translation.x = 0;
    map_tfmsg.transform.translation.y = 0;
    map_tfmsg.transform.translation.z = 0;
    map_tfmsg.transform.rotation.z = 0;
    map_tfmsg.transform.rotation.w = 1;
    tf_broadcaster->sendTransform(map_tfmsg);
}
