/* These headers are for QNX, but should all be standard on unix/linux */
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif

/* This assumes you have the mavlink headers on your include path
 or in the same folder as this source file */
#include "commUtils.h"
#include <pthread.h>

#include <cmath>

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define MISSION_TRANSFER_TIMEOUT 1000000 //In microseconds, 100ms timeout atm

void* connectionThread(void* argument);

//Calling this method returns the time in microseconds
uint64_t microsSinceEpoch();

// Constructor
commUtils::commUtils(missions* new_mission) {
	isConnected = false;
	lat = 0;
	lon = 0;
	mission = new_mission;
	toTransfer = 0;

	//Download parameters
	totalWaypoints = 0;
	missionBuffer = Mission();
	downloadTimer = 0;

	//Upload parameters
	uploadBuffer = Mission();
	uploadCounter = 0;
	uploadTimer = 0;
}

// Destructor
commUtils::~commUtils() {

}

void commUtils::init() {
	pthread_create(&connectionThreadPtr, NULL, connectionThread, this);
}

void* connectionThread(void* argument) {
	commUtils *comm = (commUtils*) argument;

	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP); 
	uint8_t buf[BUFFER_LENGTH];

	//Connection Config
	char target_ip[100];
    strcpy(target_ip, "127.0.0.1");

	struct sockaddr_in locAddr;
	struct sockaddr_in gcAddr;
	//struct sockaddr_in fromAddr;
	ssize_t recsize;
	socklen_t fromlen;
	mavlink_message_t msg;
	mavlink_message_t rcvMsg;
	uint16_t len;
	int i = 0;

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
    } 
	
	/* Attempt to make it non blocking */
#if (defined __QNX__) | (defined __QNXNTO__)
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
#else
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
#endif

    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
    }
	
	
	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);

	//Start sending heartbeats and listening to response
	uint64_t lastHbTime = 0;
	//Checks for QGC heartbeats, used to measure if a heartbeat is received every second. If you miss 3 in a row, consider system is disconnected
	bool checkConnection;
	//Counter for missed connections, 10 per second
	int missConnectCount = 0;

	//Main connection loop write and read
	for (;;) 
    {
    	//Heartbeat every one 0.1 seconds
		if(microsSinceEpoch() - lastHbTime > 100000) {
			/*Send Heartbeat */
			mavlink_msg_heartbeat_pack(1, 1, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
			//Update lastHbTime
			lastHbTime = microsSinceEpoch();

			//Regulates the logic behind the connection state
			if (checkConnection == false)
			{
				missConnectCount++;
				if (missConnectCount >= 30)
				{
					comm->setConnectionState(false);
				}
			}
			else {
				missConnectCount = 0;
				checkConnection = false;
				comm->setConnectionState(true);
			}

			comm->sendGPS_QGC(buf, msg, sock, gcAddr);
		}

		comm->checkDownloadTimeout();
		comm->checkUploadTimeout();

		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);
		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_status_t status;

			for (i = 0; i < recsize; ++i)
			{
				mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rcvMsg, &status);
			}

			/*
			Initial configuration of QGC:
			1. QGC sends PARAM_REQUEST_LIST (MsgID #21)
			2. Respond with PARAM_VALUE #22
			3. Respond to commands
			4. Respond to MISSION_REQUEST_LIST with MISSION_COUNT with 0 items
			*/
			switch(rcvMsg.msgid) {
				//Handle PARAM_REQUEST_LIST #21
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
					comm->handle_param_request_list(buf, msg, sock, gcAddr);
				break;

				//Handle COMMAND_LONG #76
				case MAVLINK_MSG_ID_COMMAND_LONG:
					comm->handle_command_long(buf, msg, sock, gcAddr, rcvMsg);
				break;

				//Handle MISSION_REQUEST_LIST #43
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
					comm->handle_mission_request_list(buf, msg, sock, gcAddr, rcvMsg);
				break;

				//Handle MISSION_COUNT #44. Start of mission download from QGC
				case MAVLINK_MSG_ID_MISSION_COUNT:
					comm->handle_mission_count(buf, msg, sock, gcAddr, rcvMsg);
				break;

				//Handle MISSION_ITEM #39. Continue mission download from QGC
				case MAVLINK_MSG_ID_MISSION_ITEM:
					comm->handle_mission_item(buf, msg, sock, gcAddr, rcvMsg);
				break;

				//Handle MISSION_CLEAR_ALL #45. Clears current mission from the vehicle
				case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
					comm->handle_mission_clear_all(buf, msg, sock, gcAddr);
				break;

				case MAVLINK_MSG_ID_MISSION_REQUEST:
					comm->handle_mission_request(buf, msg, sock, gcAddr);
				break;

				case MAVLINK_MSG_ID_MISSION_ACK:
					comm->handle_mission_ack();
				break;
			}
			//Sends all queued alerts to QGC if queue is not empty
			if(!comm->alertsEmpty()) comm->send_statustext(buf, msg, sock, gcAddr);
			//Sends all queued cleared mission items to QGC
			if(!comm->clearedEmpty()) comm->send_mission_item_cleared(buf, msg, sock, gcAddr);

			checkConnection = true;
		}
		memset(buf, 0, BUFFER_LENGTH);
    }
}

void commUtils::checkDownloadTimeout() {
	if(downloadTimer == 0) return;
	else if(microsSinceEpoch() - downloadTimer > MISSION_TRANSFER_TIMEOUT){
		totalWaypoints = 0;
		missionBuffer.clear();
		downloadTimer = 0;
	}
}

void commUtils::checkUploadTimeout() {
	if(uploadTimer == 0) return;
	else if(microsSinceEpoch() - uploadTimer > MISSION_TRANSFER_TIMEOUT){
			uploadCounter = 0;
			uploadBuffer.clear();
			uploadTimer = 0;
	}
}

void commUtils::handle_mission_ack() {
	uploadCounter = 0;
	uploadBuffer.clear();
	uploadTimer = 0;
}

void commUtils::handle_mission_request(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	Waypoint waypoint = uploadBuffer.front();

	mavlink_msg_mission_item_pack(1, 1, &msg, 1, 1, waypoint.seq, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT,
		(uint8_t) mission->compareWaypoint(waypoint), 1, 0, waypoint.acceptanceRadius, waypoint.passThrough, waypoint.desiredYaw,
		waypoint.lat, waypoint.lon, waypoint.alt, MAV_MISSION_TYPE_MISSION);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	uploadBuffer.pop_front();
}


void commUtils::send_mission_item_cleared(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	uint16_t seq;
	for(std::vector<uint16_t>::iterator iter = clearBuffer.begin(); iter != clearBuffer.end(); ++ iter) {
		mavlink_msg_mission_item_reached_pack(1, 1, &msg, *iter);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		seq = *iter;
	}

	clearBuffer.clear();

	//Anounces current mission is the next one
	mavlink_msg_mission_current_pack(1, 1, &msg, seq+1);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

void commUtils::handle_mission_clear_all(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	mission->clearMission();

	mavlink_msg_mission_ack_pack(1, 1, &msg, 1, 1, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

void commUtils::handle_mission_item(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg) {
	mavlink_mission_item_t mission_item;
	mavlink_msg_mission_item_decode(&rcvMsg, &mission_item);

	//mission_item.x = latitude, mission_item.y = longitude, mission_item.param2 = acceptance radius in meters
	//TODO duration is 0 and dest is true for now, may change later
	Waypoint waypointBuffer = {.duration = 0, .acceptanceRadius = mission_item.param2, .passThrough = mission_item.param3, 
		.desiredYaw = mission_item.param4, .lat = mission_item.x, .lon = mission_item.y, .alt = mission_item.z,
		.dest = true, .seq = mission_item.seq};

	//Acrescenta o waypoint à lista
	missionBuffer.push_back(waypointBuffer);

	//Finish download
	if(missionBuffer.size() == totalWaypoints) {
		//Updates list on mission class
		mission->updateMission(missionBuffer);

		//Reset download parameters
		totalWaypoints = 0;
		missionBuffer.clear();
		downloadTimer = 0;

		//Send MISSION_ACK with MAV_MISSION_ACCEPTED to confirm end of download
		mavlink_msg_mission_ack_pack(1, 1, &msg, 1, 1, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	}

	//Request the next mission item and continue download
	else {
		mavlink_msg_mission_request_pack(1, 1, &msg, 1, 1, missionBuffer.size(), MAV_MISSION_TYPE_MISSION);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	}
}

void commUtils::waypoint_cleared(uint16_t seq) {
	clearBuffer.push_back(seq);
}

bool commUtils::clearedEmpty() {
	return clearBuffer.empty();
}

void commUtils::send_statustext(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	for(std::vector<string>::iterator iter = alerts.begin(); iter != alerts.end(); ++ iter) {
		char * writable = new char[iter->size() + 1];
		std::copy(iter->begin(), iter->end(), writable);
		writable[iter->size()] = '\0'; // don't forget the terminating 0

		mavlink_msg_statustext_pack(1, 1, &msg, MAV_SEVERITY_ALERT, writable);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

		delete[] writable;
	}

	alerts.clear();
}

bool commUtils::alertsEmpty() {
	return alerts.empty();
}

void commUtils::send_alert(string message) {
	//Adds the message to the alerts queue
	alerts.push_back(message);
}

void commUtils::handle_mission_request_list(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg) {
	mavlink_mission_request_list_t mvl_mrl;
	mavlink_msg_mission_request_list_decode(&rcvMsg, &mvl_mrl);

	//mission_type = MAV_MISSION_TYPE_MISSION means qgc wants the whole mission list
	mavlink_msg_mission_count_pack(1, 1, &msg, 1, 1, mission->getNumWaypoints(), MAV_MISSION_TYPE_MISSION);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	uploadBuffer = mission->getFullMission();
	uploadTimer = microsSinceEpoch();
}

void commUtils::handle_param_request_list(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	mavlink_msg_param_value_pack(1, 1, &msg, "teste\0", 5, MAV_PARAM_TYPE_REAL32, 1, 0);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

void commUtils::handle_command_long(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg) {

	//Decode message and parameters
	mavlink_command_long_t mvl_cmd;
	mavlink_msg_command_long_decode(&rcvMsg, &mvl_cmd);

	//Checks which command was sent
	switch (mvl_cmd.command) {
		//519
		case MAV_CMD_REQUEST_PROTOCOL_VERSION:
			mavlink_msg_protocol_version_pack(1, 1, &msg, 200, 100, 200, 0, 0);
		break;

		//520
		case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
			//Create the capabilities bitmask
			uint64_t capabilities = 0;
			capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
			mavlink_msg_autopilot_version_pack(1, 1, &msg, capabilities, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0);
		break;
	}

	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

void commUtils::handle_mission_count(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg) {
	mavlink_mission_count_t misCount;
	mavlink_msg_mission_count_decode(&rcvMsg, &misCount);

	//If mission type is valid
	if(misCount.mission_type == MAV_MISSION_TYPE_MISSION) {
		//Store the number of waypoints
		totalWaypoints = misCount.count;
		//Start download timer for timeout
		downloadTimer = microsSinceEpoch();
		//The parametere 0 indicates that it's the first mission to receive
		mavlink_msg_mission_request_pack(1, 1, &msg, 1, 1, 0, MAV_MISSION_TYPE_MISSION);
		int len = mavlink_msg_to_send_buffer(buf, &msg);
		sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
	}
}

void commUtils::sendGPS_QGC(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr) {
	

	//Send GPS status
	//

	//Send GPS information
	//altitude = 30m = 30000mm
	mavlink_msg_global_position_int_pack(1, 1, &msg, (microsSinceEpoch()/1000), lat, lon, 50000, 50000, 0, 0, 0, UINT16_MAX);
	int len = mavlink_msg_to_send_buffer(buf, &msg);
	sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
}

void commUtils::setLatLon(float new_lat, float new_lon) {

	//Floating point to fixed point conversion
	uint32_t int_lat = ((uint32_t) std::abs(new_lat)) * 10000000L;
	int_lat += (uint32_t) ((std::abs(new_lat) - (uint32_t) std::abs(new_lat)) * 10000000L);
	if(new_lat < 0) int_lat = -int_lat;

	uint32_t int_lon = ((uint32_t) std::abs(new_lon)) * 10000000L;
	int_lon += (uint32_t) ((std::abs(new_lon) - (uint32_t) std::abs(new_lon)) * 10000000L);
	if(new_lon < 0) int_lon = -int_lon;

	lat = int_lat;
	lon = int_lon;
}

void commUtils::setConnectionState(bool state) {
	isConnected = state;
}

bool commUtils::getConnection() {
	return isConnected;
}


//============================================================================================

/* QNX timer version
Weird time code magic
*/
#if (defined __QNX__) | (defined __QNXNTO__)
uint64_t microsSinceEpoch()
{
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}
#else
uint64_t microsSinceEpoch()
{
	
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}
#endif
