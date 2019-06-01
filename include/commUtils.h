#include <MAVLINK/mavlink.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <list>
#include "missions.h"

#define BUFFER_LENGTH 2041 

class commUtils {
public:
	commUtils(missions* new_mission);
	~commUtils();

	//Internal (private) methods, these are declared public because the thread is considered external to the class
	//Changes the connection state
	void setConnectionState(bool state);
	//Handles the paramenter
	void handle_param_request_list(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	//Handles commmands. ATM: Protocol version and Autopilot capabilities
	void handle_command_long(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg);
	//Sends GPS data to QGC
	void sendGPS_QGC(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	
	//Handles mission count which starts the download from QGC
	void handle_mission_count(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg);
	//Handles mission_item_int which is necessary for mission plan download
	void handle_mission_item(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg);
	//Verifies if there was a timeout on the download routine, true menans timeout was reached
	void checkDownloadTimeout();
	
	//Clears the alert queue and sends all alerts to QGC
	void send_statustext(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	//Returns true if alerts queue is empty
	bool alertsEmpty();

	//Notifies QGC with the cleared mission
	void send_mission_item_cleared(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	//Returns true if clearBuffer is empty
	bool clearedEmpty();

	//Handles the order to clear (delete) mission
	void handle_mission_clear_all(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	
	//Handles mission list request, start of upload routine
	void handle_mission_request_list(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr, mavlink_message_t &rcvMsg);
	//Handles mission request, part of upload routine
	void handle_mission_request(uint8_t (&buf)[BUFFER_LENGTH], mavlink_message_t &msg, int sock, sockaddr_in &gcAddr);
	//Handles mission_ack, end of upload routine
	void handle_mission_ack();
	//Verifies if there was a timeout on the upload routine
	void checkUploadTimeout();


	//Public methods for use in main
	//Start connection in new thread, must be called once at start of main
	void init();
	//Indicates connection status, returns true if it has received a QGC heartbeat in the last 3 seconds
	bool getConnection();
	//Updates latitude and longitude of the vehicle
	void setLatLon(float new_lat, float new_lon);
	//Sends an alert to be displayed in QGC, HAS A MAXIMUM OF 49 CHARACTERS
	void send_alert(string message);
	//When mission item is cleared, updates QGC
	void waypoint_cleared(uint16_t seq);
private:
	//Indicates connection status
	bool isConnected;
	//Mission List
	missions* mission;
	//Latitude
	uint32_t lat;
	//Longitude
	uint32_t lon;
	//Next Waypoint to download
	uint16_t toTransfer;
	
	//Message buffer
	vector <string> alerts;
	//Mission clear buffer
	vector <uint16_t> clearBuffer;

	//Mission buffer when downloading
	Mission missionBuffer;
	//Total number of waypoints to download
	uint16_t totalWaypoints;
	//Download timer to detect if there was loss of connections
	uint64_t downloadTimer;

	//Mission buffer when uploading
	Mission uploadBuffer;
	//Counter for upload
	uint16_t uploadCounter;
	//Upload timer to detect if there was loss of connection
	uint64_t uploadTimer;

	pthread_t connectionThreadPtr;
};
