#include <list>
#include <iostream>
#include <iomanip>
#include <mutex>

using namespace std;

// contains GPS coordinates and its purpose

typedef struct {
	uint32_t duration; // only relevant for station keep. else this is zero
	float acceptanceRadius; //margin for determining if the waypoint was reached
	float passThrough;
	float desiredYaw;
	float lat; //lattitude
	float lon; // longitude
	float alt; //altitude

	bool dest; //true if waypoint is a new destination, false for station keep
	uint16_t seq;
} Waypoint;

typedef list<Waypoint> Mission;

class missions {
public:
	missions();
	~missions();

	//returns all waypoints
	Mission getFullMission();

	//true if mission is not empty and getNextWaypoint() can therefore be called, false otherwise
	bool hasWaypoints();

	// returns total number of waypoints (completed and not completed)
	uint16_t getNumWaypoints();

	//returns the current Waypoint
	Waypoint getNextWaypoint();

	// returns true if it's the misson's last waypoint
	bool lastWaypoint(uint16_t seq);

	//replaces the current mission with newMission
	void updateMission(Mission newMission);

	//iterator moves to the next waypoint
	void clearWaypoint(uint16_t seq);

	//true if curr == the current waypoint in mission. false otherwise
	bool compareWaypoint(Waypoint curr);

	//prints waypoints from first to last (from the end of the list to the beginning)
	void printMission();
	void clearMission();


private:
	Mission mission;
	bool active;
	Mission::iterator currentWaypoint;
	mutex mtx;
};
