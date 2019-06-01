#include "missions.h"

//constructor

missions::missions() {
	mission = Mission();
	active = false;
}

//destructor

missions::~missions() {
}

bool missions::hasWaypoints() {
	return active;
}

uint16_t missions::getNumWaypoints() {
	return mission.size();
}

Mission missions::getFullMission() {
	lock_guard<mutex> lock(mtx);
	return mission;
}

Waypoint missions::getNextWaypoint() {
	return (*currentWaypoint);
}

bool missions::lastWaypoint(uint16_t seq) {
	return (seq == mission.back().seq);
}

void missions::clearWaypoint(uint16_t seq) {

	if (lastWaypoint(seq)) {
		active = false;
		return;
	}

	currentWaypoint++;

}

bool missions::compareWaypoint(Waypoint curr) {
	return (curr.lat == (*currentWaypoint).lat && curr.lon == (*currentWaypoint).lon && curr.seq == (*currentWaypoint).seq);
}

void missions::updateMission(Mission newMission) {
	lock_guard<mutex> lock(mtx);
	mission = newMission;
	currentWaypoint = mission.begin();
	active = true;
}

void missions::clearMission() {
	mission.clear();
	active = false;
}

void missions::printMission() {

	for (Mission::iterator it = mission.begin(); it != mission.end(); ++it) {
		if ((*it).dest) {
			cout << fixed << setprecision(7) << "Lat: " << (*it).lat << ", Lon: " << (*it).lon << " DEST" << endl;
		} else {
			cout << fixed << setprecision(7) << "Lat: " << (*it).lat << ", Lon: " << (*it).lon << " STATION, Duration: " << (*it).duration << endl;
		}
	}
}
