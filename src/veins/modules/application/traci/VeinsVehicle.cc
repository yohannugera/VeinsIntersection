//
// Our Algorithm...
//
// CoBegin // for each vehicle i
//
// On entering the monitoring area:
//      send REQUEST(i,lid_of_i);
//      st_of_i = WAITING;
//      wait for PERMIT from Controller;
//
// On Receiving PERMIT(plt):
//      if (i in plt){
//          st_of_i = PASSING;
//          move and pass the core area;
//      }
//
// On exiting the intersection:
//      if (i is the last vehicle in pass list){
//          send RELEASE(i, lid_of_i) to the Controller;
//      st_of_i = IDLE;
//
// CoEnd
//
// AWARE - vehicles send packets with its speed, position, parking status to other vehicles
// CoreAWARE - vehicles send these packets when they're in the Monitoring area. Used to send parking slot number (Queue no.)

#include "veins/modules/application/traci/VeinsVehicle.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <string.h>
#include <omnetpp.h>

#include <map>
#include <limits>

using namespace std;
using namespace omnetpp;
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t VeinsVehicle::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);

// Vehicle States
const int IDLE      = 0;
const int WAITING   = 1;
const int PASSING   = 2;

// Packet Type - PsID
const int AWARE     = 0;
const int REQUEST   = 1;
const int RELEASE   = 2;
const int PERMIT    = 3;

// Vehicular Attributes
const double minGap = 1.0;
const double length = 5.0;

Define_Module(VeinsVehicle);

void VeinsVehicle::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);

		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();

		// Newly Added...
		vehicleState = IDLE;
		occupiedLength = 0.0;
		mylane = traciVehicle->getLaneId();
	}
}

void VeinsVehicle::onBeacon(WaveShortMessage* wsm) {
    // Vehicle receiving a beacon from RSU
    //      - Vehicle is in the Monitoring Zone  ->  send REQUEST as a Data msg.
    if(wsm->getSenderAddress() == 9999){
        if(vehicleState == IDLE and !(traciVehicle->getLaneId().c_str()[0] == '-')){
            // Vehicle status change...
            vehicleState = WAITING;

            // Stopping at intersection...
            traci->vehicle(mobility->getExternalId()).stopAt(mobility->getRoadId(), traci->lane(traciVehicle->getLaneId()).getLength(), 0, 10, 999);

            // Sending REQUEST...
            t_channel channel = dataOnSch ? type_SCH : type_CCH;
            WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
            wsm->setSenderAddress(stoi(mobility->getExternalId().substr(3)));       // By this method - external nodes are limited to (RSUAddress-1)
            wsm->setPsid(REQUEST);
            wsm->setLane(traciVehicle->getLaneId().c_str());
            sendWSM(wsm);
        }
    }
}

void VeinsVehicle::onData(WaveShortMessage* wsm) {
    // Vehicle receiving a data from RSU
    //      - RSU only transmit PERMIT as data
    if(wsm->getSenderAddress() == 9999){

        // Check the incoming packet Lane -> sample Lane = "1$2$3"
        string str = wsm->getLane();
        cout << str << endl;
        std::vector<int> permitted;
        std::stringstream ss(str);
        int i;
        while (ss >> i) {
            permitted.push_back(i);
            if (ss.peek() == '$')
                ss.ignore();
        }
        int my_ID = stoi(mobility->getExternalId().substr(3));
        bool exists = find(begin(permitted), end(permitted), my_ID) != end(permitted);

        // if this vehicle ID is included in the list -> exists = true
        if (exists) {
            // Move on if there's no preceding vehicles
            vehicleState = PASSING;

            // Setting motion...
            traciVehicle->setSpeedMode(8);
            traciVehicle->setSpeed(10.0);
        }
    }

	findHost()->getDisplayString().updateWith("r=16,blue");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));
	if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
}

void VeinsVehicle::sendMessage(std::string blockedRoadId) {
	sentMessage = true;

	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void VeinsVehicle::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
	Enter_Method_Silent();
	if (signalID == mobilityStateChangedSignal) {
		handlePositionUpdate(obj);
	}
	else if (signalID == parkingStateChangedSignal) {
		handleParkingUpdate(obj);
	}
}

void VeinsVehicle::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}

void VeinsVehicle::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);
    // Initial stop position setting...
	if (traciVehicle->getLaneId().c_str()[0] == '-' and vehicleState==PASSING){
        vehicleState = IDLE;

        // Sending RELEASE...
        t_channel channel = dataOnSch ? type_SCH : type_CCH;
        WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
        wsm->setSenderAddress(stoi(mobility->getExternalId().substr(3)));
        wsm->setPsid(RELEASE);
        wsm->setLane(mylane.c_str());
        sendWSM(wsm);
	}
}

void VeinsVehicle::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}
