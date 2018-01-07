//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
// Our Algorithm...
// On Receiving REQUEST(i, laneID_i) from vehicle i
// if (all lanes in lsl_i are locked by vehicles at the same lane and |plt| < np){
//      add i to plt;
//      broadcast PERMIT(plt);
// }
// if (all lanes in lsl_i are unlocked){
//      lock the lanes in lsl_i;
//      construct the pass list plt by adding vehicles in the same lane (at most np vehicles);
//      broadcast PERMIT(plt);
// }
// else
//      add i to the pending list rp;
//
// On Receiving RELEASE(i, laneID_i)
// if (i is the vehicle recorded for unlocked lanes in lsl_i){
//      unlock lanes in lsl_i;
//      for each vehicle j that (j is in rp) do
//          if (lanes in lsl_j are unlocked){
//              lock lanes in lsl_j;
//              construct the pass list plt by adding at most np vehicles in the same lane;
//              broadcast PERMIT(plt);
//          }
// }

#include "veins/modules/application/traci/VeinsRSU.h"
#include <vector>
#include <map>
#include <string.h>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>

using namespace std;
using Veins::AnnotationManagerAccess;

// Packet Type - PsID
const int AWARE   = 0;
const int REQUEST = 1;
const int RELEASE = 2;
const int PERMIT  = 3;

// Maximum allowable number of vehicles
const int np = 3;

Define_Module(VeinsRSU);

void VeinsRSU::initialize(int stage) {
	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobi = dynamic_cast<BaseMobility*> (getParentModule()->getSubmodule("mobility"));
		ASSERT(mobi);
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);
		sentMessage = false;

		// Initializing newly added...
		rp.clear();     // rp = vehicles in the queue
		plt.clear();    // plt = vehicles that are allowed to pass the intersection
		rl.clear();     // rl = lanes of the vehicles in the queue (map from vehicleID to laneID)
		pll.clear();    // pll = currently allowed lanes
		ll.clear();     // ll = currently locked lanes
		count_RELEASE = 0;
	}
}

void VeinsRSU::onBeacon(WaveShortMessage* wsm) {
    // beacons sent by vehicles doesn't bother me...
}

void VeinsRSU::onData(WaveShortMessage* wsm) {
    // RSU received a REQUEST from a vehicle...
    if (wsm->getPsid() == REQUEST) {
        cout << "Received a REQUEST pkt. from " << wsm->getSenderAddress() << " : " << wsm->getLane() << "," << wsm->getPsid() << endl;

        // if (all lanes in lsl_i are unlocked){
        //      lock the lanes in lsl_i;
        //      construct the pass list plt by adding vehicles in the same lane (at most np vehicles);
        //      broadcast PERMIT(plt);
        // }
        if ((find(ll.begin(), ll.end(), laneNumber(wsm->getLane())) == ll.end()) and (plt.size()<np)){

            // Update permitted lanes list (pll)...
            pll.push_back(laneNumber(wsm->getLane()));
            pll.erase( unique( pll.begin(), pll.end() ), pll.end() );

            // Update locked lanes list (ll)... (remove repeated elements as well)
            std::vector<int> myvector = lsl(laneNumber(wsm->getLane()));
            for (vector<int>::iterator it = myvector.begin() ; it != myvector.end(); ++it)
                ll.push_back(*it);
            sort( ll.begin(), ll.end() );
            ll.erase( unique( ll.begin(), ll.end() ), ll.end() );

            // Update permitted vehicles list (plt)...
            plt.push_back(wsm->getSenderAddress());

            // Send plt list as a PERMIT message...
            if (plt.size() == np){
                t_channel channel = dataOnSch ? type_SCH : type_CCH;
                WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
                wsm->setSenderAddress(9999);
                ostringstream oss;
                if (!plt.empty()){
                  std::copy(plt.begin(), plt.end()-1, ostream_iterator<int>(oss, "$"));
                  oss << plt.back();
                }
                wsm->setLane(oss.str().c_str());
                wsm->setPsid(PERMIT);
                sendWSM(wsm);
            }
        }
        // else
        //      add i to the pending list rp;
        else{
            rp.push_back(wsm->getSenderAddress());
            rl[wsm->getSenderAddress()] = laneNumber(wsm->getLane());
        }
    }

    // RSU received a RELEASE from vehicle...
    else if (wsm->getPsid() == RELEASE) {

        // Update status
        cout << "Received a RELEASE pkt. from " << wsm->getSenderAddress() << " : " << wsm->getLane() << "," << wsm->getPsid() << endl;
        count_RELEASE += 1;
        cout << "Release count : " << count_RELEASE << endl;

        // On Receiving RELEASE(i, laneID_i)
        // if (i is the vehicle recorded for unlocked lanes in lsl_i){
        //      unlock lanes in lsl_i;
        //      for each vehicle j that (j is in rp) do
        //          if (lanes in lsl_j are unlocked){
        //              lock lanes in lsl_j;
        //              construct the pass list plt by adding at most np vehicles in the same lane;
        //              broadcast PERMIT(plt);
        //          }
        // }
        if (count_RELEASE==plt.size() and rp.size()!=0){

            //make new pll, plt, ll
            vector<int> new_plt;
            vector<int> new_pll;
            vector<int> new_ll;
            new_plt.clear();
            new_pll.clear();
            new_ll.clear();

            for (vector<int>::iterator it = rp.begin(); it != rp.end(); ++it){
                if (new_plt.size()<np){
                    if (find(new_ll.begin(), new_ll.end(), rl[*it]) == new_ll.end()){
                        // Update permitted lanes list (new_pll)...
                        new_pll.push_back(rl[*it]);
                        new_pll.erase( unique( new_pll.begin(), new_pll.end() ), new_pll.end() );

                        // Update locked lanes list(new_ll)...
                        std::vector<int> myvector = lsl(rl[*it]);
                        for (vector<int>::iterator it = myvector.begin() ; it != myvector.end(); ++it)
                            new_ll.push_back(*it);
                        sort( new_ll.begin(), new_ll.end() );
                        new_ll.erase( unique( new_ll.begin(), new_ll.end() ), new_ll.end() );

                        // Update permitted vehicles list (new_plt)...
                        new_plt.push_back(*it);
                    }
                    else {
                        continue;
                    }
                }
                else {
                    break;
                }
            }

            for (vector<int>::iterator it = new_plt.begin() ; it != new_plt.end(); ++it){
                rl.erase(*it);
                vector<int>::iterator index = find(rp.begin(), rp.end(), *it);
                vector<int>::iterator t = rp.begin() + *index;
                rp.erase(index);
            }

            // Setting new vectors to global ones...
            plt = new_plt;
            pll = new_pll;
            ll = new_ll;

            // Send plt list as a PERMIT message...
            t_channel channel = dataOnSch ? type_SCH : type_CCH;
            WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
            wsm->setSenderAddress(9999);
            ostringstream oss;
            if (!plt.empty()){
              std::copy(plt.begin(), plt.end()-1, ostream_iterator<int>(oss, "$"));
              oss << plt.back();
            }
            wsm->setLane(oss.str().c_str());
            wsm->setPsid(PERMIT);
            sendWSM(wsm);
            count_RELEASE = 0;
        }
    }
	findHost()->getDisplayString().updateWith("r=16,green");

	annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobi->getCurrentPosition(), "blue"));

	//if (!sentMessage) sendMessage(wsm->getWsmData());
}

void VeinsRSU::sendMessage(std::string blockedRoadId) {
	sentMessage = true;
	t_channel channel = dataOnSch ? type_SCH : type_CCH;
	WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
	wsm->setWsmData(blockedRoadId.c_str());
	sendWSM(wsm);
}

void VeinsRSU::sendWSM(WaveShortMessage* wsm) {
	sendDelayedDown(wsm,individualOffset);
}

// Newly Added Functions...
void VeinsRSU::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            WaveShortMessage* rsu_beacon = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
            rsu_beacon->setSenderAddress(9999);
            rsu_beacon->setPsc("RSU");
            sendWSM(rsu_beacon);
            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
            break;
        }
        default: {
            if (msg)
                DBG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

std::vector<int> VeinsRSU::lsl(int lane){
    switch (lane){
    case 0:{
        int arr[] = {2,5,6,7};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 1:{
        int arr[] = {2,3,4,7};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 2:{
        int arr[] = {0,1,4,7};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 3:{
        int arr[] = {1,4,5,6};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 4:{
        int arr[] = {1,2,3,6};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 5:{
        int arr[] = {0,3,6,7};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 6:{
        int arr[] = {0,3,4,5};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    case 7:{
        int arr[] = {0,1,2,5};
        std::vector<int> vector(arr,arr+4);
        return vector;
    }
    default:{
        int arr[] = {0,1,2,3,4,5,6,7};
        std::vector<int> vector(arr,arr+8);
        return vector;
    }
    }
}

int VeinsRSU::laneNumber(string lane){
    if(strcmp(lane.c_str(), "L0_0") == 0)
        return 0;
    else if(strcmp(lane.c_str(), "L0_1") == 0)
        return 1;
    else if(strcmp(lane.c_str(), "L1_0") == 0)
        return 2;
    else if(strcmp(lane.c_str(), "L1_1") == 0)
        return 3;
    else if(strcmp(lane.c_str(), "L2_0") == 0)
        return 4;
    else if(strcmp(lane.c_str(), "L2_1") == 0)
        return 5;
    else if(strcmp(lane.c_str(), "L3_0") == 0)
        return 6;
    else if(strcmp(lane.c_str(), "L3_1") == 0)
        return 7;
    else
        return -1;
}
