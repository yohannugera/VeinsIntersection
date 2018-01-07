#ifndef VeinsRSU_H
#define VeinsRSU_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/world/annotations/AnnotationManager.h"
#include <string.h>
#include <map>

using Veins::AnnotationManager;
using namespace std;

/**
 * Small RSU Demo using 11p
 */
class VeinsRSU : public BaseWaveApplLayer {
	public:
		virtual void initialize(int stage);
	protected:
		AnnotationManager* annotations;
		BaseMobility* mobi;
		bool sentMessage;

		// newly added..
		map<int,int> rl;
		vector<int> rp;
		vector<int> plt;
		vector<int> pll;
		map<int, vector<int>> permitted;
		vector<int> ll;
		int count_RELEASE;

	protected:
		virtual void onBeacon(WaveShortMessage* wsm);
		virtual void onData(WaveShortMessage* wsm);
		void sendMessage(std::string blockedRoadId);
		virtual void sendWSM(WaveShortMessage* wsm);
		virtual void handleSelfMsg(cMessage* msg) override;
		vector< int > lsl(int lane);
		int laneNumber(string lane);
};

#endif
