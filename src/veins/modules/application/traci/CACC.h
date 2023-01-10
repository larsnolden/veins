//
// Demo CACC class for the Cyber Physical Systems course
//

#ifndef __VEINS_CACC_H_
#define __VEINS_CACC_H_

#include <omnetpp.h>
#include "veins/veins.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/CACCMessage_m.h"

using namespace omnetpp;

namespace veins {

class VEINS_API CACC : public DemoBaseApplLayer {
    public:
        ~CACC();
        virtual void initialize(int stage);
        virtual void finish();

    protected:
        virtual void onBSM(DemoSafetyMessage* bsm);
        virtual void onWSM(BaseFrame1609_4* wsm);
        virtual void onWSA(DemoServiceAdvertisment* wsa);
        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);
	
	private:
		short int TRAFFIC_CHECK = 176; //arbitrary type number
        cMessage * traffic_check;

    };

    struct CarStatusUpdate {
        double address;
        Coord position;
        double dist;
        Coord speed;
        simtime_t time_received;
    };

    std::map<double, CarStatusUpdate> CarStatusUpdates;

//    WATCH(CarStatusUpdates.size());
//    EV << CarStatusUpdates.size() << endl;


} // namespace veins
#endif
