//
// Demo car tracking class for the Cyber Physical Systems course
//

#ifndef __VEINS_CARTRACKING_H_
#define __VEINS_CARTRACKING_H_

#include <omnetpp.h>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/CarTrackingMessage_m.h"

using namespace omnetpp;

namespace veins {

class VEINS_API CarTracking : public DemoBaseApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void finish();
    protected:
        virtual void onBSM(DemoSafetyMessage* bsm);
        virtual void onWSM(BaseFrame1609_4* wsm);
        virtual void onWSA(DemoServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);

    };
} // namespace veins
#endif
