//
// Demo VTL class for the Cyber Physical Systems course
//

#ifndef __VEINS_VIRTUALTRAFFICLIGHTS_H_
#define __VEINS_VIRTUALTRAFFICLIGHTS_H_

#include <omnetpp.h>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/VTLMessage_m.h"

using namespace omnetpp;

namespace veins {


class VEINS_API VirtualTrafficLights : public DemoBaseApplLayer {
    public:
        ~VirtualTrafficLights();
        virtual void initialize(int stage);
        virtual void finish();
    protected:
        virtual void onBSM(DemoSafetyMessage* bsm);
        virtual void onWSM(BaseFrame1609_4* wsm);
        virtual void onWSA(DemoServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);

        short int TRAFFIC_CHECK = 176;
        cMessage * traffic_check;
    private:
        int crashedCars;
        int passedCars;
        double simulationDuration;
    };

    struct CarStatus {
        LAddress::L2Type address;
        std::string preferredRoad1;
        std::string preferredRoad2;
        Coord position;
        double dist;
        Coord speed;
        simtime_t time_received;
        int leaderRank;
        std::string roadID;
        double lanePosition;
        long nextCars[4];
//        std::string prefferedRoads[2];
    };

    std::map<double, CarStatus> CarStatuses;
} // namespace veins
#endif
