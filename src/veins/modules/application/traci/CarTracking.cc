//
// Demo car tracking class for the Cyber Physical Systems course
//

#include "./CarTracking.h"
#include "veins/base/utils/Heading.h"


using namespace veins;

Define_Module(veins::CarTracking);

void CarTracking::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;
    }
    else if (stage == 1) {
        //Initializing members that require initialized other modules goes here

    }
}

void CarTracking::finish() {
    DemoBaseApplLayer::finish();
    //statistics recording goes here
}

struct CarStatusUpdate {
    double dist;
    double address;
    double speed;
    double heading_rad;
    simtime_t time_received;
};

CarStatusUpdate Status[] = {};

void CarTracking::onBSM(DemoSafetyMessage* bsm) {
    //Your application has received a beacon message from another car or RSU
    //code for handling the message goes here
    if (CarTrackingMessage* ctm = dynamic_cast<CarTrackingMessage*>(bsm)) { 
            // check for and handle CarTrackingMessage
            double dist = fabs( curPosition.x - ctm->getSenderPos().x );
            double address = fabs( ctm->getSenderAddress());
            double speed = mobility->getSpeed();
            Heading heading = mobility->getHeading();
            double heading_rad = heading.getRad();
            simtime_t time_received = simTime();
            EV << "Received message from " << ctm->getSenderAddress() << " | distance: " << dist << " meters." << " speed: " << speed << " heading: " << heading_rad << endl;

//            update CarStatusUpdate  = new CarStutsUpdate();
//            update.dist = dist;
//            update.adddress = address;
//            update.speed = speed;
//            update.heading_rad = heading_rad;
//            update.time_receeived = time_received;
        }


}

void CarTracking::onWSM(BaseFrame1609_4* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples
	//Can remain empty for the CPS CAD assignments
}

void CarTracking::onWSA(DemoServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples
	//Can remain empty for the CPS CAD assignments
}

void CarTracking::handleSelfMsg(cMessage* msg) {
    // overwrite beacon behavior so we can add data to it!
    // This beacon event is initially scheduled in DemoBaseApplLayer, but we have to reschedule it as we are handling it ourselves now!
    if (msg->getKind() == SEND_BEACON_EVT) { // check for and handle SEND_BEACON_EVT
        CarTrackingMessage* ctm = new CarTrackingMessage(); // make a new carTracking message
        populateWSM(ctm); // auto fill all the default fields from a DemoSafetyMessage
        ctm->setSenderAddress(myId); // set our id as the sender!

        sendDown(ctm); // give the packet to the lower layer for transmission
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt); // schedule the next transmission in beaconInterval seconds
        EV << "Transmitted beacon!" << endl;
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg); // let the BaseWaveApplLayer handle all other events
    //this method is for self messages (mostly timers)
    //it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
}

void CarTracking::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
	// Is called by 'external' code outside of our control. Output won't show in the simulator.
	// Easier to not put your own code here.
}
