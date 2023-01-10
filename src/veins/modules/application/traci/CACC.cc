//
// Demo CACC class for the Cyber Physical Systems course
//
#include "CACC.h"
#include <map>
#include <string>


using namespace veins;

Define_Module(veins::CACC);

void CACC::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << " with id: " << myId << std::endl;

        // DO NOT REMOVE
        traciVehicle->setSpeedMode( 0 ); //disables sumo speed control, this assignment does not work correctly without it!
        traciVehicle->setSpeed( traciVehicle->getSpeed() ); // fix spawning speed to prevent automatic braking.
        // Do whatever you want below here

        // schedule traffic check
        traffic_check = new cMessage();
        traffic_check->setKind(TRAFFIC_CHECK);
        scheduleAt(simTime()+0.5, traffic_check); // schedule in 0.5 seconds

    }
    else if (stage == 1) {
        //Initializing members that require initialized other modules goes here

    }
}

CACC::~CACC(){
    EV << "Killed!" << std::endl;
    cancelAndDelete(traffic_check); //remove remaining traffic_check timers (if any exist)
}

void CACC::finish() {
    DemoBaseApplLayer::finish();
    //statistics recording goes here

}

void CACC::onBSM(DemoSafetyMessage* bsm) {

    if (CACCMessage* ccm = dynamic_cast<CACCMessage*>(bsm)){
          EV << "Receveived caccMessage from " << ccm->getSenderAddress() <<" @ pos: " << ccm->getSenderPos() << " with speed " << ccm->getSenderSpeed() << std::endl;
          // check for and handle CarTrackingMessage
          double dist = fabs( ccm->getSenderPos().x - curPosition.x );
          Coord position = ccm->getSenderPos();
          double address = fabs( ccm->getSenderAddress());
          Coord speed = ccm->getSenderSpeed();
          simtime_t time_received = simTime();
          EV << "Received message from " << ccm->getSenderAddress() << " | distance: " << dist << " meters." << " speed: " << speed << " heading: " << endl;

          double own_address = myId;
          if(address != own_address) {
              // do not save own messages
              CarStatusUpdate update = { address, position, dist, speed, time_received };
              CarStatusUpdates[address] = update;
          }
          EV << "Size of car message struct: " << CarStatusUpdates.size() << endl;

          //  The car's are stuck in the simulation and report a distance of 0. Even though own messages should be filtered already. (see above)

          // check for all cars in front (higher x pos than this car
          if(CarStatusUpdates.size() > 1) {
              CarStatusUpdate closestCar = CarStatusUpdates[CarStatusUpdates.begin()->first]; // closest car found, initialize with the first car in the map
              for(auto& entry: CarStatusUpdates)
              {
                  CarStatusUpdate otherCar = entry.second;
                  if(otherCar.dist > 0) { // car is in front
                      if(otherCar.dist < closestCar.dist) {
                          // take the closest one of those
                          closestCar = otherCar;
                      }
                  }
              }

              EV << "Closest distance: " << closestCar.dist << endl;

              // check if distance to that car is above our desired distance
              if(closestCar.dist > 10) {
                  // if false: increase speed above the speed of the other car
                  double speedupFactor = 1.1;
                  double desiredSpeed = closestCar.speed.x * speedupFactor;
                  EV << "Decreasing distance, speed set to: " << desiredSpeed << endl;
                  traciVehicle->setSpeed(desiredSpeed);
              } else {
                  // if true: set speed the same speed as this car
                  traciVehicle->setSpeed(closestCar.speed.x);
                  EV << "Keeping distance, speed set to: " << closestCar.speed.x << endl;
              }
          }
     }
}

void CACC::onWSM(BaseFrame1609_4* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void CACC::onWSA(DemoServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples

}

void CACC::handleSelfMsg(cMessage* msg) {
    if ( msg->getKind() == TRAFFIC_CHECK ){
        EV << "TRAFFIC_CHECK triggered!" << std::endl;

        traffic_check = msg->dup();
        scheduleAt(simTime()+0.5, traffic_check);
        return;
    } else if (msg->getKind() == SEND_BEACON_EVT) { // overwrite beacon behavior
        CACCMessage* bsm = new CACCMessage();
        populateWSM(bsm);
        bsm->setSenderAddress(myId);

        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        return;
    }

    EV << "handling self msg.." << std::endl;
    DemoBaseApplLayer::handleSelfMsg(msg);
    //this method is for self messages (mostly timers)
    //it is important to call the DemoBaseApplLayer function for BSM and WSM transmission

}

void CACC::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
    // Is called by 'external' code outside of our control. Output won't show in the simulator.
	// Easier to not put your own code here.
}
