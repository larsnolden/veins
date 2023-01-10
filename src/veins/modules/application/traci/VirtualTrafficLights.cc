//
// Demo VTL class for the Cyber Physical Systems course
//

#include "VirtualTrafficLights.h"
#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "veins/veins.h"


using namespace veins;

Define_Module(veins::VirtualTrafficLights);

CarStatus getLeader(LAddress::L2Type ownId) {
    if(CarStatuses.size() > 1) {
        CarStatus leaderCar = CarStatuses[CarStatuses.begin()->first];
        for(auto& entry: CarStatuses)
        {
            CarStatus otherCar = entry.second;
            if(otherCar.leaderRank > leaderCar.leaderRank) {
                // make the car with the lowest leaderId the leader
                leaderCar = otherCar;
            }
        }
        //        EV << "Leader Car " << leaderCar.address << endl;
        return leaderCar;
    }
    // return own Id in case there are no other cars yet
    LAddress::L2Type address = ownId;
    // also make sure that the own car is added to CarStatuses not sure if that is done rn?
    std::string preferredRoad1 = "none";
    std::string preferredRoad2 = "none";
    CarStatus self = {
            address,
            preferredRoad1,
            preferredRoad2
    };
    return self;
}

int countCarsBeforeInteresection(std::string roadId) {
    int count = 0;
    for(auto& entry: CarStatuses)
    {
        CarStatus car = entry.second;
        //        EV << "RoadId " << car.roadID << endl;
        if(car.roadID == roadId) {
            count++;
        }
    }
    return count;
}

void UpdateCarControl (CarStatus leader, bool isLeader, TraCICommandInterface::Vehicle* traciVehicle, double ownAddressDouble) {
    // For unknown reasons the preferred Roads is empty when printing
    std::string currentRoadId = traciVehicle->getRoadId().c_str();
    double position = traciVehicle->getLanePosition();
    EV << "ownAddress: " << ownAddressDouble << "\n" << std::endl;
    EV << "nextCar0: " << leader.nextCars[0] << "nextCar1: " << leader.nextCars[1] << "nextCar2: " << leader.nextCars[2] << "nextCar3: " << leader.nextCars[3] << "\n" << std::endl;
    long ownAddress = (long) ownAddressDouble;

    bool isAfterIntersection = currentRoadId.find("1to") != std::string::npos;
    bool isBeforeIntersection = (currentRoadId.find("to1") != std::string::npos) || (currentRoadId.find("_1") != std::string::npos) ;
    bool isAfterIntersection = (currentRoadId.find("1to") != std::string::npos);
    bool isBeforeIntersection = (currentRoadId.find("to1") != std::string::npos);
//
//
//    //    testing whether we should be leader or whether we have already passed the intersection and we need to find another leader.
//   if(isLeader && isAfterIntersection){
//
//   }
//
//





    bool isNextCar;
        long *nextCar = std::find(std::begin(leader.nextCars), std::end(leader.nextCars), ownAddressDouble);
        if(nextCar != std::end(leader.nextCars)) {
            isNextCar = true;
            traciVehicle->setColor(veins::TraCIColor::fromTkColor("blue"));
        } else {
            isNextCar = false;
            traciVehicle->setColor(veins::TraCIColor::fromTkColor("red"));
        }





    EV << "currentRoadId: " << currentRoadId << std::endl;

    if(isBeforeIntersection) {
        EV << "currentRoadId before intersection: " << currentRoadId << std::endl;
        if(isNextCar) {
            traciVehicle->setColor(veins::TraCIColor::fromTkColor("green"));
            if(traciVehicle->isStopReached()) {
                traciVehicle->resume();

            }
        } else {
            traciVehicle->setColor(veins::TraCIColor::fromTkColor("red"));
            traciVehicle->stopAt(currentRoadId, 195,  traciVehicle->getLaneIndex(), 0, 1);
        }
    } else {
        // car is past intersection and of no concern
        traciVehicle->setColor(veins::TraCIColor::fromTkColor("pink"));

        if(isLeader){
            // tried to add this, lars can do better.

            if(CarStatuses.size() > 1) {

                  CarStatuses.erase(0);
                  traciVehicle->setColor(veins::TraCIColor::fromTkColor("yellow"));

              }




        }

        if(traciVehicle->isStopReached()) {
            traciVehicle->resume();

        }
    }





}

std::vector<CarStatus> getCarsOnRoad(std::string roadId) {
    std::vector< CarStatus > cars;
    for(auto& entry: CarStatuses)
    {
        CarStatus car = entry.second;
        if(car.roadID == roadId) {
            cars.push_back(car);
        }
    }
    return cars;
}

bool compareCarPosition(const CarStatus &a, const CarStatus &b) {
    return a.lanePosition > b.lanePosition;
}

std::vector< CarStatus > getNextCars(std::string roadId, int maxCount) {
    std::vector< CarStatus > cars = getCarsOnRoad(roadId);
    // sort cars by distance to intersection
    std::sort(cars.begin(), cars.end(), compareCarPosition);
    // only take the closes #maxCount cars
    cars.resize(maxCount);
    return cars;
}

std::string safeRoad1 = "1_5";
std::string safeRoad2 = "1_4";
std::string safeRoad3 = "1_3";
std::string safeRoad4 = "1_2";

bool getNextCarsHavePassed(long nextCars[4]) {
    bool allPassed[] = {false, false, false, false};
    for(auto& entry: CarStatuses)
    {
        for (int i = 0; i < 4; i++) {
            CarStatus car = entry.second;
            EV << "nextCar[i]: " <<  nextCars[i] << "car.address" << car.address << std::endl;

            if(car.address == nextCars[i]) {
                EV << "nextCar found for check: " <<  nextCars[i] << std::endl;
                //                bool isOnSafeRoad = (car.roadID == safeRoad1 || car,roadId == safeRoad2 || );
                //                bool isAfterIntersection = (car.roadID.find("1_") != std::string::npos) || (car.roadID.find("1to") != std::string::npos);
                EV << "nextCar RoadId: " <<  car.roadID << std::endl;
                bool isAfterIntersection = (car.roadID.find("1to") != std::string::npos) ||  (car.roadID.find("1_") != std::string::npos);
                bool isBeforeIntersection = (car.roadID.find("to1") != std::string::npos);
                if(!isBeforeIntersection && car.lanePosition > 20) {
                    allPassed[i] = true;
                    EV << "car passed " << std::endl;

                } else {
                    EV << "car not passed " << std::endl;
                }
            }
        }
    }
    bool allPassedS = (std::all_of(
            std::begin(allPassed),
            std::end(allPassed),
            [](bool i)
                  {
                    return i; // or return !i ;
                  }
      ));
    return allPassedS;
}

void VirtualTrafficLights::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        //Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << " with id: " << myId << std::endl;

        // DO NOT REMOVE
        traciVehicle->setSpeedMode( 0 ); //disables right of way check at junctions!
        traciVehicle->setColor(veins::TraCIColor::fromTkColor("pink")); // we start as a blue car because reasons
        // Do whatever you want below here

        // schedule traffic check
        traffic_check = new cMessage();
        traffic_check->setKind(TRAFFIC_CHECK);
        scheduleAt(simTime()+0.5, traffic_check);

        // tip: you can schedule as many messages of any type you like!
    }
    else if (stage == 1) {
        //Initializing members that require initialized other modules goes here

    }
}

VirtualTrafficLights::~VirtualTrafficLights(){
    cancelAndDelete(traffic_check); //remove remaining traffic_check timers (if any exist)
}

void VirtualTrafficLights::finish() {
    DemoBaseApplLayer::finish();
    //statistics recording goes here
    recordScalar("carsPassed", passedCars);
    EV<< passedCars << std::endl;

}

void VirtualTrafficLights::onBSM(DemoSafetyMessage* bsm) {
    if (VTLMessage* vtlm = dynamic_cast<VTLMessage*>(bsm)) {
        //        EV << "Receveived cavtlmessage from " << vtlm->getSenderAddress() <<" @ pos: " << vtlm->getSenderPos() << " with speed " << vtlm->getSenderSpeed() << std::endl;
        // check for and handle CarTrackingMessage
        double dist = fabs( vtlm->getSenderPos().x - curPosition.x );
        Coord position = vtlm->getSenderPos();
        LAddress::L2Type address = vtlm->getSenderAddress();
        Coord speed = vtlm->getSenderSpeed();
        simtime_t time_received = simTime();
        int leaderRank = fabs(vtlm->getSenderLeaderRank());
        double lanePosition = vtlm->getLanePosition();
        std::string roadID = vtlm -> getRoadID();
        std::string preferredRoad1 = vtlm->getSenderPreferredRoad1();
        std::string preferredRoad2 = vtlm->getSenderPreferredRoad2();

        double ownAddress = myId;
        if(ownAddress ==  vtlm->getNextCars(0) || ownAddress ==  vtlm->getNextCars(1) || ownAddress ==  vtlm->getNextCars(2) || ownAddress ==  vtlm->getNextCars(3)) {
                // traciVehicle->setColor(veins::TraCIColor::fromTkColor("blue"));

        }

            // do not save own messages
            //            CarStatus update = { address, position, dist, speed, time_received, leaderRank, roadID, prefferedRoads[0]=vtlm->getSenderPreferredRoads(0), prefferedRoads[1]=vtlm->getSenderPreferredRoads(1) };
            CarStatus update = { address, preferredRoad1, preferredRoad2, position, dist, speed, time_received, leaderRank, roadID, lanePosition, vtlm->getNextCars(0), vtlm->getNextCars(1), vtlm->getNextCars(2), vtlm->getNextCars(3)};
            //            EV << "update car preferredRoad1 " << update.preferredRoad1 << std::endl;
            CarStatuses[address] = update;

    }
}

void VirtualTrafficLights::onWSM(BaseFrame1609_4* wsm) {
    //Your application has received a data message from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples
}

void VirtualTrafficLights::onWSA(DemoServiceAdvertisment* wsa) {
    //Your application has received a service advertisement from another car or RSU
    //code for handling the message goes here, see TraciDemo11p.cc for examples
}

void VirtualTrafficLights::handleSelfMsg(cMessage* msg) {
    CarStatus leader = getLeader(myId);
    if ( msg->getKind() == TRAFFIC_CHECK ){

        //  check if we are leader before sending a leader message
        bool isLeader = leader.address == myId;




        // maybe we need to implement some logic here to revoke the leadership?

//
//        bool isAfterIntersection = (currentRoadId.find("1to") != std::string::npos);
//
//        if (isLeader && isAfterIntersection){
//            CarStatus leader = CarStatuses[CarStatuses.begin()->second];
//            isLeader=false;
//        }

        UpdateCarControl(leader, isLeader, traciVehicle, myId);
        traffic_check = msg->dup();
        scheduleAt(simTime()+0.5, traffic_check);
        return;
    } else if (msg->getKind() == SEND_BEACON_EVT) { // overwrite beacon behaviour
        VTLMessage* vtlm = new VTLMessage();
        populateWSM(vtlm);
        vtlm->setSenderAddress(myId);
        vtlm->setRoadID(traciVehicle->getRoadId().c_str());
        vtlm->setLanePosition(traciVehicle->getLanePosition());

        bool nextCarsHavePassed = getNextCarsHavePassed(leader.nextCars);
        EV << "nextCarsHavePassed: " << nextCarsHavePassed << "\n" << std::endl;

        // generate a random leaderID
        // the car with the lowest leaderID will be the elected leader
        std::string roadId = vtlm->getRoadID();
        EV << "roadId::: " << roadId << "\n" << std::endl;

        bool isAfterIntersection = (roadId.find("1to") != std::string::npos) ||  (roadId.find("1_") != std::string::npos);
        bool isFarAfterIntersection = vtlm->getLanePosition() > 80;

        if((nextCarsHavePassed || vtlm->getNextCars(0) == 0) && leader.address != myId) {
//            int randomLeaderRank = rand();
//            vtlm->setSenderLeaderRank(randomLeaderRank);
//            CarStatus leader = getLeader(randomLeaderRank);
            passedCars += 4;
            int leaderRank = leader.leaderRank +1;
            vtlm->setSenderLeaderRank(leaderRank);
            std::string roadIdVt1 = "5to1";
            std::string roadIdHz1 = "4to1";
            std::string roadIdVt2 = "3to1";
            std::string roadIdHz2 = "2to1";

            int carsHz = countCarsBeforeInteresection(roadIdHz1) + countCarsBeforeInteresection(roadIdHz2);
            int carsVt = countCarsBeforeInteresection(roadIdVt1) + countCarsBeforeInteresection(roadIdVt2);


            // for now just select the next 4 cars of the preffered lane to go
            std::vector< CarStatus > nextCars;
            std::vector< CarStatus > nextCars1;
            std::vector< CarStatus > nextCars2;
            if(carsHz >= carsVt) {
                //  prefer cars Hz
                nextCars1 = getNextCars(roadIdHz1, 2);
                nextCars2 = getNextCars(roadIdHz2, 2);
                nextCars.insert(nextCars.end(), nextCars1.begin(), nextCars1.end());
                nextCars.insert(nextCars.end(), nextCars2.begin(), nextCars2.end());
            } else {
                //  prefer cars Vt
                nextCars1 = getNextCars(roadIdVt1, 2);
                nextCars2 = getNextCars(roadIdVt2, 2);
                nextCars.insert(nextCars.end(), nextCars1.begin(), nextCars1.end());
                nextCars.insert(nextCars.end(), nextCars2.begin(), nextCars2.end());
            }
            //        EV << "nextCars[0]" << nextCars[0].address << std::endl;
            //long id = (&nextCars[0])->address; How to copy value from nested pointer
            EV << "nextCars[0]" << nextCars[0].address << "nextCars[1]" << nextCars[1].address << "nextCars[2]" << nextCars[2].address << "nextCars[3]" << nextCars[3].address << std::endl;
            vtlm->setNextCars(0, nextCars[0].address);
            vtlm->setNextCars(1, nextCars[1].address);
            vtlm->setNextCars(2, nextCars[2].address);
            vtlm->setNextCars(3, nextCars[3].address);
        } else {
            // keep current leader

            long test = vtlm->getNextCars(0);
            EV << "vtlm->getNExtCars" << vtlm->getNextCars(0) << std::endl;
            //
            //            vtlm->setNextCars(0, vtlm->getNextCars(0));
            //            vtlm->setNextCars(1, vtlm->getNextCars(1));
            //            vtlm->setNextCars(2, vtlm->getNextCars(2));
            //            vtlm->setNextCars(3, vtlm->getNextCars(3));
            //            vtlm->setSenderLeaderRank(vtlm->getSenderLeaderRank());
            int leaderRank = 1;
            vtlm->setSenderLeaderRank(leaderRank);
        }

        sendDown(vtlm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        return;
    }

    //    EV << "handling self msg.." << std::endl;
    DemoBaseApplLayer::handleSelfMsg(msg);

    //this method is for self messages (mostly timers)
    //it is important to call the DemoBaseApplLayer function for BSM and WSM transmission

}

void VirtualTrafficLights::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
    // Is called by 'external' code outside of our control. Output won't show in the simulator.
    // Easier to not put your own code here.

}
