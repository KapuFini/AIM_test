#include <string>
#include <vector>
#include <map>
#include <cfloat>
#include <algorithm>
#include <libsumo/libtraci.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <iostream>
#include <microsim/MSVehicleType.h>
#pragma once
//#include "RoadController.h"

//#include <libsumo/libtraci.h>
using namespace std;
class RoadController;



class Car{
    string ID;
    vector<string> route;
    bool optimistic;
    bool reservation;
    bool accept;
    double accel;
    double decel;
    double arrivaltime;
    double OPTP;
    string typeID;
    double delay;
    double begin;
    RoadController *rc;
public:
    Car(string id,RoadController *_rc);
    Car(string id,RoadController *_rc,vector<string> rou);
    ~Car(void);
    void setRoute(vector<string> rou);
    //oid Al3(RoadController rc);
    //void Al3();
    //double opti();
    //double pess();
    string getID();
    void setDelay(int delay);
    //double getArrivalTime();
    double getAccel();
    double getDecel();
    void send(int m,double t);
    double getDelay();
    bool getReservation();
    double Arrivaltime();
    void step(int t);
    void setEnterTime(double T);
    void setOPTP(double OPTP);
    void setbegin(double _begin);
};

