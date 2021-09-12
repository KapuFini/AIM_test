
#include <string>
#include <vector>
#include <map>
#include <cfloat>
#include <queue>
#include <libsumo/libtraci.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <iostream>
#include <microsim/MSVehicleType.h>
#pragma once
using namespace std;
class Car;
class RoadController;
class Overall{
private:
    map<string,vector<string> > edge_lane;
    map<string,Car*> Cars;
    map<string,string> Car_RC;
    map<string,string > lane_edge;
    map<string,double> delays;
    map<string,double> DepartTimes;
    map<string,double> ArriveTimes;
    map<string,vector<pair<double,double> > > lane_shape;
    map<string,double> lane_speed;
    map<string,vector<string> > junction_lane;
    map<string,string> lane_junction;
    map<string,pair<string,string> > via_fromto;
    map<string,map<string,string> > fromto_via;
    map<string,RoadController*> RCs;
    map<string,string> edge_junction;
    int throughput;
    int arrived;
public:
    Overall(string netfile);
    ~Overall();
    //map<string,RoadController> RoadControllers;
    map<string,double> getLaneSpeed();
    void step(bool timebase);
    void iterative_step();
    int getThroughput();
    int getArrived();
    void reroute(string vid,bool iterative);
    void PrintDelay();
    //void setRCs(string id,RoadController rc);
};