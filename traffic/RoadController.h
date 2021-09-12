#include <string>
#include <vector>
#include <map>
#include <cfloat>
#include <libsumo/libtraci.h>
#include <stdio.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <iostream>
#include<fstream>
#include <tuple>
#include <microsim/MSVehicleType.h>
#pragma once
//#include "Car.h"

//#include <libsumo/libtraci.h>
using namespace std;
class Scheduler;
class Car;
class RoadController{
private:
    string ID;
    vector<Car*> on_road_car;
    vector<Car*> enter_car;
    //map<string,double> timeouts;
    //vector<vector<map<int,string> > > tile;
    map<string,double> car_enter_time;
    //map<string,vector<vector<vector<double> > > > car_tile;
    map<string,vector<pair<double,double> > > shape;
    vector<string> lanes;
    vector<string> edges;
    map<int,vector<double> > crossed_travelTime;
    map<string,map<string,string>> edge_via;
    map<int,vector<Car* > > waiting_dispatch;
    double N;
    double S;
    double W;
    double E;
    int height;
    int width;
    double TAV;
    double PAV;
    double oPAV;
    double TWA;
    Scheduler* sc;
public:
    RoadController(string id);
    ~RoadController();
    //void Schedule();
    //bool Al2(string v_id,double T_A,double vel,bool change,double ACCEL);
    //pair<int,double> Al1(int M,Car c,double t_i);
    void setLaneVia(map<string,map<string,string> > l_v);
    void setLane(vector<string> lanes);
    void setShape(map<string,vector<pair<double,double> > > Shape);
    void pushCar(Car* c);
    vector<string> iterative_carlist(int step);
    vector<Car*>* carlist();
    void setEdge(vector<string> edges);
    bool before(string v_id);
    void erase_Car(Car* c);
    string getID();
    void step(int T);
    double getTraversalTime();
    void setPAV(bool tmp);
    void setoPAV();
    void setTWA();
    void dispatch_car_from_waiting(int step);
    void assigned_car(int schedule_step,Car* c,int OPTP);
    void optimize(int step);
    void printenter();
    void iterative_optimize(int step);
};