#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <cfloat>
#include <libsumo/libtraci.h>
#include <stdio.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <utility>
#include <iostream>
#include <fstream>
#include <microsim/MSVehicleType.h>
#include <math.h>
#include <tuple>

#pragma once
//#include "Car.h"

//#include <libsumo/libtraci.h>
using namespace std;
class Car;
class Scheduler{
private:
    double N;
    double S;
    double W;
    double E;
    int height;
    int width;
    vector<vector<map<int,string> > > tile;
    vector<vector<map<int,string>>> ConflictPoint;
    vector<vector<map<int,string>>> ConflictPointTemp;
    
    map<string,vector<pair<double,double> > > shape;
    double __OPTP;
    double SimulationStepLength ;
    double StepPerSecond;
    double MinimumGap;
    double VehicleLength;
    double LaneDistance;
    double FixedSpeed;
    double __Hmin;
    double __MiniLaneDistance;
    double __MiniIntersectionDistance;
    double __Delta;
    double __TotalDelay;
    int __VehicleNumber;
    map<string,map<string,string> > edge_via;
    map<string,vector<pair<int,double>>> __Lane_Conflict;
    map<string,double> LastVehicleArrivalTime;
    vector<map<int,bool>> __Conflict_Point;
    vector<map<int,bool>> __Conflict_Point_temp;
    
public:
    Scheduler();
    ~Scheduler();
    double QueryTotalDelay();
    void setLaneVia(map<string,map<string,string> > l_v);
    void setShape(map<string,vector<pair<double,double> > > Shape);
    vector<tuple<string,Car*,int,int>> Simulated_Annealing(vector<Car*> comingVehicle,double CurrentTimeStep);
    pair<int,vector<tuple<string,Car*,int>>> GenerateValidSolution(vector<Car*> comingVehicle,double CurrentTimeStep);
    pair<int,vector<tuple<string,Car*,int>>> GenerateValidSolution1(vector<Car*> comingVehicle,double CurrentTimeStep);
    void __Clear_Temp();
    void __Reset(vector<string> carlist,vector<Car*> NCar);
};