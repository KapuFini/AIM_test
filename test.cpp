#include <iostream>
#include <libsumo/libtraci.h>
#include <set>
#include <utility>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <queue>
#include <cassert>
#include <algorithm>
#include <functional>
#include "traffic/Car.h"
#include "traffic/RoadController.h"
#include "traffic/Overall.h"
using namespace libtraci;
std::ofstream outputfile("num_of_v_distance.txt");
std::set<std::string> arrived;
int main(int argc, char* argv[]) {
    Overall oa=Overall("data/osm.net.xml");
    arrived=std::set<std::string>();
    long long throughput=0;
    Simulation::start({"sumo-gui", "-n", "data/osm.net.xml","-r","data/osm.rou.xml","--route-steps","1000","--step-length","0.1"});
    //Simulation::loadState("save.xml");
    for (int i = 0; i < 10000; i++) {

        Simulation::step();
        outputfile<<Simulation::getCurrentTime()<<" : "<<Vehicle::getIDList().size();
        outputfile<<"  ,Departed  "<<Simulation::getDepartedNumber();
        outputfile<<"  ,Arrived  "<<arrived.size()<<std::endl;
        //if(i==500)Simulation::loadState("save.xml");
        //std::cout<<Simulation::getStopEndingVehiclesNumber()<<"    "<<arrived.size()<<std::endl;
        //到着したらtrueで保存
        //route();
        //for(std::string id:Simulation::getArrivedIDList()){arrived.insert(id);
            //std::cout<<Vehicle::getStopArrivalDelay(id)<<std::endl;
        //}
        oa.step(false);
        for(std::string id:Vehicle::getIDList()){
            int num = atoi(id.c_str());
            std::string road=Vehicle::getRoadID(id);

            


            //std::cout<<(history[id]==road)<<history[id]<<"   "<<road<<std::endl;
            /*
            if(history[id].size()>0&&road[0]!=':'&&history[id]!=conv(road)){throughput++;}
            if(history[id]==road){
                delay_v[id]++;
            }else{
                delay_v[id]=0;
            }
            history[id]=(road);
            */
            
        }
    }
    oa.PrintDelay();
    cout<<"Throughput"<<oa.getThroughput()<<endl;
    std::cout<<Vehicle::getIDList().size()<<" Arrived"<<oa.getArrived()<<"  "<<"throughput: "<<throughput<<std::endl;
    /*
    while(Vehicle::getIDList().size()>0){
        Simulation::step();
        for(std::string id:Simulation::getDepartedIDList()){
            Vehicle::remove(id);
        }
        for(std::string id:Vehicle::getIDList()){

            std::string road=Vehicle::getRoadID(id);
            std::string Before=history[id];
            std::string before=conv(Before);
            if(before==road){
                delay_v[id]++;
            }else{
                delaysum_i[before]+=std::max(0.0,delay_v[id]-length[Before]/speed[Before]);
                num_i[before]++;
                delay_v[id]=0;
            }
            history[id]=(road);
            
        }
    }
    double delay=0;
    
    */
    /*
    for(std::pair<std::string,double> p:delaysum_i){
        std::string id=p.first;
        if(num_i[id]!=0)delay+=p.second/num_i[id];
        //std::cout<<id<<" "<<delaysum_i[id]<<" "<<num_i[id]<<std::endl;
        //if(num_i[id]!=0)delay+=delaysum_i[id]/num_i[id];
    }
    */
    //std::cout<<"delay: "<<delay<<std::endl;
    Simulation::close();
}