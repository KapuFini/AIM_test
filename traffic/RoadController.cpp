#include "RoadController.h"
#include "Car.h"
#include "Scheduler.h"
using namespace std;
ofstream TravelTime("TravelTime.txt");
RoadController::RoadController(string id){
    ID=id;
    on_road_car=vector<Car*>();
    enter_car=vector<Car*>();
    PAV=0;
    sc=new Scheduler();
}
RoadController::~RoadController(){}
//RoadController::Schedule(){}
void RoadController::setLaneVia(map<string,map<string,string> > l_v){
    this->edge_via=l_v;
    sc->setLaneVia(l_v);
    
}
void RoadController::setLane(vector<string> lanes){
    this->lanes=lanes;
    
}
void RoadController::setEdge(vector<string> edges){
    this->edges=edges;
}


void RoadController::pushCar(Car* c){
    //cout<<c->getID()<<": ";for(Car* CAR:on_road_car){cout<<CAR->getID()<<" ";}cout<<endl;
    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    car_enter_time[c->getID()]=libtraci::Simulation::getCurrentTime();
    //cout<<"pushCar  "<<c->getID()<<" "<<libtraci::Simulation::getCurrentTime()<<" "<<endl;
    on_road_car.push_back(c);
    enter_car.push_back(c);

    //cout<<this->getID()<<" :  "<<c->getID()<<":push car ";for(Car* CAR:on_road_car){cout<<CAR->getID()<<" ";}cout<<endl;
}
void RoadController::setShape(map<string,vector<pair<double,double> > > Shape){
    this->shape=Shape;
    E=DBL_MIN;
    W=DBL_MAX;
    N=DBL_MIN;
    S=DBL_MAX;
    //cout<<Shape.size()<<endl;
    for(pair<string,vector<pair<double,double> > > vec:Shape){
        //cout<<vec.first<<": ";
        for(pair<double,double> v:vec.second){
            E=max(E,v.first);
            W=min(W,v.first);
            N=max(N,v.second);
            S=min(S,v.second);
            //cout<<"("<<v.first<<","<<v.second<<"),";
        }
        //cout<<endl;
    }
    
    //cout<<ID<<" E"<<E<<", W"<<W<<", N"<<N<<", S"<<S<<endl;
    height =max(0,(int)((double)((int)N+1-(int)S)/0.5));
    width  =max(0,(int)((double)((int)E+1-(int)W)/0.5));
    //this->lanes=lanes;
    //edge_via=l_v;
    sc->setShape(Shape);

}
vector<Car *>* RoadController::carlist(){
    return &on_road_car;
}

bool RoadController::before(string v_id){
    string Road=libtraci::Vehicle::getRoadID(v_id);
    for(string edge:edges){
        if(edge==Road)return true;
    }
    return false;
}
void RoadController::erase_Car(Car* c){
    //cout<<c.getID()<<endl;
    //cout<<"erase : ";
    //for(int i=0;i<on_road_car.size();i++)cout<<on_road_car[i]->getID()<<" ";
    //cout<<endl;
    //cout<<on_road_car.size()<<endl;
    for(int i=0;i<on_road_car.size();i++){
        
        //cout<<"i:"<<i<<"="<<on_road_car[i]->getID()<<" ";
        if(on_road_car[i]->getID()==c->getID()){
            on_road_car.erase(on_road_car.begin()+i);
            //cout<<c->getID()<<" in "<<ID<<" was deleted"<<endl;
            break;
        }
    }
    for(int i=0;i<enter_car.size();i++){
        //cout<<"i:"<<i<<"="<<on_road_car[i]->getID()<<" ";
        if(enter_car[i]->getID()==c->getID()){
            enter_car.erase(enter_car.begin()+i);
            //cout<<c->getID()<<" in "<<ID<<" was deleted"<<endl;
            break;
        }
    }

    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    crossed_travelTime[stepnum].push_back((libtraci::Simulation::getCurrentTime()-car_enter_time[c->getID()])/1000);
    //cout<<"traveltime   "<<c->getID()<<"  "<<libtraci::Simulation::getCurrentTime()<<"  "<<car_enter_time[c->getID()]<<endl;
    //cout<<on_road_car.size()<<endl;
    //for(int i=0;i<on_road_car.size();i++)cout<<on_road_car[i]->getID()<<" ";
    //cout<<endl;
    if(car_enter_time.find(c->getID())!=car_enter_time.end()){
        car_enter_time.erase(c->getID());
    }
}
string RoadController::getID(){
    return ID;
}
double RoadController::getTraversalTime(){
    TAV=(double)on_road_car.size();
    setoPAV();
    setTWA();
    //cout<<"predictTime:"<<0.09*TAV+0.83*PAV+0.25*oPAV+0.25*TWA+2.26<<endl;
    //cout<<TAV<<" "<<PAV<<" "<<oPAV<<" "<<TWA<<endl;
    return 0.09*TAV+0.83*PAV+0.25*oPAV+0.25*TWA+20;
}
void RoadController::setPAV(bool tmp){
    if(tmp){
        PAV++;
    }else{
        PAV=max(PAV-1,0.0);
    }
}
void RoadController::setoPAV(){
    double a=0;
    double current=libtraci::Simulation::getCurrentTime();
    
    for(Car* c:on_road_car){
        if(car_enter_time.find(c->getID())!=car_enter_time.end()){
            a=max(a,(current-car_enter_time[c->getID()])/1000);
        }
    }
    oPAV=a;
}

void RoadController::step(int T){
    for(Car* c:on_road_car){
        c->step(T);
    }
}
void RoadController::dispatch_car_from_waiting(int step){
    if(waiting_dispatch.find(step)!=waiting_dispatch.end()){
        for(Car* c:waiting_dispatch[step]){
            //pushCar(c);
        }
    }    
}
void RoadController::assigned_car(int schedule_step,Car* c,int OPTP){
    /*
    if(waiting_dispatch.find(schedule_step)==waiting_dispatch.end()){
        waiting_dispatch[schedule_step]=vector<Car*>(0);
    }
    waiting_dispatch[schedule_step].push_back(c);
    */
    TAV=(double)on_road_car.size();
    setoPAV();
    setTWA();
    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    TravelTime<<TAV<<" "<<PAV<<" "<<oPAV<<" "<<TWA<<" "<<schedule_step-stepnum<<endl;
    c->setEnterTime(schedule_step);
    c->setOPTP(OPTP);
    c->setbegin((double)stepnum);
    c->setDelay(schedule_step);
    
}
void RoadController::printenter(){
    cout<<"printenter"<<getID()<<endl;
    for(Car* c:enter_car)cout<<c->getID()<<" ";
    cout<<endl;
}
void RoadController::optimize(int step){
    //cout<<"enter1"<<endl;
    //for(Car* c:enter_car)cout<<c->getID()<<" ";
    //cout<<endl;
    //cout<<"step"<<endl;
    //cout<<"RoadController ID : "<<ID<<endl; 
    vector<tuple<string,Car*,int,int>> scheduled=sc->Simulated_Annealing(enter_car,(double)step);
    //enter_car=vector<Car*>(0);
    
    enter_car.clear();
    
    //cout<<"enter2"<<endl;
    //for(Car* c:enter_car)cout<<c->getID()<<" ";
    //cout<<endl;
    /*
    cout<<"on road"<<endl;
    for(Car* c:on_road_car)cout<<c->getID()<<" ";
    cout<<endl;
    */
    for(tuple<string,Car*,int,int> T:scheduled){
        string via=get<0>(T);
        Car* car=get<1>(T);
        //string c=get<2>(T);
        int tempdelay=get<2>(T);
        //cout<<via<<" "<<car->getID()<<" "<<tempdelay<<endl;
        car->setEnterTime(tempdelay);
        assigned_car(tempdelay,car,get<3>(T));
        //cout<<tempdelay<<"  OPTP:"<<get<3>(T)<<"   "<<tempdelay-get<3>(T)-step<<"   "<<car->getDelay()<<endl;;
    }
}
vector<string> RoadController::iterative_carlist(int step){
    vector<string> optimizeCar(0);
    bool booltmp=false;
    for(Car* c:on_road_car){
        if(c->Arrivaltime()-step>200){
            booltmp=true;
        }
        if(booltmp){
            optimizeCar.push_back(c->getID());
        }
    }
    return optimizeCar;
}
void RoadController::iterative_optimize(int step){
    bool booltmp=false;
    vector<Car*> optimizeCar(0);
    vector<string> lis(0);
    vector<Car*> NCar(0);
    
    for(Car* c:on_road_car){
        if(c->Arrivaltime()-step>100){
            booltmp=true;
        }
        if(booltmp){
            optimizeCar.push_back(c);
            lis.push_back(c->getID());
        }else{
            NCar.push_back(c);
        }
    }
    cout<<"Reset"<<endl;
    sc->__Reset(lis,NCar);
    cout<<"annealing"<<endl;
    vector<tuple<string,Car*,int,int>> scheduled=sc->Simulated_Annealing(optimizeCar,(double)step);
    enter_car=vector<Car*>(0);
    cout<<"endl"<<endl;
    optimizeCar.clear();
    
    
    for(tuple<string,Car*,int,int> T:scheduled){
        string via=get<0>(T);
        Car* car=get<1>(T);
        //string c=get<2>(T);
        int tempdelay=get<2>(T);
        //cout<<via<<" "<<car->getID()<<" "<<tempdelay<<endl;
        car->setEnterTime(tempdelay);
        //cout<<car->getID()<<" :   "<<tempdelay<<endl;
        car->setDelay(tempdelay);
        //car->setDelay(tempdelay-get<3>(T)-step);
        //cout<<tempdelay<<"  OPTP:"<<get<3>(T)<<"   "<<tempdelay-get<3>(T)-step<<"   "<<car->getDelay()<<endl;;
    }
}
void RoadController::setTWA(){
    TWA=0;
    double tmp=0;
    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    for(int i=stepnum-500;i<=stepnum;i++){
        if(crossed_travelTime.find(i)!=crossed_travelTime.end()){
            for(double t:crossed_travelTime[i]){
                TWA+=t;
                //cout<<"twa"<<":"<<t<<endl;
                tmp++;
            }
        }
    }
    if(tmp==0){
        TWA=0;
    }else{
        TWA=TWA/tmp;
    }
    //cout<<"TWA  :"<<TWA<<endl;
}