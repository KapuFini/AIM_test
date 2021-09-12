#include "Overall.h"
#include "Car.h"
#include "RoadController.h"
using namespace std;
ofstream Throughput("Throughput.txt");
ofstream Delay("Delay.txt");
vector<string> split(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
    if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}


Overall::Overall(string netfile){
    throughput=0;
    using namespace boost::property_tree;
    ptree pt;
    read_xml(netfile, pt);
    for (auto it :pt.get_child("net")){
        if(it.first=="junction"){
            if(auto id=it.second.get_optional<std::string>("<xmlattr>.id")){
                std::string Id=id.get();
                if(auto TYPES=it.second.get_optional<std::string>("<xmlattr>.type")){
                    if(TYPES.get()=="priority"){
                        if(auto icL=it.second.get_optional<std::string>("<xmlattr>.incLanes")){
                            std::string incLanes_str=icL.get();
                            std::vector<std::string> incLanes=split(incLanes_str,' ');
                        
                            junction_lane[Id]=incLanes;
                            for(std::string la:incLanes){
                                lane_junction[la]=Id;
                            }
                        }
                    }
                    
                
                }
            }
        }else if(it.first=="edge"){
            if(auto id=it.second.get_optional<std::string>("<xmlattr>.id")){
                std::string Id=id.get();
                if(auto FUNC=it.second.get_optional<std::string>("<xmlattr>.function")){
                    if(FUNC.get()=="internal"){
                        for(auto &itt:it.second){
                            if(itt.first=="lane"){
                                std::string LaneId=itt.second.get<std::string>("<xmlattr>.id");
                                std::string speed=itt.second.get<std::string>("<xmlattr>.speed");
                                std::string shapestr=itt.second.get<std::string>("<xmlattr>.shape");
                                std::vector<std::string> shapeliststr=split(shapestr,' ');
                                std::vector<std::pair<double,double>> shape(0);
                                for(std::string ShapeStr:shapeliststr){
                                    std::vector<std::string> a=split(ShapeStr,',');
                                    shape.push_back(make_pair<double,double>(stod(a[0]),stod(a[1])));
                                }
                                lane_shape[LaneId]=shape;
                                lane_speed[LaneId]=stod(speed);
                                lane_edge[LaneId]=Id;
                                if(edge_lane.find(Id)==edge_lane.end()){
                                    edge_lane[Id]=vector<string>(0);
                                    edge_lane[Id].push_back(LaneId);
                                    //cout<<Id<<" "<<LaneId<<endl;
                                }else{
                                    edge_lane[Id].push_back(LaneId);
                                }
                                
                            }
                        }
                    }
                }else{
                    if(auto F=it.second.get_optional<std::string>("<xmlattr>.from")){
                        std::string from=F.get();
                        if(auto T=it.second.get_optional<std::string>("<xmlattr>.to")){
                            std::string to=T.get();
                        
                            for(auto &itt:it.second){
                                if(itt.first=="lane"){
                                    std::string LaneId=itt.second.get<std::string>("<xmlattr>.id");
                                    //cout<<Id<<" "<<LaneId<<endl;
                                    std::string speed=itt.second.get<std::string>("<xmlattr>.speed");
                                    std::string shapestr=itt.second.get<std::string>("<xmlattr>.shape");
                                    std::vector<std::string> shapeliststr=split(shapestr,' ');
                                    std::vector<std::pair<double,double>> shape(0);
                                    for(std::string ShapeStr:shapeliststr){
                                        std::vector<std::string> a=split(ShapeStr,',');
                                        shape.push_back(make_pair<double,double>(stod(a[0]),stod(a[1])));
                                    }
                                    lane_edge[LaneId]=Id;
                                    lane_shape[LaneId]=shape;
                                    lane_speed[LaneId]=stod(speed);
                                    if(edge_lane.find(Id)==edge_lane.end()){
                                        edge_lane[Id]=vector<string>(0);
                                        edge_lane[Id].push_back(LaneId);
                                    }else{
                                        edge_lane[Id].push_back(LaneId);
                                    }
                                }
                            }
                        }
                    }
                }

            }
        }else if(it.first=="connection"){
            if(auto F=it.second.get_optional<std::string>("<xmlattr>.from")){
                std::string from=F.get();
                if(auto T=it.second.get_optional<std::string>("<xmlattr>.to")){
                    std::string to=T.get();
                    if(auto V=it.second.get_optional<std::string>("<xmlattr>.via")){
                        std::string via=V.get();
                        fromto_via[from][to]=via;
                        //cout<<from<<" "<<to<<" "<<via<<endl;
                        //cout<<from<<" "<<to<<" "<<via<<endl;
                        via_fromto[via]=std::make_pair(from,to);
                    }
                }
            }
        }
        
    }
    for(std::pair<std::string,std::vector<std::string> > p:junction_lane){
        std::string id=p.first;
        std::vector<std::string> lanes=p.second;
        std::map<std::string,std::map<std::string,std::string> > l_via;
        RoadController* rc=new RoadController(id);
        
        //RCs[id]=rc;
        rc->setLane(lanes);
        
        std::map<std::string,std::vector<std::pair<double,double> > > Shape;
        std::vector<std::string> edges(0);
        for(int i=0;i<lanes.size();i++){
            
            std::string edge_i=lane_edge[lanes[i]];
            edges.push_back(edge_i);
            std::map<std::string,std::string> MAP=fromto_via[edge_i];

            //cout<<lanes[i]<<" "<<edge_i<<" MAPSIZE"<<MAP.size()<<endl;
            for(std::pair<std::string,std::string> p:MAP){
                std::string edge_j=p.first;
                //for(std::string lane_j:edge_lane[edge_j]){
                l_via[edge_i][edge_j]=p.second;
                Shape[p.second]=lane_shape[p.second];
                std::string pp=lane_edge[p.second],nvia=p.second;
                //cout<<pp<<endl;
                while(fromto_via.find(pp)!=fromto_via.end()){
                    
                    if(fromto_via[pp].find(edge_j)!=fromto_via[pp].end()){
                        if(fromto_via[pp][edge_j]=="")break;
                        l_via[nvia][edge_j]=fromto_via[pp][edge_j];
                        Shape[fromto_via[pp][edge_j]]=lane_shape[fromto_via[pp][edge_j]];
                        /*
                        cout<<fromto_via[pp][edge_j]<<endl;
                        
                        for(int w=0;w<Shape[fromto_via[pp][edge_j]].size();w++){
                            cout<<Shape[fromto_via[pp][edge_j]][w].first<<" "<<Shape[fromto_via[pp][edge_j]][w].second<<endl;
                        }
                        cout<<"pp"<<pp<<" , via"<<fromto_via[pp][edge_j]<<endl;
                        */
                        pp=lane_edge[fromto_via[pp][edge_j]];
                        nvia=fromto_via[pp][edge_j];
                    }else{
                        //cout<<"break"<<endl;
                        break;
                    }
                }
                //}
            }
            
            

        }
        rc->setEdge(edges);
        rc->setLaneVia(l_via);
        rc->setShape(Shape);
        RCs.insert(std::make_pair(id,rc));
        for(int i=0;i<lanes.size();i++){
            
            std::string edge_i=lane_edge[lanes[i]];
            edge_junction[edge_i]=id;
        }
    }
}
using namespace std;
Overall::~Overall(){};
void Overall::step(bool timebase){
    //cout<<"Step"<<endl;
    

    //cout<<"CurrentTime:"<<libtraci::Simulation::getCurrentTime()<<endl;
    int departed_step=0;
    //cout<<"depart"<<endl;
    for(string id:libtraci::Simulation::getDepartedIDList()){
        string edge=libtraci::Vehicle::getRoadID(id);
        //RoadController RoadCon=RCs[edge];
        departed_step++;
        decltype(RCs)::iterator it = RCs.find(edge_junction[edge]);
        //RCs[edge].pushCar(c);
        DepartTimes[id]=libtraci::Simulation::getCurrentTime()/100;
        if(it!=RCs.end()){
            if(timebase){
                reroute(id,false);
                
            }
            for(string EDGE:libtraci::Vehicle::getRoute(id)){
                string JUN=edge_junction[EDGE];
                auto It=RCs.find(JUN);
                if(It!=RCs.end()){
                    It->second->setPAV(true);
                }
            }


            Car* c=new Car(id,it->second);
            //cout<<&c<<endl;
            it->second->pushCar(c);
            //cout<<it->second.carlist().size()<<endl;
            Cars.insert(make_pair(id,c));
            Car_RC.insert(make_pair(id,edge_junction[edge]));
            //Car_RC.insert(make_pair(id,it->second));
        }
    }
    //cout<<"arrive"<<endl;
    int arrived_step=0;
    for(string id:libtraci::Simulation::getArrivedIDList()){
        auto iter=Cars.find(id);
        //cout<<"arrived!  "<<id<<endl;
        arrived+=1;
        arrived_step++;
        ArriveTimes[id]=libtraci::Simulation::getCurrentTime()/100;
        if(iter!=end(Cars)){
            //auto iter_RC=Car_RC.find(id);
            //cout<<id<<endl;
            string Junc=Car_RC[id];
            //cout<<id<<" "<<V_EDGE <<endl;
            //string Junc=edge_junction[V_EDGE];
            auto iter_RC=RCs.find(Junc);
            if(iter_RC!=end(RCs)){
                
                //for(Car *c :*iter_RC->second.carlist()){cout<<c->getID()<<" ";}cout<<endl;
                //cout<<"RC's ID:"<<iter_RC->second->getID()<<endl;
                iter_RC->second->erase_Car(iter->second);
                iter_RC->second->setPAV(false);
            }
            //cout<<"DELETE"<<endl;
            //Car_RC[id].erase_Car(iter->second);
            Cars.erase(id);
            //c.~Car();
            //iter->second.~Car();
        }

    }
    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    //cout<<"optimize"<<endl;
    for(pair<string,RoadController*> P :RCs){
        RoadController* RC=P.second;
        //cout<<RC->getID()<<endl;
        RC->optimize(stepnum);
        //cout<<"dispatch"<<endl;
        RC->dispatch_car_from_waiting(stepnum);
        //cout<<"step"<<endl;
        RC->step(stepnum);
        
        //RC->printenter();
    }
    //cout<<"change"<<endl;
    for(pair<string,Car*> CAR:Cars){
        string C_ID=CAR.first;
        string Junc_ID=Car_RC[C_ID];
        string EDGE=libtraci::Vehicle::getRoadID(C_ID);
        
        if(edge_junction.find(EDGE)!=edge_junction.end()){
            string ED_Junc_ID=edge_junction[EDGE];
            if(ED_Junc_ID!=Junc_ID){
                auto iter=RCs.find(Junc_ID);
                //cout<<C_ID<<" : Junction: "<<Junc_ID<<" "<<ED_Junc_ID<<endl;
                if(iter!=RCs.end()){
                    auto iter_RC=RCs.find(ED_Junc_ID);
                    //cout<<"exist"<<
                    if(iter_RC!=RCs.end()){
                        delays[C_ID]+= CAR.second->getDelay();
                        //cout<<"delay:  "<<C_ID<<" "<<delays[C_ID]<<" "<<CAR.second->getDelay()<<endl;
                        //Times[C_ID]+=((double)(CAR.second->Arrivaltime()-stepnum))/10-CAR.second->getDelay();
                        iter->second->erase_Car(CAR.second);
                        iter->second->setPAV(false);
                        Car* c=new Car(C_ID,iter_RC->second);
                        iter_RC->second->pushCar(c);
                        Cars[C_ID]=c;
                        Car_RC[C_ID]=ED_Junc_ID;
                        //cout<<"change"<<EDGE<<"==>>"<<ED_Junc_ID<<endl;
                        throughput+=1;
                    }
                }
            }
        }
    }
    Throughput<<departed_step<<" "<<arrived_step<<endl;
}

int Overall::getThroughput(){
    return throughput;
}
int Overall::getArrived(){
    return arrived;
}

void Overall::reroute(string vid,bool iterative){
    map<string,vector<string> > route;
    map<string,double> time;
    vector<string> be=libtraci::Vehicle::getRoute(vid);
    std::string TypeID=libtraci::Vehicle::getTypeID(vid);
    std::string VeClass=libtraci::VehicleType::getVehicleClass(TypeID);
    //std::cout<<"Vehicle Class  :  "<<VeClass<<std::endl;
    
    string o=be[0],d=be[be.size()-1];
    //string now=o;
    string now=libtraci::Vehicle::getRoadID(vid);
    o=now;
    priority_queue<pair<double,string>,vector<pair<double,string>>,greater<pair<double,string>>> pq;
    pq.emplace(make_pair(0,now));
    time[now]=0;
    route[now]=vector<string>(0);
    route[now].push_back(now);
    //cout<<o<<" "<<d<<endl;
    while(!pq.empty()){
        pair<double,string> p=pq.top();
        pq.pop();
        string v=p.second;
        if(v==d){
            break;
        }
        string jun=edge_junction[v];
        double tra=1;
        auto it=RCs.find(jun);
        if (it!=RCs.end()){
            tra=it->second->getTraversalTime();
        }
        //RCs[jun]->getTraversalTime();
    
        double CurrentT=p.first;

        vector<string> rou=route[v];
        for(pair<string,string> Pair:fromto_via[v]){
            string EDGE=Pair.first;
            bool allowed=false;
            //cout<<"edge"<<EDGE<<"   "<<edge_lane[EDGE].size()<<"  "<<libtraci::Edge::getLaneNumber(EDGE)<<endl;
            for(string L:edge_lane[EDGE]){
                //cout<<"Lane  "<<L;
                bool Al=false;
                for(string vclass:libtraci::Lane::getAllowed(L)){
                    if(vclass==VeClass){
                        Al=true;
                    }
                    //cout<<"     "<<vclass;
                }
                //cout<<endl;
                bool DisAl=false;
                for(string vclass:libtraci::Lane::getDisallowed(L)){
                    if(vclass==VeClass){
                        DisAl=true;
                    }
                    //cout<<"     "<<vclass;
                }

                if(Al&&!DisAl){
                    allowed=true;
                }else if(!Al&&!DisAl){
                    allowed=true;
                }else if(DisAl){
                    allowed=false;
                }
                
                //cout<<endl;
                if(allowed)break;
            }
            if(allowed){
                if(time.find(EDGE)==time.end()){
                    time[EDGE]=CurrentT+tra;
                    route[EDGE]=rou;
                    route[EDGE].push_back(EDGE);
                    pq.emplace(make_pair(CurrentT+tra,EDGE));
                }else if(time[EDGE]>CurrentT+tra){
                    time[EDGE]=CurrentT+tra;
                    route[EDGE]=rou;
                    route[EDGE].push_back(EDGE);
                    pq.emplace(make_pair(CurrentT+tra,EDGE));
                }
            }
            //cout<<"   "<<EDGE<<"   "<<time[EDGE]<<endl;
        }
    }
    bool tmp=false;
    for(string ed:route[d]){
        if(iterative)cout<<ed<<" ";
        if(ed==now)tmp=true;
        if(iterative){
            RCs[edge_junction[ed]]->setPAV(true);
        }
    }
    if(iterative){
        cout<<endl;
        cout<<endl;
    }
    //cout<<"before: ";for(string r:be){cout<<r<<" ";}cout<<endl<<"after: ";for(string r:route[d]){cout<<r<<" ";}cout<<endl;
    libtraci::Vehicle::setRoute(vid,route[d]);
}
void Overall::PrintDelay(){
    for(pair<string,double> p:delays){
        double traveltime=libtraci::Simulation::getCurrentTime()/100;;
        if(ArriveTimes.find(p.first)!=ArriveTimes.end()){

            Delay<<ArriveTimes[p.first]-DepartTimes[p.first]<<" "<<delays[p.first]<<endl;
        }
        
    }
}


void Overall::iterative_step(){
    //cout<<"Step"<<endl;
    

    //cout<<"CurrentTime:"<<libtraci::Simulation::getCurrentTime()<<endl;
    int departed_step=0;
    //cout<<"depart"<<endl;
    for(string id:libtraci::Simulation::getDepartedIDList()){
        string edge=libtraci::Vehicle::getRoadID(id);
        //RoadController RoadCon=RCs[edge];
        departed_step++;
        decltype(RCs)::iterator it = RCs.find(edge_junction[edge]);
        //RCs[edge].pushCar(c);
        DepartTimes[id]=libtraci::Simulation::getCurrentTime()/100;
        if(it!=RCs.end()){
            reroute(id,false);
            for(string EDGE:libtraci::Vehicle::getRoute(id)){
                string JUN=edge_junction[EDGE];
                auto It=RCs.find(JUN);
                if(It!=RCs.end()){
                    It->second->setPAV(true);
                }
            }


            Car* c=new Car(id,it->second);
            //cout<<&c<<endl;
            it->second->pushCar(c);
            //cout<<it->second.carlist().size()<<endl;
            Cars.insert(make_pair(id,c));
            Car_RC.insert(make_pair(id,edge_junction[edge]));
            //Car_RC.insert(make_pair(id,it->second));
        }
    }
    //cout<<"arrive"<<endl;
    int arrived_step=0;
    for(string id:libtraci::Simulation::getArrivedIDList()){
        auto iter=Cars.find(id);
        //cout<<"arrived!  "<<id<<endl;
        arrived+=1;
        arrived_step++;
        ArriveTimes[id]=libtraci::Simulation::getCurrentTime()/100;
        if(iter!=end(Cars)){
            //auto iter_RC=Car_RC.find(id);
            //cout<<id<<endl;
            string Junc=Car_RC[id];
            //cout<<id<<" "<<V_EDGE <<endl;
            //string Junc=edge_junction[V_EDGE];
            auto iter_RC=RCs.find(Junc);
            if(iter_RC!=end(RCs)){
                
                //for(Car *c :*iter_RC->second.carlist()){cout<<c->getID()<<" ";}cout<<endl;
                //cout<<"RC's ID:"<<iter_RC->second->getID()<<endl;
                iter_RC->second->erase_Car(iter->second);
                iter_RC->second->setPAV(false);
            }
            //cout<<"DELETE"<<endl;
            //Car_RC[id].erase_Car(iter->second);
            Cars.erase(id);
            //c.~Car();
            //iter->second.~Car();
        }

    }
    int stepnum=libtraci::Simulation::getCurrentTime()/100;
    


    //cout<<"optimize"<<endl;
    for(pair<string,RoadController*> P :RCs){
        RoadController* RC=P.second;
        if(stepnum%100==0){
            vector<string> list=RC->iterative_carlist(stepnum);
            for(string vid:list){
                string now=libtraci::Vehicle::getRoadID(vid);
                bool tmp=false;
                cout<<vid<<"  "<<now<<endl;
                for(string ed:libtraci::Vehicle::getRoute(vid)){
                    cout<<ed<<" ";
                    if(ed==now)tmp=true;
                    if(tmp){
                        RCs[edge_junction[ed]]->setPAV(false);
                    }
                }
                cout<<endl;
                reroute(vid,true);
                
                

            }
            //cout<<"optimize"<<endl;
            RC->iterative_optimize(stepnum);
            
        }
        //cout<<RC->getID()<<endl;
        RC->optimize(stepnum);
        //cout<<"dispatch"<<endl;
        //RC->dispatch_car_from_waiting(stepnum);
        //cout<<"step"<<endl;
        RC->step(stepnum);
        
        //RC->printenter();
    }
    
    //cout<<"change"<<endl;
    for(pair<string,Car*> CAR:Cars){
        string C_ID=CAR.first;
        string Junc_ID=Car_RC[C_ID];
        string EDGE=libtraci::Vehicle::getRoadID(C_ID);
        
        if(edge_junction.find(EDGE)!=edge_junction.end()){
            string ED_Junc_ID=edge_junction[EDGE];
            if(ED_Junc_ID!=Junc_ID){
                auto iter=RCs.find(Junc_ID);
                //cout<<C_ID<<" : Junction: "<<Junc_ID<<" "<<ED_Junc_ID<<endl;
                if(iter!=RCs.end()){
                    auto iter_RC=RCs.find(ED_Junc_ID);
                    //cout<<"exist"<<
                    if(iter_RC!=RCs.end()){
                        delays[C_ID]+= CAR.second->getDelay();
                        //cout<<"delay:  "<<C_ID<<" "<<delays[C_ID]<<" "<<CAR.second->getDelay()<<endl;
                        //Times[C_ID]+=((double)(CAR.second->Arrivaltime()-stepnum))/10-CAR.second->getDelay();
                        iter->second->erase_Car(CAR.second);
                        iter->second->setPAV(false);
                        Car* c=new Car(C_ID,iter_RC->second);
                        iter_RC->second->pushCar(c);
                        Cars[C_ID]=c;
                        Car_RC[C_ID]=ED_Junc_ID;
                        //cout<<"change"<<EDGE<<"==>>"<<ED_Junc_ID<<endl;
                        throughput+=1;
                    }
                }
            }
        }
    }
    Throughput<<departed_step<<" "<<arrived_step<<endl;
}
