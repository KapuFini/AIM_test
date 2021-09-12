#include "Scheduler.h"
#include "Car.h"
#include <random>
ofstream Outputfile("TILE.txt");
Scheduler::Scheduler(){
    
    SimulationStepLength = 0.1;
    StepPerSecond = ceil(1/SimulationStepLength);
    MinimumGap = 1.0;
    VehicleLength = 5.0;
    LaneDistance = 300;
    FixedSpeed = 15.0;
    __OPTP=(LaneDistance/FixedSpeed)*StepPerSecond;
    __MiniLaneDistance=MinimumGap;
    __Hmin=ceil((__MiniLaneDistance/5.0)*StepPerSecond);
    __MiniIntersectionDistance=VehicleLength+__MiniLaneDistance;
    __Delta=ceil((__MiniIntersectionDistance/5.0)*StepPerSecond);
    __TotalDelay=0.0;
    __VehicleNumber=0;
}
void Scheduler::setLaneVia(map<string,map<string,string> > l_v){
    this->edge_via=l_v;
    
}

void Scheduler::setShape(map<string,vector<pair<double,double> > > Shape){
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
    tile=vector<vector<map<int,string> > >(height,vector<map<int,string> >(width,map<int,string>()));
    ConflictPoint=vector<vector<map<int,string>>>(height,vector<map<int,string>>(width,map<int,string>()));
    ConflictPointTemp=vector<vector<map<int,string>>>(height,vector<map<int,string>>(width,map<int,string>()));
    
}
double Scheduler::QueryTotalDelay(){
    if (__VehicleNumber==0){
        return __TotalDelay/StepPerSecond;

    }else{
        return (__TotalDelay/StepPerSecond)/__VehicleNumber;

    }
}
void Scheduler::__Clear_Temp(){
    //for(int i=0;i<__Conflict_Point_temp.size();i++){__Conflict_Point_temp[i]={};}
    for(int i=0;i<ConflictPoint.size();i++){
        for(int j=0;j<ConflictPoint[i].size();j++){
            ConflictPointTemp[i][j]=ConflictPoint[i][j];
        }
    }
}
void Scheduler::__Reset(vector<string> carlist,vector<Car*> NCar){
    for(int i=0;i<ConflictPoint.size();i++){
        for(int j=0;j<ConflictPoint[i].size();j++){
            bool booltmp=false;
            for(pair<int,string> P:ConflictPoint[i][j]){

                for(string vid:carlist){
                    if(vid==P.second){
                        booltmp=true;
                        auto iter=ConflictPoint[i][j].find(P.first);
                        ConflictPoint[i][j].erase(iter,ConflictPoint[i][j].end());
                    }
                    if(booltmp)break;
                }
                if(booltmp)break;
            }
        
        }

        
    }
    for(pair<string,int> P:LastVehicleArrivalTime){
        LastVehicleArrivalTime[P.first]=libtraci::Simulation::getCurrentTime()/100;
    }
    for(Car* c:NCar){
        //Car* c=comingVehicle[index];
        string via="";
        string now=libtraci::Vehicle::getRoadID(c->getID());
        string next="";
        vector<string> Route=libtraci::Vehicle::getRoute(c->getID());
        //cout<<"Via"<<endl;
        string v_id=c->getID();
        for(int i=0;i<Route.size();i++){
            string e=Route[i];
            if(e==now&&i<Route.size()-1){
                via=edge_via[e][Route[i+1]];
                next=Route[i+1];
            }else if(e==now&&i==Route.size()-1){
                next="LAST";
            }
        }
        if(next=="")continue;
        if(next=="LAST"&&via==""){
            for(pair<string,string> p:edge_via[now]){
                via=p.second;
                next=p.first;
                break;
            }
        }
        LastVehicleArrivalTime[via]=max(LastVehicleArrivalTime[via],c->Arrivaltime());
    }
    
}

vector<tuple<string,Car*,int,int>> Scheduler::Simulated_Annealing(vector<Car*> comingVehicle,double CurrentTimeStep){
    //cout<<"annealing : "<<comingVehicle.size()<<endl;
    //for(Car* c:comingVehicle)cout<<c->getID()<<" ";
    //cout<<endl;
    double v_vel=5.0;
    //double v_vel=FixedSpeed;
    double BestDelay=DBL_MAX;
    int Iteration=3000;
    vector<tuple<string,Car*,int>> BestSol(0);
    if(comingVehicle.size()==0){
        vector<tuple<string,Car*,int,int>> tmp(0);
        return tmp;
    }
    int SumTempDelay=0;
    double LastDelay=0;
    int MaxDelay=20*StepPerSecond;
    random_device rd;
    default_random_engine eng(rd());
    uniform_int_distribution<> distr(0,MaxDelay);
    vector<tuple<string,Car*,int>> TempSol;
    /*
    for(int i=0;i<Iteration;i++){
        //cout<<"Iteration"<<i<<endl;
        __Clear_Temp();
        pair<double,vector<tuple<string,Car*,int>>> p=GenerateValidSolution(comingVehicle,CurrentTimeStep);
        SumTempDelay=p.first;
        TempSol=p.second;
        if(SumTempDelay<BestDelay){
            BestSol=TempSol;
            BestDelay=SumTempDelay;
            LastDelay=SumTempDelay;
        }else{
            double Temperature=(double)Iteration/(double)(i+1);
            double difference=LastDelay-SumTempDelay;
            double average=(double)(difference)/(double)(comingVehicle.size());
            average/=StepPerSecond;
            if(distr(eng)<exp(-average/Temperature)){
                LastDelay=SumTempDelay;
            }
        }
    }
    */
    __Clear_Temp();
    //cout<<"generate"<<endl;
    pair<double,vector<tuple<string,Car*,int>>> p=GenerateValidSolution1(comingVehicle,CurrentTimeStep);
    //cout<<"generate end"<<endl;
    SumTempDelay=p.first;
    TempSol=p.second;
    if(SumTempDelay<BestDelay){
        BestSol=TempSol;
        BestDelay=SumTempDelay;
        LastDelay=SumTempDelay;
    }
    __TotalDelay+=BestDelay;
    __VehicleNumber+=comingVehicle.size();
    //cout<<"Clear"<<endl;
    //__Clear_Temp();
    vector<tuple<string,Car*,int,int> > ans(0);
    //cout<<"size:"<<comingVehicle.size()<<endl;
    for (int index=0;index<comingVehicle.size();index++){
        Car* c=comingVehicle[index];
        string via="";
        string now=libtraci::Vehicle::getRoadID(c->getID());
        string next="";
        vector<string> Route=libtraci::Vehicle::getRoute(c->getID());
        //cout<<"Via"<<endl;
        string v_id=c->getID();
        for(int i=0;i<Route.size();i++){
            string e=Route[i];
            if(e==now&&i<Route.size()-1){
                via=edge_via[e][Route[i+1]];
                next=Route[i+1];
            }else if(e==now&&i==Route.size()-1){
                next="LAST";
            }
        }
        if(next=="")continue;
        if(via==""&&next=="LAST"){
            for(pair<string,string> p:edge_via[now]){
                via=p.second;
                next=p.first;
                break;
            }
        }
        v_vel=5.0;
        //v_vel=max(3.0,(double)floor(libtraci::Lane::getMaxSpeed(via)));
        double L_length     =libtraci::Lane::getLength(libtraci::Vehicle::getLaneID(c->getID()));
        double pos          =libtraci::Vehicle::getLanePosition(c->getID());
        
        LaneDistance=L_length-pos;
        __OPTP=0;
        double first_speed=(double)floor(libtraci::Vehicle::getSpeed(comingVehicle[index]->getID()));
        double end_speed=(double)floor(v_vel);
        double maxspeed=(double)floor(FixedSpeed);
        double lanelength=LaneDistance+(first_speed*2+1)*first_speed*2/40+(end_speed*2+1)*end_speed*2/40;
        /*
        lanelength-=2*(maxspeed*2+1)*maxspeed*2/40;
        __OPTP=lanelength/maxspeed+(maxspeed*2-first_speed-end_speed)/5;
        __OPTP*=StepPerSecond;
        __OPTP=(double)ceil(__OPTP);
        */
        if(lanelength>=2*(maxspeed*2+1)*maxspeed*2/40){
            lanelength-=2*(maxspeed*2+1)*maxspeed*2/40;
            __OPTP=lanelength/maxspeed+(maxspeed*2-first_speed-end_speed)/5;
            __OPTP*=StepPerSecond;
            __OPTP=(double)ceil(__OPTP);
        }else{
            double ms=max(end_speed,first_speed)+0.5;
            while(lanelength>2*(ms*2+1)*ms*2/40){
                ms+=0.5;
            }
            ms-=0.5;
            __OPTP=max(0.0,lanelength-2*(ms*2+1)*ms*2/40)/ms+(ms*2-first_speed-end_speed)/5;
            __OPTP*=StepPerSecond;
            __OPTP=(double)ceil(__OPTP);
        }
        
        
        
        double T_Stop_Line=CurrentTimeStep+__OPTP+get<2>(BestSol[index]);


        //ans.push_back(make_tuple(get<0>(BestSol[index]),get<1>(BestSol[index]),CurrentTimeStep+get<2>(BestSol[index])));
        ans.push_back(make_tuple(get<0>(BestSol[index]),get<1>(BestSol[index]),T_Stop_Line,(int)__OPTP));
        string lane=libtraci::Vehicle::getLaneID(c->getID());
        LastVehicleArrivalTime[lane]=T_Stop_Line;
        //LastVehicleArrivalTime[via]=T_Stop_Line;
        double t=T_Stop_Line;
        //double t=T_Stop_Line;
        if(comingVehicle.size()>1){
            
            //cout<<"annealing "<<CurrentTimeStep<<" "<<__OPTP<<" "<<c->getID()<<"  "<<get<2>(BestSol[index])<<endl;
        }
        double x=shape[via][0].first,y=shape[via][0].second;
        int Now=0,Next=1;
            

        while(Next<shape[via].size()){
            double dX=shape[via][Next].first-x;
            double dY=shape[via][Next].second-y;
            //cout<<dX<<" "<<dY<<endl;
            //double dX=shape[Via][Next].first-shape[Via][Now].first;
            //double dY=shape[Via][Next].second-shape[Via][Now].second;
            double cos=dX/sqrt(dX*dX+dY*dY);
            double sin=dY/sqrt(dX*dX+dY*dY);
            string VTYPE=libtraci::Vehicle::getTypeID(v_id);
            double v_length=libtraci::VehicleType::getLength(VTYPE);
            double v_width=libtraci::VehicleType::getWidth(VTYPE);
            double X=x-floor(W);
            double Y=y-floor(S);
            /*
                3                     0
                |--------------------|
                |                    |
                |                    | =====>>>>>>>
                |                    |
                |--------------------|
                2                     1
            
            */
            vector<pair<double,double> > corner(4);
            //cout<<v_width<<" "<<v_length<<endl;
            /*
            corner[0].first=v_length/2;
            corner[0].second=v_width/2;
            corner[1].first=v_length/2;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length/2;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length/2;
            corner[3].second=v_width/2;
            */
            
            corner[0].first=0;
            corner[0].second=v_width/2;
            corner[1].first=0;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length;
            corner[3].second=v_width/2;
            for(int i=0;i<4;i++){
                //double fir=corner[i].first,sec=corner[i].second;
                corner[i].first+=X;
                corner[i].second+=Y;
            }
            vector<vector<bool> > grid(height,vector<bool>(width,false));
            vector<pair<double,double>> grid_corner(4);
            grid_corner[0]=make_pair(0.0,0.0);
            grid_corner[1]=make_pair(0.5,0.0);
            grid_corner[2]=make_pair(0.0,0.5);
            grid_corner[3]=make_pair(0.5,0.5);
            /*
            if(c->getID()=="0"){
                cout<<"("<<X<<","<<Y<<"),("<<floor(W)<<endl;
                cout<<"("<<shape[via][Now].first<<","<<shape[via][Now].second<<"),("<<shape[via][Next].first<<","<<shape[via][Next].second<<")"<<endl;
            }
            */
            for(int i=0;i<height;i++)for(int j=0;j<width;j++){
                for(pair<double,double> gc:grid_corner){
                    double Xcorner=((double)j)*0.5+gc.first-X;
                    double Ycorner=((double)i)*0.5+gc.first-Y;
                    double Xc=Xcorner,Yc=Ycorner;
                    Xcorner=Xc*cos+Yc*sin+X;
                    Ycorner=-Xc*sin+Yc*cos+Y;
                    if(Xcorner<=corner[0].first&&Xcorner>=corner[3].first&&
                        Ycorner<=corner[0].second&&Ycorner>=corner[1].second){
                            grid[i][j]=true;
                            //for(int Buf=-1;Buf<=1;Buf++)tile_time[i][j].push_back(t+Buf);
                            break;
                    }
                }
            }
            


            for(int i=0;i<height;i++){
                for(int j=0;j<width;j++){
                    if(grid[i][j]){
                        int Upper=ceil(t+__Delta);
                        //int Lower=floor(t-__Delta);
                        int Lower=floor(t-5);
                        for(int k=Lower;k<=Upper;k++){
                            if(ConflictPoint[i][j].find(k)!=ConflictPoint[i][j].end()){
                                if(ConflictPoint[i][j][k]!=c->getID()){
                                    cout<<t<<"   "<<ConflictPoint[i][j][k]<<" change!"<<c->getID()<<"  "<<i<<" "<<j<<" "<<k<<endl;
                                }
                            }
                            ConflictPoint[i][j][k]=c->getID();
                        }
                        
                    }
                }
                
            }
            
            Outputfile<<"time: "<<(int)t<<"   ,car: "<<c->getID()<<"   ,via:"<<via<<endl;
            for(int i=height-1;i>=0;i--){
                for(int j=0;j<width;j++){
                    if(ConflictPoint[i][j].find((int)(t))!=ConflictPoint[i][j].end()){
                        Outputfile<<ConflictPoint[i][j][t]<<" ";
                    }else{
                        Outputfile<<"non ";
                    }
                }
                Outputfile<<endl;
            }

            t+=1;
            // move V
            double MoveLen=0.1*v_vel;
            //cout<<"dis="<<sqrt(dX*dX+dY*dY)<<"   "<<MoveLen<<endl;
            while(sqrt(dX*dX+dY*dY)<MoveLen&&Next<shape[via].size()){
                MoveLen-=sqrt(dX*dX+dY*dY);
                Now++;
                Next++;
                if(Next>=shape[via].size())break;
                dX=shape[via][Next].first-shape[via][Now].first;
                dY=shape[via][Next].second-shape[via][Now].second;
                cos=dX/sqrt(dX*dX+dY*dY);
                sin=dY/sqrt(dX*dX+dY*dY);
                x=shape[via][Now].first;
                y=shape[via][Now].second;
                //cout<<"v_id="<<v_id<<": x="<<x<<", y="<<y<<endl;
            }

            //cout<<t<<": c :"<<X<<","<<Y<<endl;
            if(Next<shape[via].size()){

                //cout<<v_id<<": size="<<shape[Via].size()<<"  now="<<Now<<" next="<<Next<<" "<<v_vel<<" "<<x<<" "<<y<<endl;
                //cout<<"("<<dX<<", "<<dY<<")"<<endl;
                x+=MoveLen*cos;
                y+=MoveLen*sin;
            }else if(Next==shape[via].size()){
                if(edge_via[via].find(next)==edge_via[via].end())break;
                via=edge_via[via][next];
                Now=0;Next=1;
                //cout<<"via"<<via<<" ,next"<<next<<endl;
                if(shape.find(via)==shape.end())break;
                //cout<<shape[via].size()<<endl;
            }

            
        }
        //cout<<"anneling "<<T_Stop_Line<<endl;
    }
    return ans;
}

pair<int,vector<tuple<string,Car*,int> > > Scheduler::GenerateValidSolution(vector<Car*> comingVehicle,double CurrentTimeStep){
    double MaxDelay=20*StepPerSecond;
    vector<tuple<string,Car*,int> > TempSol(0);
    int SumTempDelay=0;
    int TempDelay=0;
    
    double v_vel=5.0;
    bool successful=false;
    random_device rd;
    //double T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
    
    default_random_engine eng(rd());
    uniform_int_distribution<> distr(0,MaxDelay);
    for(Car* c:comingVehicle){
        string via="";
        string now=libtraci::Vehicle::getRoadID(c->getID());
        string next="";
        vector<string> Route=libtraci::Vehicle::getRoute(c->getID());
        for(int i=0;i<Route.size();i++){
            string e=Route[i];
            if(e==now&&i<Route.size()-1){
                via=edge_via[e][Route[i+1]];
                next=Route[i+1];
            }else if(e==now&&i==Route.size()-1){
                next="LAST";
            }
        }
        string v_id=c->getID();
        if(via==""){
            for(pair<string,string> p:edge_via[now]){
                via=p.second;
                next=p.first;
                break;
            }
        }
        v_vel=5.0;
        //v_vel=max(3.0,(double)floor(libtraci::Lane::getMaxSpeed(via)));

        __OPTP=0;
        double first_speed=(double)floor(libtraci::Vehicle::getSpeed(c->getID()));
        double end_speed=(double)floor(v_vel);
        double maxspeed=(double)floor(FixedSpeed);
        double lanelength=LaneDistance+(first_speed*2+1)*first_speed*2/40+(end_speed*2+1)*end_speed*2/40;
        lanelength-=2*(maxspeed*2+1)*maxspeed*2/40;
        __OPTP=lanelength/maxspeed+(maxspeed*2-first_speed-end_speed)/5;
        __OPTP*=StepPerSecond;
        __OPTP=(double)ceil(__OPTP);
        double T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
        T_Stop_Line=(double)ceil(T_Stop_Line);
        
        successful=false;
        bool SubValid;
        //cout<<"while"<<endl;
        while(!successful){
            TempDelay=distr(eng);
            //cout<<TempDelay<<endl;
            T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
            T_Stop_Line=(double)ceil(T_Stop_Line);
            string lane=libtraci::Vehicle::getLaneID(c->getID());
            //while(T_Stop_Line<__Hmin+LastVehicleArrivalTime[via]){
            //T_Stop_Line=(double)ceil(__Hmin+LastVehicleArrivalTime[lane]);
            //TempDelay=(int)(T_Stop_Line-CurrentTimeStep-__OPTP);
            while(T_Stop_Line<__Hmin+LastVehicleArrivalTime[lane]){
                //TempDelay=distr(eng);
                TempDelay++;
                T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
                
            }
            SubValid=true;
            // LaneConflict
            
            double t=T_Stop_Line;
            
            double x=shape[via][0].first,y=shape[via][0].second;
            int Now=0,Next=1;
            
            //LaneConflict
            //cout<<"conflict"<<endl;
                

            ////////

            while(Next<shape[via].size()){
                double dX=shape[via][Next].first-x;
                double dY=shape[via][Next].second-y;
                //cout<<dX<<" "<<dY<<endl;
                double cos=dX/sqrt(dX*dX+dY*dY);
                double sin=dY/sqrt(dX*dX+dY*dY);
                string VTYPE=libtraci::Vehicle::getTypeID(v_id);
                double v_length=libtraci::VehicleType::getLength(VTYPE);
                double v_width=libtraci::VehicleType::getWidth(VTYPE);
                double X=x-floor(W);
                double Y=y-floor(S);
                /*
                    3                     0
                    |--------------------|
                    |                    |
                    |                    | =====>>>>>>>
                    |                    |
                    |--------------------|
                    2                     1
                
                */
                vector<pair<double,double> > corner(4);
                /*
                corner[0].first=v_length/2;
                corner[0].second=v_width/2;
                corner[1].first=v_length/2;
                corner[1].second=-v_width/2;
                corner[2].first=-v_length/2;
                corner[2].second=-v_width/2;
                corner[3].first=-v_length/2;
                corner[3].second=v_width/2;
                */
                corner[0].first=0;
                corner[0].second=v_width/2;
                corner[1].first=0;
                corner[1].second=-v_width/2;
                corner[2].first=-v_length;
                corner[2].second=-v_width/2;
                corner[3].first=-v_length;
                corner[3].second=v_width/2;
                for(int i=0;i<4;i++){
                    //double fir=corner[i].first,sec=corner[i].second;
                    corner[i].first+=X;
                    corner[i].second+=Y;
                }
                vector<vector<bool> > grid(height,vector<bool>(width,false));
                vector<pair<double,double>> grid_corner(4);
                grid_corner[0]=make_pair(0.0,0.0);
                grid_corner[1]=make_pair(0.5,0.0);
                grid_corner[2]=make_pair(0.0,0.5);
                grid_corner[3]=make_pair(0.5,0.5);
                
                for(int i=0;i<height;i++)for(int j=0;j<width;j++){
                    for(pair<double,double> gc:grid_corner){
                        double Xcorner=((double)j)*0.5+gc.first-X;
                        double Ycorner=((double)i)*0.5+gc.first-Y;
                        double Xc=Xcorner,Yc=Ycorner;
                        Xcorner=Xc*cos+Yc*sin+X;
                        Ycorner=-Xc*sin+Yc*cos+Y;
                        if(Xcorner<=corner[0].first&&Xcorner>=corner[3].first&&
                            Ycorner<=corner[0].second&&Ycorner>=corner[1].second){
                                grid[i][j]=true;
                                //for(int Buf=-1;Buf<=1;Buf++)tile_time[i][j].push_back(t+Buf);
                                break;
                        }
                    }
                }
                


                for(int i=0;i<height;i++){
                    for(int j=0;j<width;j++){
                        if(grid[i][j]){
                            int Upper=ceil(t+__Delta);
                            //int Lower=floor(t-__Delta);
                            int Lower=floor(t-5);
                        
                            for(int T=Lower;T<=Upper;T++){
                                if(ConflictPoint[i][j].find(T)!=ConflictPoint[i][j].end()||ConflictPointTemp[i][j].find(T)!=ConflictPointTemp[i][j].end()){
                                    //cout<<"true"<<endl;
                                    if(ConflictPointTemp[i][j][T]!=c->getID()){
                                        SubValid=false;
                                        break;
                                    }
                                    //t==upperbound
                                }else if(T==Upper){
                                    //cout<<"false;"<<endl;
                                    SubValid=true;
                                }
                                
                            }
                        }
                        if(!SubValid)break;
                    }
                    if(!SubValid)break;
                }
                

                t+=1;
                // move V
                double MoveLen=0.1*v_vel;
                //cout<<"dis="<<sqrt(dX*dX+dY*dY)<<"   "<<MoveLen<<endl;
                while(sqrt(dX*dX+dY*dY)<MoveLen&&Next<shape[via].size()){
                    MoveLen-=sqrt(dX*dX+dY*dY);
                    Now++;
                    Next++;
                    if(Next>=shape[via].size())break;
                    dX=shape[via][Next].first-shape[via][Now].first;
                    dY=shape[via][Next].second-shape[via][Now].second;
                    cos=dX/sqrt(dX*dX+dY*dY);
                    sin=dY/sqrt(dX*dX+dY*dY);
                    x=shape[via][Now].first;
                    y=shape[via][Now].second;
                    //cout<<"v_id="<<v_id<<": x="<<x<<", y="<<y<<endl;
                }
                if(Next<shape[via].size()){

                    //cout<<v_id<<": size="<<shape[Via].size()<<"  now="<<Now<<" next="<<Next<<" "<<v_vel<<" "<<x<<" "<<y<<endl;
                    //cout<<"("<<dX<<", "<<dY<<")"<<endl;
                    x+=MoveLen*cos;
                    y+=MoveLen*sin;
                }else if(Next==shape[via].size()){
                    if(edge_via[via].find(next)==edge_via[via].end())break;
                    via=edge_via[via][next];
                    Now=0;Next=1;
                    //cout<<"via"<<via<<" ,next"<<next<<endl;
                    if(shape.find(via)==shape.end())break;
                    //cout<<shape[via].size()<<endl;
                }
                //dX-=MoveLen*cos;
                //dY-=MoveLen*sin;
                
                
            }
            




            if(!SubValid){
                continue;
            }else{
                successful=true;
            }
        }

        double t=T_Stop_Line;
            
        double x=shape[via][0].first,y=shape[via][0].second;
        int Now=0,Next=1;
            

        while(Next<shape[via].size()){
            double dX=shape[via][Next].first-x;
            double dY=shape[via][Next].second-y;
            //cout<<dX<<" "<<dY<<endl;
            //double dX=shape[Via][Next].first-shape[Via][Now].first;
            //double dY=shape[Via][Next].second-shape[Via][Now].second;
            double cos=dX/sqrt(dX*dX+dY*dY);
            double sin=dY/sqrt(dX*dX+dY*dY);
            string VTYPE=libtraci::Vehicle::getTypeID(v_id);
            double v_length=libtraci::VehicleType::getLength(VTYPE);
            double v_width=libtraci::VehicleType::getWidth(VTYPE);
            double X=x-floor(W);
            double Y=y-floor(S);
            /*
                3                     0
                |--------------------|
                |                    |
                |                    | =====>>>>>>>
                |                    |
                |--------------------|
                2                     1
            
            */
            vector<pair<double,double> > corner(4);
            /*
            corner[0].first=v_length/2;
            corner[0].second=v_width/2;
            corner[1].first=v_length/2;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length/2;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length/2;
            corner[3].second=v_width/2;
            */
            corner[0].first=0;
            corner[0].second=v_width/2;
            corner[1].first=0;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length;
            corner[3].second=v_width/2;
            for(int i=0;i<4;i++){
                //double fir=corner[i].first,sec=corner[i].second;
                corner[i].first+=X;
                corner[i].second+=Y;
            }
            vector<vector<bool> > grid(height,vector<bool>(width,false));
            vector<pair<double,double>> grid_corner(4);
            grid_corner[0]=make_pair(0.0,0.0);
            grid_corner[1]=make_pair(0.5,0.0);
            grid_corner[2]=make_pair(0.0,0.5);
            grid_corner[3]=make_pair(0.5,0.5);
            
            for(int i=0;i<height;i++)for(int j=0;j<width;j++){
                for(pair<double,double> gc:grid_corner){
                    double Xcorner=((double)j)*0.5+gc.first-X;
                    double Ycorner=((double)i)*0.5+gc.first-Y;
                    double Xc=Xcorner,Yc=Ycorner;
                    Xcorner=Xc*cos+Yc*sin+X;
                    Ycorner=-Xc*sin+Yc*cos+Y;
                    if(Xcorner<=corner[0].first&&Xcorner>=corner[3].first&&
                        Ycorner<=corner[0].second&&Ycorner>=corner[1].second){
                            grid[i][j]=true;
                            //for(int Buf=-1;Buf<=1;Buf++)tile_time[i][j].push_back(t+Buf);
                            break;
                    }
                }
            }
            


            for(int i=0;i<height;i++){
                for(int j=0;j<width;j++){
                    if(grid[i][j]){
                        int Upper=ceil(t+__Delta);
                        //int Lower=floor(t-__Delta);
                        int Lower=floor(t-5);
                        
                        for(int k=Lower;k<=Upper;k++){
                            ConflictPointTemp[i][j][k]=c->getID();
                        }
                        
                    }
                }
                
            }
            

            t+=1;
            // move V
            double MoveLen=0.1*v_vel;
            //cout<<"dis="<<sqrt(dX*dX+dY*dY)<<"   "<<MoveLen<<endl;
            while(sqrt(dX*dX+dY*dY)<MoveLen&&Next<shape[via].size()){
                MoveLen-=sqrt(dX*dX+dY*dY);
                Now++;
                Next++;
                if(Next>=shape[via].size())break;
                dX=shape[via][Next].first-shape[via][Now].first;
                dY=shape[via][Next].second-shape[via][Now].second;
                cos=dX/sqrt(dX*dX+dY*dY);
                sin=dY/sqrt(dX*dX+dY*dY);
                x=shape[via][Now].first;
                y=shape[via][Now].second;
                //cout<<"v_id="<<v_id<<": x="<<x<<", y="<<y<<endl;
            }
            if(Next<shape[via].size()){

                //cout<<v_id<<": size="<<shape[Via].size()<<"  now="<<Now<<" next="<<Next<<" "<<v_vel<<" "<<x<<" "<<y<<endl;
                //cout<<"("<<dX<<", "<<dY<<")"<<endl;
                x+=MoveLen*cos;
                y+=MoveLen*sin;
            }else if(Next==shape[via].size()){
                if(edge_via[via].find(next)==edge_via[via].end())break;
                via=edge_via[via][next];
                Now=0;Next=1;
                //cout<<"via"<<via<<" ,next"<<next<<endl;

                if(shape.find(via)==shape.end())break;
                //cout<<shape[via].size()<<endl;
            }
            
        }



        //cout<<"while end"<<endl;
        /*
        for(pair<int,double> Cpoint:__Lane_Conflict[via]){
            int Upper=ceil(T_Stop_Line+Cpoint.second+__Delta);
            int Lower=floor(T_Stop_Line+Cpoint.second-__Delta);
            __Conflict_Point_temp[Cpoint.first][Upper]=true;
            __Conflict_Point_temp[Cpoint.first][Lower]=true;
            
        }*/
        TempSol.push_back(make_tuple(via,c,TempDelay));
        SumTempDelay+=TempDelay;
    }
    return make_pair(SumTempDelay,TempSol);
}


pair<int,vector<tuple<string,Car*,int> > > Scheduler::GenerateValidSolution1(vector<Car*> comingVehicle,double CurrentTimeStep){
    double MaxDelay=20*StepPerSecond;
    vector<tuple<string,Car*,int> > TempSol(0);
    int SumTempDelay=0;
    int TempDelay=0;
    double v_vel=5.0;
    //double v_vel=FixedSpeed;
    //__Clear_Temp();
    bool successful=false;
    //random_device rd;
    //double T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
    //cout<<CurrentTimeStep<<endl;
    //default_random_engine eng(rd());
    //uniform_int_distribution<> distr(0,MaxDelay);
    for(Car* c:comingVehicle){
        string via="";
        string now=libtraci::Vehicle::getRoadID(c->getID());
        string next="";
        vector<string> Route=libtraci::Vehicle::getRoute(c->getID());
        for(int i=0;i<Route.size();i++){
            string e=Route[i];
            if(e==now&&i<Route.size()-1){
                via=edge_via[e][Route[i+1]];
                next=Route[i+1];
            }else if(e==now&&i==Route.size()-1){
                next="LAST";
            }
        }
        string v_id=c->getID();
        //cout<<"test "<<v_id<<" now  "<<now<<" via "<<via<<"   "<<shape[via].size()<<endl;
        if(next==""){
            continue;
        }
        /*
        if(shape[libtraci::Vehicle::getLaneID(v_id)].size()>0){
            cout<<"Shape  "<<shape[libtraci::Vehicle::getLaneID(v_id)].size()<<endl;
            via=libtraci::Vehicle::getLaneID(v_id);
        } 
        */
        if(via==""&&next=="LAST"){
            
            //cout<<edge_via[now].size()<<endl;
            /*
            if(shape[libtraci::Vehicle::getLaneID(v_id)].size()>0){
                cout<<S<<" "<<N<<" "<<W<<" "<<E<<endl;
                cout<<shape[libtraci::Vehicle::getLaneID(v_id)][0].first<<" "<<shape[libtraci::Vehicle::getLaneID(v_id)][0].second<<endl;
                cout<<shape[libtraci::Vehicle::getLaneID(v_id)][1].first<<" "<<shape[libtraci::Vehicle::getLaneID(v_id)][1].second<<endl;
            }
            */
            for(pair<string,string> p:edge_via[now]){
                via=p.second;
                next=p.first;
                break;
            }
        }
        
        
        //LaneDistance=libtraci::Lane::getLength(libtraci::Vehicle::getLaneID(c->getID()));
        double L_length     =libtraci::Lane::getLength(libtraci::Vehicle::getLaneID(c->getID()));
        double pos          =libtraci::Vehicle::getLanePosition(c->getID());
        
        LaneDistance=L_length-pos;
        
        v_vel=5.0;
        
        //v_vel=max(3.0,(double)floor(libtraci::Lane::getMaxSpeed(via)));

        __OPTP=0;
        double first_speed=(double)floor(libtraci::Vehicle::getSpeed(c->getID()));
        double end_speed=(double)floor(v_vel);
        double maxspeed=(double)floor(FixedSpeed);
        double lanelength=LaneDistance+(first_speed*2+1)*first_speed*2/40+(end_speed*2+1)*end_speed*2/40;
        if(lanelength>=2*(maxspeed*2+1)*maxspeed*2/40){
            lanelength-=2*(maxspeed*2+1)*maxspeed*2/40;
            __OPTP=lanelength/maxspeed+(maxspeed*2-first_speed-end_speed)/5;
            __OPTP*=StepPerSecond;
            __OPTP=(double)ceil(__OPTP);
        }else{
            double ms=max(end_speed,first_speed)+0.5;
            while(lanelength>2*(ms*2+1)*ms*2/40){
                ms+=0.5;
            }
            ms-=0.5;
            __OPTP=max(0.0,lanelength-2*(ms*2+1)*ms*2/40)/ms+(ms*2-first_speed-end_speed)/5;
            __OPTP*=StepPerSecond;
            __OPTP=(double)ceil(__OPTP);
        }
        double T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
        //T_Stop_Line=(double)ceil(T_Stop_Line);
        string lane=libtraci::Vehicle::getLaneID(c->getID());
        //cout<<lane<<"  "<<via<<endl;
        T_Stop_Line=(double)ceil(__Hmin+LastVehicleArrivalTime[lane]);
        TempDelay=(int)(T_Stop_Line-CurrentTimeStep-__OPTP);
        if(TempDelay<0){
            TempDelay=0;
            T_Stop_Line=(double)ceil(__Hmin+LastVehicleArrivalTime[lane]);
        }    
        successful=false;
        bool SubValid;
        //cout<<"while"<<endl;
        while(!successful){
            //TempDelay=distr(eng);
            TempDelay++;
            T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
            //if(c->getID()=="55")cout<<CurrentTimeStep<<" "<<__OPTP<<"  "<<c->getID()<<" "<<TempDelay<<endl;
            



            
            //while(T_Stop_Line<__Hmin+LastVehicleArrivalTime[via]){
            while(T_Stop_Line<__Hmin+LastVehicleArrivalTime[lane]){
                //TempDelay=distr(eng);
                TempDelay++;
                T_Stop_Line=CurrentTimeStep+__OPTP+TempDelay;
                cout<<c->getID()<<"  wrong   "<<TempDelay<<endl;
            }
            //cout<<lane<<endl;
            SubValid=true;
            // LaneConflict
            
            double t=T_Stop_Line;
            //cout<<via<<"  "<<shape[via].size()<<endl;
            double x=shape[via][0].first,y=shape[via][0].second;
            int Now=0,Next=1;
            
            //LaneConflict
            //cout<<"conflict"<<endl;
                
            string Via=via;
            ////////
            
            while(Next<shape[Via].size()){
                //cout<<Via<<endl;
                double dX=shape[Via][Next].first-x;
                double dY=shape[Via][Next].second-y;
                //cout<<dX<<" "<<dY<<endl;
                double cos=dX/sqrt(dX*dX+dY*dY);
                double sin=dY/sqrt(dX*dX+dY*dY);
                string VTYPE=libtraci::Vehicle::getTypeID(v_id);
                double v_length=libtraci::VehicleType::getLength(VTYPE);
                double v_width=libtraci::VehicleType::getWidth(VTYPE);
                double X=x-floor(W);
                double Y=y-floor(S);
                /*
                    3                     0
                    |--------------------|
                    |                    |
                    |                    | =====>>>>>>>
                    |                    |
                    |--------------------|
                    2                     1
                
                */
                vector<pair<double,double> > corner(4);
                /*
                corner[0].first=v_length/2;
                corner[0].second=v_width/2;
                corner[1].first=v_length/2;
                corner[1].second=-v_width/2;
                corner[2].first=-v_length/2;
                corner[2].second=-v_width/2;
                corner[3].first=-v_length/2;
                corner[3].second=v_width/2;
                */
                corner[0].first=0;
                corner[0].second=v_width/2;
                corner[1].first=0;
                corner[1].second=-v_width/2;
                corner[2].first=-v_length;
                corner[2].second=-v_width/2;
                corner[3].first=-v_length;
                corner[3].second=v_width/2;
                for(int i=0;i<4;i++){
                    //double fir=corner[i].first,sec=corner[i].second;
                    corner[i].first+=X;
                    corner[i].second+=Y;
                }
                vector<vector<bool> > grid(height,vector<bool>(width,false));
                vector<pair<double,double>> grid_corner(4);
                grid_corner[0]=make_pair(0.0,0.0);
                grid_corner[1]=make_pair(0.5,0.0);
                grid_corner[2]=make_pair(0.0,0.5);
                grid_corner[3]=make_pair(0.5,0.5);
                
                for(int i=0;i<height;i++)for(int j=0;j<width;j++){
                    for(pair<double,double> gc:grid_corner){
                        double Xcorner=((double)j)*0.5+gc.first-X;
                        double Ycorner=((double)i)*0.5+gc.first-Y;
                        double Xc=Xcorner,Yc=Ycorner;
                        Xcorner=Xc*cos+Yc*sin+X;
                        Ycorner=-Xc*sin+Yc*cos+Y;
                        if(Xcorner<=corner[0].first&&Xcorner>=corner[3].first&&
                            Ycorner<=corner[0].second&&Ycorner>=corner[1].second){
                                grid[i][j]=true;
                                //for(int Buf=-1;Buf<=1;Buf++)tile_time[i][j].push_back(t+Buf);
                                break;
                        }
                    }
                }
                


                for(int i=0;i<height;i++){
                    for(int j=0;j<width;j++){
                        if(grid[i][j]){
                            int Upper=ceil(t+__Delta);
                            //int Lower=floor(t-__Delta);
                            int Lower=floor(t-5);
                        
                            for(int T=Lower;T<=Upper;T++){
                                if(ConflictPoint[i][j].find(T)!=ConflictPoint[i][j].end()||ConflictPointTemp[i][j].find(T)!=ConflictPointTemp[i][j].end()){
                                    //cout<<"true"<<endl;
                                    if(ConflictPointTemp[i][j][T]!=c->getID()){
                                        //if(c->getID()=="78")cout<<T_Stop_Line<<"   "<<T<<" chan"<<c->getID()<<" "<<ConflictPointTemp[i][j][T]<<endl;
                                        SubValid=false;
                                        break;
                                    }
                                    //t==upperbound
                                }else if(T==Upper){
                                    //cout<<"false;"<<endl;
                                    SubValid=true;
                                }
                                
                            }
                        }
                        if(!SubValid)break;
                    }
                    if(!SubValid)break;
                }
                if(!SubValid)break;

                t+=1;
                // move V
                double MoveLen=0.1*v_vel;
                //cout<<"dis="<<sqrt(dX*dX+dY*dY)<<"   "<<MoveLen<<endl;
                while(sqrt(dX*dX+dY*dY)<MoveLen&&Next<shape[Via].size()){
                    MoveLen-=sqrt(dX*dX+dY*dY);
                    Now++;
                    Next++;
                    if(Next>=shape[Via].size())break;
                    dX=shape[Via][Next].first-shape[Via][Now].first;
                    dY=shape[Via][Next].second-shape[Via][Now].second;
                    cos=dX/sqrt(dX*dX+dY*dY);
                    sin=dY/sqrt(dX*dX+dY*dY);
                    x=shape[Via][Now].first;
                    y=shape[Via][Now].second;
                    //cout<<"v_id="<<v_id<<": x="<<x<<", y="<<y<<endl;
                }
                //cout<<t<<" : a :"<<X<<","<<Y<<endl;
                if(Next<shape[Via].size()){

                    //cout<<v_id<<": size="<<shape[Via].size()<<"  now="<<Now<<" next="<<Next<<" "<<v_vel<<" "<<x<<" "<<y<<endl;
                    //cout<<"("<<dX<<", "<<dY<<")"<<endl;
                    x+=MoveLen*cos;
                    y+=MoveLen*sin;
                }else if(Next==shape[Via].size()){
                    //cout<<Via<<endl;
                    //if(c->getID()=="2")cout<<"2 : "<<edge_via[Via].size()<<endl;
                    if(edge_via[Via].find(next)==edge_via[Via].end())break;
                    Via=edge_via[Via][next];
                    
                    //cout<<"change via "<<Via<<endl;
                    Now=0;Next=1;
                    //cout<<"via"<<via<<" ,next"<<next<<endl;

                    if(shape.find(Via)==shape.end())break;
                    //cout<<shape[via].size()<<endl;
                }
                //dX-=MoveLen*cos;
                //dY-=MoveLen*sin;
                
                
            }
            




            if(!SubValid){
                continue;
            }else{
                
                //if(c->getID()=="78")cout<<"TTTT    "<<t<<"    "<<T_Stop_Line<<endl;
                successful=true;
            }
        }
        //cout<<"TEST"<<endl;
        double t=T_Stop_Line;
        //if(c->getID()=="78")cout<<"TStopLine  "<<T_Stop_Line<<endl;
        double x=shape[via][0].first,y=shape[via][0].second;
        int Now=0,Next=1;
        string Via=via;
        //cout<<"mid"<<CurrentTimeStep<<"  "<<c->getID()<<endl;
        while(Next<shape[Via].size()){
            double dX=shape[Via][Next].first-x;
            double dY=shape[Via][Next].second-y;
            //cout<<dX<<" "<<dY<<endl;
            //double dX=shape[Via][Next].first-shape[Via][Now].first;
            //double dY=shape[Via][Next].second-shape[Via][Now].second;
            double cos=dX/sqrt(dX*dX+dY*dY);
            double sin=dY/sqrt(dX*dX+dY*dY);
            string VTYPE=libtraci::Vehicle::getTypeID(v_id);
            double v_length=libtraci::VehicleType::getLength(VTYPE);
            double v_width=libtraci::VehicleType::getWidth(VTYPE);
            double X=x-floor(W);
            double Y=y-floor(S);
            /*
                3                     0
                |--------------------|
                |                    |
                |                    | =====>>>>>>>
                |                    |
                |--------------------|
                2                     1
            
            */
            vector<pair<double,double> > corner(4);
            /*
            corner[0].first=v_length/2;
            corner[0].second=v_width/2;
            corner[1].first=v_length/2;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length/2;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length/2;
            corner[3].second=v_width/2;
            */
            corner[0].first=0;
            corner[0].second=v_width/2;
            corner[1].first=0;
            corner[1].second=-v_width/2;
            corner[2].first=-v_length;
            corner[2].second=-v_width/2;
            corner[3].first=-v_length;
            corner[3].second=v_width/2;
            for(int i=0;i<4;i++){
                //double fir=corner[i].first,sec=corner[i].second;
                corner[i].first+=X;
                corner[i].second+=Y;
            }
            vector<vector<bool> > grid(height,vector<bool>(width,false));
            vector<pair<double,double>> grid_corner(4);
            grid_corner[0]=make_pair(0.0,0.0);
            grid_corner[1]=make_pair(0.5,0.0);
            grid_corner[2]=make_pair(0.0,0.5);
            grid_corner[3]=make_pair(0.5,0.5);
            
            for(int i=0;i<height;i++)for(int j=0;j<width;j++){
                for(pair<double,double> gc:grid_corner){
                    double Xcorner=((double)j)*0.5+gc.first-X;
                    double Ycorner=((double)i)*0.5+gc.first-Y;
                    double Xc=Xcorner,Yc=Ycorner;
                    Xcorner=Xc*cos+Yc*sin+X;
                    Ycorner=-Xc*sin+Yc*cos+Y;
                    if(Xcorner<=corner[0].first&&Xcorner>=corner[3].first&&
                        Ycorner<=corner[0].second&&Ycorner>=corner[1].second){
                            grid[i][j]=true;
                            //for(int Buf=-1;Buf<=1;Buf++)tile_time[i][j].push_back(t+Buf);
                            break;
                    }
                }
            }
            
            

            for(int i=0;i<height;i++){
                for(int j=0;j<width;j++){
                    if(grid[i][j]){
                        int Upper=ceil(t+__Delta);
                        //int Lower=floor(t-__Delta);
                        int Lower=floor(t-5);
                        
                        for(int k=Lower;k<=Upper;k++){
                            if(ConflictPoint[i][j].find(k)!=ConflictPoint[i][j].end()){
                                if(ConflictPointTemp[i][j][k]!=c->getID()){
                                    cout<<"TMP     "<<t<<"   "<<ConflictPointTemp[i][j][k]<<" change!"<<c->getID()<<"   "<<i<<" "<<j<<" "<<k<<endl;
                                }
                            }
                            ConflictPointTemp[i][j][k]=c->getID();
                        }
                        
                    }
                }
                
            }
            

            t+=1;
            // move V
            double MoveLen=0.1*v_vel;
            //cout<<"dis="<<sqrt(dX*dX+dY*dY)<<"   "<<MoveLen<<endl;
            while(sqrt(dX*dX+dY*dY)<MoveLen&&Next<shape[Via].size()){
                MoveLen-=sqrt(dX*dX+dY*dY);
                Now++;
                Next++;
                if(Next>=shape[Via].size())break;
                dX=shape[Via][Next].first-shape[Via][Now].first;
                dY=shape[Via][Next].second-shape[Via][Now].second;
                cos=dX/sqrt(dX*dX+dY*dY);
                sin=dY/sqrt(dX*dX+dY*dY);
                x=shape[Via][Now].first;
                y=shape[Via][Now].second;
                //cout<<"v_id="<<v_id<<": x="<<x<<", y="<<y<<endl;
            }
            //cout<<t<<": b :"<<X<<","<<Y<<endl;
            if(Next<shape[Via].size()){

                //cout<<v_id<<": size="<<shape[Via].size()<<"  now="<<Now<<" next="<<Next<<" "<<v_vel<<" "<<x<<" "<<y<<endl;
                //cout<<"("<<dX<<", "<<dY<<")"<<endl;
                x+=MoveLen*cos;
                y+=MoveLen*sin;
            }else if(Next==shape[Via].size()){
                
                if(edge_via[Via].find(next)==edge_via[Via].end())break;
                Via=edge_via[Via][next];
                Now=0;Next=1;

                //cout<<"via"<<via<<" ,next"<<next<<endl;

                if(shape.find(Via)==shape.end())break;
                //cout<<shape[via].size()<<endl;
            }
            
        }
        //if(c->getID()=="78")cout<<"TTTT  2  "<<t<<endl;


        //cout<<"while end"<<endl;
        /*
        for(pair<int,double> Cpoint:__Lane_Conflict[via]){
            int Upper=ceil(T_Stop_Line+Cpoint.second+__Delta);
            int Lower=floor(T_Stop_Line+Cpoint.second-__Delta);
            __Conflict_Point_temp[Cpoint.first][Upper]=true;
            __Conflict_Point_temp[Cpoint.first][Lower]=true;
            
        }
        */
        if(comingVehicle.size()>1){
            
            //cout<<CurrentTimeStep<<" "<<__OPTP<<" "<<c->getID()<<"  "<<TempDelay<<endl;
        }
        TempSol.push_back(make_tuple(via,c,TempDelay));
        SumTempDelay+=TempDelay;
        //cout<<"valid "<<T_Stop_Line<<endl;
    }
    
    return make_pair(SumTempDelay,TempSol);
}