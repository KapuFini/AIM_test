#include "Car.h"
#include "RoadController.h"
#include "Overall.h"
using namespace std;



map<string,double > Overall::getLaneSpeed(){
    return lane_speed;

}
double Car::Arrivaltime(){
    return this->arrivaltime;
}
Car::Car(string id,RoadController* _rc){
    ID=id;
    optimistic=true;
    route=libtraci::Vehicle::getRoute(id);
    
    rc=_rc;
    accept=false;
    reservation=false;
    arrivaltime=-1;
    typeID=libtraci::Vehicle::getTypeID(id);
    libtraci::VehicleType::setImperfection(typeID,0.0);
    libtraci::VehicleType::setMinGap(typeID,0.3);
    libtraci::Vehicle::setSpeedMode(id,7);
    libtraci::VehicleType::setMinGapLat(typeID,0.0);
    accel=libtraci::VehicleType::getAccel(typeID);
    decel=libtraci::VehicleType::getDecel(typeID);
    //libtraci::Vehicle::setLaneChangeMode(id,0);
}
Car::Car(string id,RoadController *_rc,vector<string> rou){
    ID=id;
    route=rou;
    optimistic=true;
    accept=false;
    arrivaltime=-1;
    reservation=false;
    rc=_rc;
    libtraci::Vehicle::setRoute(ID,route);
    typeID=libtraci::Vehicle::getTypeID(id);
    accel=libtraci::VehicleType::getAccel(typeID);
    decel=libtraci::VehicleType::getDecel(typeID);
    libtraci::VehicleType::setImperfection(typeID,0.0);
    //libtraci::VehicleType::setMinGap(typeID,0.3);
    libtraci::VehicleType::setMinGapLat(typeID,0.3);
    libtraci::Vehicle::setSpeedMode(ID,7);
    //libtraci::Vehicle::setLaneChangeMode(id,0);
    
}

Car::~Car(){
    
}
void Car::setRoute(vector<string> rou){
    route=rou;
    libtraci::Vehicle::setRoute(ID,route);
}

void Car::setEnterTime(double T){
    this->arrivaltime=T;
}
void Car::setOPTP(double _OPTP){
    this->OPTP=_OPTP;
}

string Car::getID(){
    return ID;
}
double Car::getAccel(){
    return accel;
}
double Car::getDecel(){
    return decel;
}

double accellen(double speed){
    double TempSpeed=((double)((int)(speed*2)))*0.5;
    return TempSpeed*2*(TempSpeed*2+1)/40;
}
void Car::step(int t){
    //if(t<=2)cout<<ID<<"speed"<<libtraci::Vehicle::getSpeed(ID)<<endl;
    string L_id=libtraci::Vehicle::getLaneID(ID);
    double maxspeed=15;
    double lane_speed=5.0;


    
    
    
    
    //lane_speed=max(3.0,(double)floor(libtraci::Lane::getMaxSpeed(via)));




    //if(L_id!="")maxspeed=libtraci::Lane::getMaxSpeed(L_id);
    if(arrivaltime>t){
        string L_id         =libtraci::Vehicle::getLaneID(ID);
    
        double L_length     =libtraci::Lane::getLength(L_id);
        double pos          =libtraci::Vehicle::getLanePosition(ID);
        double last_length  =L_length-pos;
        double speed=libtraci::Vehicle::getSpeed(ID);
        double TempSpeed=((double)((int)(speed*2)))*0.5;
            
        double aclen=0;
        double Time=0;
        double minsp=TempSpeed;
        if(minsp==speed){
            minsp=TempSpeed-0.5;
        }
        double maxsp=TempSpeed+0.5;
        if(speed<=maxspeed){
            aclen=accellen(maxspeed)-accellen(TempSpeed)+accellen(maxspeed)+accellen(lane_speed);
            Time=(maxspeed-TempSpeed+maxspeed+lane_speed)*2;
            if(last_length>aclen){
                if(last_length-aclen<=maxspeed*0.1*(arrivaltime-t-Time)){
                    //  _----_
                    //_-      -_      _-
                    //          -____-
                    //if(ID=="64")cout<<"A"<<"  "<<last_length-aclen<<endl;
                    speed=min(TempSpeed+0.5,maxspeed);
                    if(TempSpeed+0.5>=maxspeed){

                        speed=max(maxspeed-0.5,(last_length-aclen)*10/((double)ceil((last_length-aclen)*10/maxspeed)));
                    }
                }else{
                    //  _----_
                    //_-      ---------
                    //          
                    //if(ID=="64")cout<<"B"<<endl;
                    speed=min(TempSpeed+0.5,maxspeed);
                    
                }
            }else{
                if(TempSpeed==speed){
                    TempSpeed=speed-0.5;
                    TempSpeed=max(0.0,TempSpeed);
                }
                aclen=accellen(TempSpeed)+accellen(lane_speed);
                Time=(TempSpeed+lane_speed)*2;
                if(last_length>aclen&&last_length-aclen<=(arrivaltime-t-Time)*0.1*(TempSpeed+0.5)){
                    //--_
                    //   -_      _-
                    //     -____-
                    //if(ID=="64")cout<<"C    Time:"<<Time<<"   lane:"<<last_length-aclen<<"   speed:"<<speed<<" ";
                    
                    if(last_length-aclen==0){
                        speed=TempSpeed;
                        if(lane_speed/0.5>=Time){
                            speed=0.5;
                        }
                    }else{
                        if(last_length-aclen<=(arrivaltime-t-Time)*0.1*(speed)){
                            //speed=max(minsp,(last_length-aclen))
                            //last_length-aclen
                            //speed=max(minsp,(last_length-aclen)*10/(arrivaltime-t-Time));
                            speed=minsp;
                            if(speed==0){
                                speed=min(0.5,(last_length-aclen)*10);
                            }
                        }else{
                            speed=TempSpeed+0.5;
                        }
                        
                    }
                    
                    //cout<<speed<<endl;
                }else{
                    //----_
                    //     -____
                    //     
                    if(TempSpeed>=lane_speed){
                        //cout<<"D"<<endl;
                        aclen=accellen(TempSpeed)-accellen(lane_speed-0.5);
                        Time=(TempSpeed-lane_speed+0.5)*2;
                        double lastTime=arrivaltime-t-Time;
                        double aclen1=accellen(TempSpeed)-accellen(lane_speed);
                        double Time1=(TempSpeed-lane_speed)*2;
                        double lastTime1=arrivaltime-t-Time1;
                        //cout<<"pattern1  :"<<Time<<"   "<<lastTime<<"  "<<last_length<<"  "<<aclen<<"  "<<(last_length-aclen)*10/lastTime<<endl;
                        //cout<<"pattern2  :"<<Time1<<"   "<<lastTime1<<"  "<<last_length<<"  "<<aclen1<<"  "<<(last_length-aclen1)*10/lastTime1<<endl;
                        /*
                        if(last_length-aclen1>=0){
                            
                        }else{
                            
                        }
                        */
                        if(last_length-aclen1>=lastTime1*0.1*minsp&&lastTime1>0){
                            if(last_length-aclen1-0.1*(TempSpeed+0.5)>=(lastTime1-1)*0.1*minsp&&lastTime1-1>0){
                                speed=min(maxsp,(last_length-aclen1)*10/lastTime1);
                            }else{
                                speed=min(TempSpeed+0.5,(last_length-aclen1)*10/lastTime1);
                            }
                            //if(TempSpeed+0.5==speed)
                        }else{
                            speed=TempSpeed;
                            if(Time1==1&&lastTime1==0){
                                speed=last_length*10;
                            }
                        }
                        /*
                        if(last_length-aclen>=lastTime*0.1*minsp&&lastTime>0){
                            speed=min(TempSpeed+0.5,(last_length-aclen)*10/lastTime);
                        }else{
                            cout<<TempSpeed<<endl;
                            speed=TempSpeed;
                            if(Time==1&&lastTime==0){
                                speed=last_length*10;
                            }
                        }
                        */
                    }else{
                        //if(ID=="64")cout<<"E"<<endl;
                        TempSpeed=((double)((int)(speed*2)))*0.5;
                        aclen=accellen(lane_speed)-accellen(TempSpeed);
                        Time=(lane_speed-TempSpeed)*2;
                        double lastTime=arrivaltime-t-Time;
                        //cout<<Time<<"   "<<lastTime<<"  "<<last_length<<"  "<<aclen<<endl;
                        
                        if(last_length-aclen<=lastTime*0.1*(TempSpeed+0.5)&&lastTime>0){
                            speed=max(minsp,(last_length-aclen)*10/lastTime);
                        }else{
                            speed=TempSpeed+0.5;
                        }

                    }


                }

                //if()
            }
        }
        //(maxspeed*2+1)*maxspeed*2/40-(speed*2+1)*speed*;
        



        /*
        if(speed<=lane_speed){
            aclen=(lane_speed*2+1)*(lane_speed*2)/40-(TempSpeed*2+1)*TempSpeed*2/40;
            double Time=(lane_speed-TempSpeed)*2;
            //if(ID=="0")cout<<TempSpeed<<"  aclen : "<<aclen<<" Time : "<<Time<<endl; 
            if(arrivaltime-Time-t>0){
                if(abs((last_length-aclen)*10/(arrivaltime-Time-t)-TempSpeed)<=0.5){
                    speed=(last_length-aclen)*10/(arrivaltime-Time-t);

                }else{
                    if((last_length-aclen)*10/(arrivaltime-Time-t)-TempSpeed>0){
                        speed=TempSpeed+0.5;
                    }else{
                        if(speed>TempSpeed){
                            speed=TempSpeed;
                        }else{
                            speed-=0.5;
                        }
                        //speed=TempSpeed
                    }
                }
            }else //if(arrivaltime==Time+t)
            {
                speed=TempSpeed+0.5;
            }

           
            
        }else{
            //double TempSpeed=((double)((int)(speed*2)))*0.5;
            
            aclen=-(lane_speed*2+1)*(lane_speed*2)/40+(TempSpeed*2+1)*TempSpeed*2/40;

            double Time=(TempSpeed-lane_speed)*2;
            //if(ID=="0")cout<<TempSpeed<<"  aclen : "<<aclen<<" Time : "<<Time<<endl; 
            if(arrivaltime-Time-t>0){
                if(abs((last_length-aclen)*10/(arrivaltime-Time-t)-TempSpeed)<=0.5){
                    speed=(last_length-aclen)*10/(arrivaltime-Time-t);

                }else{
                    if((last_length-aclen)*10/(arrivaltime-Time-t)-TempSpeed>0){
                        speed=TempSpeed+0.5;
                    }else{
                        if(speed>TempSpeed){
                            speed=TempSpeed;
                        }else{
                            speed-=0.5;
                        }
                        //speed=TempSpeed
                    }
                }
            }else{
                if(speed>TempSpeed){
                    speed=TempSpeed;
                }else{
                    speed-=0.5;
                }
                
            }
        }
        */
        /*
        if((t-arrivaltime)*libtraci::Vehicle::getSpeed(ID)<last_length){
            speed=min(libtraci::Vehicle::getSpeed(ID)+0.5,maxspeed);


        }else{
            speed=min(libtraci::Vehicle::getSpeed(ID))-0.5;
        }
        */
        //double speed=(last_length/((double)(arrivaltime-t)))*10;
        pair<string,double> leader=libtraci::Vehicle::getLeader(ID);
        pair<string,double> follower=libtraci::Vehicle::getFollower(ID);
        
        if(leader.first!=""){
            double leadersp=libtraci::Vehicle::getSpeed(leader.first);
            //leadersp=;
            
            double LeaderTmpSpeed=((double)((int)(leadersp*2)))*0.5;
            leadersp=min(LeaderTmpSpeed-0.5,leadersp*1);
            LeaderTmpSpeed=((double)((int)(leadersp*2)))*0.5;
            LeaderTmpSpeed=max(0.0,LeaderTmpSpeed);
            if(leader.second<=0.5+libtraci::Vehicle::getSpeed(ID)*0.1+(TempSpeed*2+1)*TempSpeed*2/40-(LeaderTmpSpeed*2+1)*LeaderTmpSpeed*2/40){
            //if(leader.second<=0.1+libtraci::Vehicle::getSpeed(ID)*0.1+(TempSpeed*2+1)*TempSpeed*2/40){
                if(libtraci::Vehicle::getSpeed(ID)!=TempSpeed){
                    speed=TempSpeed;
                }else{
                    speed=max(0.0,TempSpeed-0.5);
                }
                //if(ID=="2"){cout<<TempSpeed<<"danger :  "<<leader.first<<" "<<leader.second<<endl;}
                //speed=min(speed,leader.second*10);
                //speed=min(libtraci::Vehicle::getSpeed(leader.first),speed);
            }
            //if(ID=="2"){cout<<TempSpeed<<"confirm :  "<<leader.first<<" "<<leader.second<<endl;}
            /*
            else if(leader.second<=10){
                speed=min(speed,libtraci::Vehicle::getSpeed(leader.first));
            }
            */
        }
        
        libtraci::Vehicle::setSpeedMode(ID,0);
        libtraci::Vehicle::setSpeed(ID,speed);
        /*
        if(ID=="94"||ID=="43"){
            cout<<arrivaltime<<"   "<<t<<"   "<<libtraci::Vehicle::getLaneID(ID)<<endl;
            cout<<"ID:"<<ID<<"   t:"<<t<<" "<<last_length<<"    now:"<<libtraci::Vehicle::getSpeed(ID)<<"   next "<<speed<<endl;
        }*/

        
    }else{
        //cout<<ID<<" "<<t<<" "<<libtraci::Vehicle::getLaneID(ID)<<endl;;
        libtraci::Vehicle::setSpeedMode(ID,0);
        double speed1=libtraci::Vehicle::getSpeed(ID);
        double TempSpeed1=((double)((int)(speed1*2)))*0.5;
         
        libtraci::Vehicle::setSpeed(ID,min(lane_speed,TempSpeed1+0.5));
        pair<string,double> leader=libtraci::Vehicle::getLeader(ID);
        pair<string,double> follower=libtraci::Vehicle::getFollower(ID);
        
        if(leader.first!=""){
            
            double leadersp=libtraci::Vehicle::getSpeed(leader.first);
            //leadersp=;
            
            double LeaderTmpSpeed=((double)((int)(leadersp*2)))*0.5;
            leadersp=min(LeaderTmpSpeed-0.5,leadersp*1);
            LeaderTmpSpeed=((double)((int)(leadersp*2)))*0.5;
            LeaderTmpSpeed=max(0.0,LeaderTmpSpeed);
            double speed=libtraci::Vehicle::getSpeed(ID);
            double TempSpeed=((double)((int)(speed*2)))*0.5;
            //libtraci::Vehicle::setSpeed(ID,min(TempSpeed))
            /*if(ID=="94"||ID=="43"){
                cout<<t<<" "<<leader.first<<" "<<leader.second<<" "<<libtraci::Vehicle::getSpeed(leader.first)<<"  "<<libtraci::Vehicle::getSpeed(ID)<<endl;
                cout<<leader.second<<" "<<0.1+libtraci::Vehicle::getSpeed(ID)*0.1+(TempSpeed*2+1)*TempSpeed*2/40-(LeaderTmpSpeed*2+1)*LeaderTmpSpeed*2/40<<endl;
            }*/
            if(leader.second<=0.5+libtraci::Vehicle::getSpeed(ID)*0.1+(TempSpeed*2+2)*(TempSpeed*2+1)/40-(LeaderTmpSpeed*2+1)*LeaderTmpSpeed*2/40){
            //if(leader.second<=0.1+libtraci::Vehicle::getSpeed(ID)*0.1+(TempSpeed*2+1)*TempSpeed*2/40){
                
                
                if(libtraci::Vehicle::getSpeed(ID)!=TempSpeed){
                    speed=TempSpeed;
                }else{
                    speed=max(0.0,TempSpeed-0.5);
                }
                libtraci::Vehicle::setSpeedMode(ID,0);
                libtraci::Vehicle::setSpeed(ID,speed);

                //if(ID=="2"){cout<<TempSpeed<<"danger :  "<<leader.first<<" "<<leader.second<<endl;}
                //speed=min(speed,leader.second*10);
                //speed=min(libtraci::Vehicle::getSpeed(leader.first),speed);
            }
            /*
            if(leader.second<=lane_speed*0.1){
                libtraci::Vehicle::setSpeed(ID,min(lane_speed,leader.second*9));
                //speed=min(libtraci::Vehicle::getSpeed(leader.first),speed);
            }
            */
            /*
            else if(leader.second<=10){
                speed=min(speed,libtraci::Vehicle::getSpeed(leader.first));
            }
            */

        }
    }
    pair<string,double> leader=libtraci::Vehicle::getLeader(ID);
    if(ID=="39"){
        cout<<t<<" "<<libtraci::Vehicle::getSpeed(ID)<<endl;
        if(leader.first!="")cout<<t<<" "<<leader.first<<" "<<leader.second<<" "<<libtraci::Vehicle::getSpeed(leader.first)<<"  "<<libtraci::Vehicle::getSpeed(ID)<<endl;
    }
}
void Car::setDelay(int _delay){
    this->delay=(((double)_delay)-begin-OPTP)/10;
    //cout<<ID<<"  "<<delay<<endl;
}

double Car::getDelay(){
    return delay;
}
void Car::setbegin(double _begin){
    begin=_begin;
}