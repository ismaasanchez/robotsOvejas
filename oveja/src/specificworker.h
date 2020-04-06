/*
 *    Copyright (C)2019 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QPointF>
#include <BrainTree.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <QTime>
#include <QString>
#include <Scheduler/Scheduler.h>
#include <thread>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
    void initialize(int period);

    void compute();
    void readRobotState();
    void loadPoints();
    void createTreeManually(BrainTree::BehaviorTree &btree);
    void createTree1(BrainTree::BehaviorTree &btree);
    void createTree2(BrainTree::BehaviorTree &btree);
    void createTree3(BrainTree::BehaviorTree &btree);
    void createTree4(BrainTree::BehaviorTree &btree);
    int getCoordXFood();
    int getCoordYFood();
    int getCoordXWater();
    int getCoordYWater();
    QPointF getFoodDispenser();
    QPointF getWaterDispenser();
    void escribirCoords(std::string nRobot, float x, float y); 
    int getIntName(std::string nRobot);
    void crearArboles();
    void initCron();
    std::string robotName;

    QPointF foodDispenser;
    QPointF waterDispenser;
    RoboCompGenericBase::TBaseState bState;
	std::shared_ptr<InnerModel> innerModel;
    BrainTree::BehaviorTree btree;
    BrainTree::BehaviorTree btree1;
    BrainTree::BehaviorTree btree2;
    BrainTree::BehaviorTree btree3;
    BrainTree::BehaviorTree btree4;
    QTime timeAction;
private:    
};

class ActionInitSleep : public BrainTree::Node
{
    public:
        ActionInitSleep(SpecificWorker *x)
        {
            qDebug() << "Constructor initSleep";
            this->sp = x;
        }
        Status update() override 
        {
            
            if(first_epoch)
            { 
             // sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "Empiezo a dormir";
                reloj.restart();
                first_epoch = false;
                sp->differentialrobot_proxy -> setSpeedBase(0,0);
                return Node::Status::Running;        
            }
            else
            {
                qDebug() << "Durmiendo...";
                if(reloj.elapsed() > 4000)
                {
                    qDebug() << "Dormi suficiente";
                    first_epoch = true;
                  //  qDebug() << "Posicion X en findormir" << sp->bState.x;
                  //  qDebug() << "Posicion Y en findormir" << sp->bState.z;
                    return Node::Status::Success;       
                }
                else
                {
                 //   qDebug() << "Necesito seguir durmiendo";
                    return Node::Status::Running;
                }
                    
            }
        }
    private:
        bool first_epoch = true;
        QTime reloj;
        SpecificWorker* sp;
};

class ActionStandToEat : public BrainTree::Node 
{
    public:
        ActionStandToEat(SpecificWorker *x)
        {
            this->sp = x;
            qDebug() << "Constructor StandToEat";
        }
        Status update() override 
        {
            qDebug() << "Posicion X en standtoeat" << sp->bState.x;
            qDebug() << "Posicion Y en standtoeat" << sp->bState.z;
            qDebug() << "Posicionandome hacia el comedero";
            QPointF t;
            t = sp->getFoodDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform(sp->robotName.c_str(), QVec::vec3(t.x(),0,t.y()), "world");
            angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
            qDebug() << "Angulo = " << angle;
            qDebug() << "Posicion X en standtoeat" << sp->bState.x;
            qDebug() << "Posicion Y en standtoeat" << sp->bState.z;
            qDebug() << "------------------------------------------------";
            if(fabs(angle) < 0.001)
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "Stand to eat --------> SUCCESS";
                return Node::Status::Success;
            }else
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,angle);  
                qDebug() << "Stand to eat --------> RUNNING";
                return Node::Status::Running;
            }
        }
    private:
        SpecificWorker* sp;
};

class ActionGoToEat : public BrainTree::Node 
{
    public:
        ActionGoToEat(SpecificWorker *x)
        {
            qDebug() << "Constructor GoToEat";
            this->sp = x;
            esquivar = false;
        }
        Status update() override 
        {
            //qDebug() << "De camino al comedero";
            float coordX;
            float coordY;
            coordX = sp->getCoordXFood();
            coordY = sp->getCoordYFood();
            if((((coordX - sp->bState.x) < 20) && (coordX - sp->bState.x) > -20) && (((coordY - sp->bState.z) < 20) && (coordY - sp->bState.z) > -20))
	        {
	        	sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "Go to eat --------> SUCCESS";
	        	return Node::Status::Success;
	        }
	        else
	        {
		        //comprobar obtaculos
                    //
                RoboCompLaser::TLaserData ldata = sp->laser_proxy->getLaserData();
                float center = ldata[ldata.size()/2].dist;
                float right = ldata.front().dist;
                //sort laser data from small to large distances using a lambda function.
                std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

                if((ldata.front().dist < 450 || center < 450))
	            {
 		            sp->differentialrobot_proxy->setSpeedBase(0, -0.6);
                    qDebug() << " Comprobando si choco ";
                    if(right > 450){
                        esquivar = true;
                    }
                    qDebug() << " EMPIEZO A RODEAR ";
                    return rodearObj(center,right);
	            }

                if(esquivar){
                    qDebug() << "ME tengo que colocar";
                    return colocarse();
                    
                }else
                {
		            sp->differentialrobot_proxy -> setSpeedBase(500,0);
                    qDebug() << "Go to eat --------> RUNNING";
                    return Node::Status::Running;
                }
	        }	
        }
        Status colocarse()
        {
            QPointF t;
            t = sp->getFoodDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform(sp->robotName.c_str(), QVec::vec3(t.x(),0,t.y()), "world");
            angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
            qDebug() << "Angulo = " << angle;
            if( fabs(angle) < 0.001)
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "colocarse : estoy colocado";
                esquivar = false;
                return Node::Status::Running;
            }else
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,angle);  
                qDebug() << "colocarse : colocandome";
                return Node::Status::Running;
            }
        }

        Status rodearObj(float c, float r)
        {
            
            if(r > 400 && r < 500 && c > 450)
            {
                qDebug() << "rodear : avanzo ";
                sp->differentialrobot_proxy -> setSpeedBase(100,0);
                esquivar = true;
                return Node::Status::Running;
            }
            else
            {
                if(r < 400 || c < 350)
		        {
                    qDebug() << "rodear : giro izq ";
				    sp->differentialrobot_proxy -> setSpeedBase(100,-0.6);
                    return Node::Status::Running;
		        }
		        else if(r > 500)
		        {
                    qDebug() << "rodear giro derecha";
			    	sp->differentialrobot_proxy -> setSpeedBase(100,0.25);
                    return Node::Status::Running;
		        }
            }
        }
    private:
        SpecificWorker* sp;
        bool esquivar;   
};
class ActionInitEat : public BrainTree::Node
{
    public:
        ActionInitEat()
        {
            qDebug() << "Constructor initEat";
        }
        Status update() override 
        {
            if(first_epoch)
            { 
                qDebug() << "Empiezo a comer";
                reloj.restart();
                first_epoch = false;
                return Node::Status::Running;        
            }
            else
            {
                qDebug() << "Comiendo...";
                if(reloj.elapsed() > 4000)
                {
                    qDebug() << "Comi suficiente";
                    first_epoch = true;

                    return Node::Status::Success;       
                }
                else
                {
                    qDebug() << "Necesito seguir comiendo";
                    return Node::Status::Running;
                }
                    
            }
        }
    private:
        bool first_epoch = true;
        QTime reloj;
};     


class ActionStandToDrink : public BrainTree::Node 
{
    public:
        ActionStandToDrink(SpecificWorker* x)
        {
            qDebug() << "Constructor StandToDrink";
            this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "StandToDrink";
            QPointF t;
            t = sp->getWaterDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform(sp->robotName.c_str(), QVec::vec3(t.x(),0,t.y()), "world");
            angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
            qDebug() << "Angulo = " << angle;
            if( fabs(angle) < 0.001)
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "Stand to drink --------> SUCCESS";
                return Node::Status::Success;
            }else
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,angle);  
                qDebug() << "Stand to drink --------> RUNNING";
                return Node::Status::Running;
            }
        }
    private:
        SpecificWorker* sp;
};

class ActionGoToDrink : public BrainTree::Node 
{
    public:
        ActionGoToDrink(SpecificWorker* x)
        {
            qDebug() << "Constructor GoToDrink";
            this->sp = x;
            esquivar = false;
        }
        Status update() override 
        {
            qDebug() << "GoToDrink";
            float coordX;
            float coordY;
            coordX = sp->getCoordXWater();
            coordY = sp->getCoordYWater();
            if((((coordX - sp->bState.x) < 20) && (coordX - sp->bState.x) > -20) && (((coordY - sp->bState.z) < 20) && (coordY - sp->bState.z) > -20))
	        {
	        	sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "Go to drink --------> SUCCESS";
	        	return Node::Status::Success;
	        }
	        else
	        {       //comprobar obtaculos
                    //
                RoboCompLaser::TLaserData ldata = sp->laser_proxy->getLaserData();
                float center = ldata[ldata.size()/2].dist;
                float right = ldata.front().dist;
                //sort laser data from small to large distances using a lambda function.
                std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

                if((ldata.front().dist < 350 || center < 350))
	            {
 		            sp->differentialrobot_proxy->setSpeedBase(0, -0.6);
                    qDebug() << " Comprobando si choco ";
                    if(right > 450){
                        esquivar = true;
                    }
                    qDebug() << " EMPIEZO A RODEAR ";
                    return rodearObj(center,right);
	            }

                if(esquivar){
                    qDebug() << "ME tengo que colocar";
                    return colocarse();
                    
                }else
                {
		            sp->differentialrobot_proxy -> setSpeedBase(500,0);
                    qDebug() << "Go to eat --------> RUNNING";
                    return Node::Status::Running;
                }
            }	
        }
        Status colocarse()
        {
            QPointF t;
            t = sp->getWaterDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform(sp->robotName.c_str(), QVec::vec3(t.x(),0,t.y()), "world");
            angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
            qDebug() << "Angulo = " << angle;
            if( fabs(angle) < 0.001)
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,0);
                qDebug() << "colocarse : estoy colocado";
                esquivar = false;
                return Node::Status::Running;
            }else
            {
                sp->differentialrobot_proxy -> setSpeedBase(0,angle);  
                qDebug() << "colocarse : colocandome";
                return Node::Status::Running;
            }
        }
        Status rodearObj(float c, float r)
        {
            
            if(r > 400 && r < 500 && c > 450)
            {
                qDebug() << "rodear : avanzo ";
                sp->differentialrobot_proxy -> setSpeedBase(100,0);
                esquivar = true;
                return Node::Status::Running;
            }
            else
            {
                if(r < 400 || c < 350)
		        {
                    qDebug() << "rodear : giro izq ";
				    sp->differentialrobot_proxy -> setSpeedBase(100,-0.6);
                    return Node::Status::Running;
		        }
		        else if(r > 500)
		        {
                    qDebug() << "rodear giro derecha";
			    	sp->differentialrobot_proxy -> setSpeedBase(100,0.25);
                    return Node::Status::Running;
		        }
            }
        }
        
    private:
        SpecificWorker* sp;
        bool esquivar;
};

class ActionInitDrink : public BrainTree::Node
{
    public:
        ActionInitDrink()
        {
            qDebug() << "Constructor initDrink";
        }
        Status update() override 
        {
            if(first_epoch)
            { 
                qDebug() << "Empiezo a beber";
                reloj.restart();
                first_epoch = false;
                return Node::Status::Running;        
            }
            else
            {
                qDebug() << "Bebiendo...";
                if(reloj.elapsed() > 4000)
                {
                    qDebug() << "Bemi suficiente";
                    first_epoch = true;
                    return Node::Status::Success;       
                }
                else
                {
                    qDebug() << "Necesito seguir bebiendo";
                    return Node::Status::Running;
                }
                    
            }
        }
    private:
        bool first_epoch = true;
        QTime reloj;
};

class ActionInitWalk : public BrainTree::Node
{
    public:
        ActionInitWalk(SpecificWorker* x)
        {
            qDebug() << "Constructor initWalk";
            this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "initWalk";
            sp->timeAction.restart();
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
};
class ActionWalk : public BrainTree::Node 
{
    public:
        ActionWalk(SpecificWorker* x)
        {
            qDebug() << "Constructor Walk";
            this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "Walking";
            int waitingTime = 30000; // 15 seconds sleeping
            if(sp->timeAction.elapsed() > waitingTime){
                sp->differentialrobot_proxy->setSpeedBase(0,0);
                return Node::Status::Success;
            }
            else
            {
                RoboCompLaser::TLaserData ldata = sp->laser_proxy->getLaserData();
                //sort laser data from small to large distances using a lambda function.
                std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
    
                if( ldata.front().dist < 300)
	            {
                    /*int nr = rand() % 2;
                    if(nr == 0){
                        nr = -1;
                    }*/
 		            sp->differentialrobot_proxy->setSpeedBase(5, 0.6);
		            usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	            }
                else
                {
                  /*  int nrand = rand() % 2;
                    if(nrand  == 0 )
                    {
                        sp->differentialrobot_proxy->setSpeedBase(700, 0);
                    }
                    else
                    {*/
                        sp->differentialrobot_proxy->setSpeedBase(700,0.05);
                 //   }
                    
                }
                return Node::Status::Running;
            }
        }       
    private:
        SpecificWorker* sp;
        
};

#endif
