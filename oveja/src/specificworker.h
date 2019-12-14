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
#include <math.h>
#include <QTime>
#include <QString>


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
    void createTreeManually(BrainTree::BehaviorTree btree);
    int getCoordXFood();
    int getCoordYFood();
    int getCoordXWater();
    int getCoordYWater();
    QPointF getFoodDispenser();
    QPointF getWaterDispenser();
    
    QPointF foodDispenser;
    QPointF waterDispenser;
    RoboCompGenericBase::TBaseState bState;
	std::shared_ptr<InnerModel> innerModel;
    BrainTree::BehaviorTree btree;

    QTime timeAction;
private:    
};

class ActionInitSleep : public BrainTree::Node
{
    public:
        ActionInitSleep(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor initSleep";
          //  this->sp = x;
        }
        Status update() override 
        {
          /*  if(first_epoch)
            { 
                reloj.restart();
                first_epoch = false;
                return Node::Status::Running;        
            }
            else
            {
                if(reloj.elapsed() > 4000)
                {
                    first_epoch = false;
                    return Node::Status::Success;       
                }
                else
                    return Node::Status::Running;
            }
            
            qDebug() << "InitSleep";
            //sp->timeAction.restart();
            //return Node::Status::Success;
            */
           qDebug() << "Durmiendo";
           return Node::Status::Success;
        }
    private:
     //   SpecificWorker* sp;
     //   bool first_epoch = true;
     //   QTime reloj;
};

class ActionSleep : public BrainTree::Node 
{
    public:
        ActionSleep(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor sleep";
          //  this->sp = x;
        }
        Status update() override 
        {
         /*   qDebug() << "Entro a sleep";
            int waitingTime = 10000; // 10 seconds sleeping
            qDebug()<< "Entro al if" << sp->timeAction.elapsed();
            if(sp->timeAction.elapsed() > waitingTime){
                qDebug() << "Estoy dentro del if";
                int seg = sp->timeAction.elapsed() / 1000;
                qDebug() << "He estado durmiendo durante " << seg << " segundos.";
                return Node::Status::Success;
            }
            else
            {
                qDebug() << "Antes de Running";
                return Node::Status::Running;
                qDebug() << "Despues de Running";
            } 
            qDebug() << "Sleep";
            */
           qDebug() << "Durmiendo";
           return Node::Status::Success;
        }
    private:
     //   SpecificWorker* sp;
    
};

class ActionStandToEat : public BrainTree::Node 
{
    public:
        ActionStandToEat(/*SpecificWorker* x*/)
        {
          //  this->sp = x;
            qDebug() << "Constructor StandToEat";
        }
        Status update() override 
        {
            qDebug() << "StandToEat";
          /*  QPointF t;
            t = sp->getFoodDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform("base", QVec::vec3(t.x(),0,t.y()), "world");
            angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
            qDebug() << "Angulo = " << angle;
            if( fabs(angle) < 0.001)
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
            */
           return Node::Status::Success;
        }
    private:
     //   SpecificWorker* sp;
};

class ActionGoToEat : public BrainTree::Node 
{
    public:
        ActionGoToEat(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor GoToEat";
          //  this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "goToEat";
         /*   float coordX;
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
		        sp->differentialrobot_proxy -> setSpeedBase(500,0);
                qDebug() << "Go to eat --------> RUNNING";
                return Node::Status::Running;
	        }	
            */
           return Node::Status::Success;
        }
    private:
      //  SpecificWorker* sp;    
};
class ActionInitEat : public BrainTree::Node
{
    public:
        ActionInitEat(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor initEat";
        //    this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "initEat";
           // sp->timeAction.restart();
            return Node::Status::Success;
        }
    private:
     //   SpecificWorker* sp;
};

class ActionEat : public BrainTree::Node 
{
    public:
        ActionEat(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor Eat";
          //  this->sp = x;
        }
        Status update() override 
        {
         /*   qDebug() << "Eat";
            int waitingTime = 5000; // 5 seconds eating
            if(sp->timeAction.elapsed() > waitingTime){
                int seg = sp->timeAction.elapsed() / 1000;
                qDebug() << "He estado comiendo durante " << seg << " segundos.";
                return Node::Status::Success;
            }
            else
            {
                return Node::Status::Running;
            }*/
            qDebug() << "ActionEat";
            return Node::Status::Success;
            
        }
    private:
        SpecificWorker* sp;
};

class ActionStandToDrink : public BrainTree::Node 
{
    public:
        ActionStandToDrink(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor StandToDrink";
           // this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "StandToDrink";
            /*QPointF t;
            t = sp->getWaterDispenser();
            float angle = 0;
            //Paso el punto, de coord del mundo al robot
            QVec p = sp->innerModel->transform("base", QVec::vec3(t.x(),0,t.y()), "world");
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
            */
           return Node::Status::Success;
        }
    private:
     //   SpecificWorker* sp;
};

class ActionGoToDrink : public BrainTree::Node 
{
    public:
        ActionGoToDrink(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor GoToDrink";
            //this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "GoToDrink";
         /*   float coordX;
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
	        {
		        sp->differentialrobot_proxy -> setSpeedBase(500,0);
                qDebug() << "Go to drink --------> RUNNING";
                return Node::Status::Running;
	        }	
            */
           return Node::Status::Success;
        }
    private:
       // SpecificWorker* sp;
};

class ActionInitDrink : public BrainTree::Node
{
    public:
        ActionInitDrink(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor initDrink";
        //    this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "initDrink";
           // sp->timeAction.restart();
            return Node::Status::Success;
        }
    private:
        //SpecificWorker* sp;
};

class ActionDrink : public BrainTree::Node 
{ 
    public:
        ActionDrink(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor Drink";
          //  this->sp = x;
        }
        Status update() override 
        {
           /* qDebug() << "Drink";
            int waitingTime = 4000; // 4 seconds drinking
            if(sp->timeAction.elapsed() > waitingTime){
                int seg = sp->timeAction.elapsed() / 1000;
                qDebug() << "He estado bebiendo durante " << seg << " segundos.";
                return Node::Status::Success;
            }
            else
            {
                return Node::Status::Running;
            }*/
            qDebug() << "Bebiendo";
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
};
class ActionInitWalk : public BrainTree::Node
{
    public:
        ActionInitWalk(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor initWalk";
           // this->sp = x;
        }
        Status update() override 
        {
            qDebug() << "initWalk";
          //  sp->timeAction.restart();
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
};
class ActionWalk : public BrainTree::Node 
{
    public:
        ActionWalk(/*SpecificWorker* x*/)
        {
            qDebug() << "Constructor Walk";
          //  this->sp = x;
        }
        Status update() override 
        {
          /*  qDebug() << "Walk";
            int waitingTime = 15000; // 15 seconds sleeping
            if(sp->timeAction.elapsed() > waitingTime){
                
                return Node::Status::Success;
            }
            else
            {
                RoboCompLaser::TLaserData ldata = sp->laser_proxy->getLaserData();
                //sort laser data from small to large distances using a lambda function.
                std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
    
                if( ldata.front().dist < 300)
	            {
 		            sp->differentialrobot_proxy->setSpeedBase(5, 0.6);
		            usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	            }
                else
                {
                    sp->differentialrobot_proxy->setSpeedBase(700, 0);
                }
                return Node::Status::Running;
            }*/
            qDebug() << "Andando";
            return Node::Status::Success;
        }       
    private:
       // SpecificWorker* sp;
        
};

#endif
