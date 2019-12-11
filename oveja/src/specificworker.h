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

    void walk();
    void compute();
    
    void andar();
    void standTo(int x);
    void readRobotState();
    void loadPoints();
    void goTo(int x);
    void waitTime(int x);
    void createTreeBuilders();
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
private:    
};


class ActionSleep : public BrainTree::Node 
{
    public:
        ActionSleep(SpecificWorker* x)
        {
            this->sp = x;
             qDebug() << "hola cos";
        }
        Status update() override 
        {
           /* QTime tStart;
            tStart.start();
            int waitingTime = 0;
            QString msg;

            qDebug() << msg;
            if(tStart.elapsed() < waitingTime){
                int seg = tStart.elapsed() / 1000;
                qDebug() << "He estado durmiendo durante " << seg << " segundos.";
                return Node::Status::Success;
            }
            else
            {
                return Node::Status::Running;
            }   */
            qDebug() << "hola";
            sp->waitTime(2);
            //std::cout << "Durmiendo!" << std::endl;
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
    
};

class ActionStandToEat : public BrainTree::Node 
{
    public:
        ActionStandToEat(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            std::cout << "En Stand to eat!" << std::endl;
            QPointF t;
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
        }
    private:
        SpecificWorker* sp;
};

class ActionGoToEat : public BrainTree::Node 
{
    public:
        ActionGoToEat(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            std::cout << "En go to eat!" << std::endl;
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
		        sp->differentialrobot_proxy -> setSpeedBase(500,0);
                qDebug() << "Go to eat --------> RUNNING";
                return Node::Status::Running;
	        }	
        }
    private:
        SpecificWorker* sp;    
};

class ActionEat : public BrainTree::Node 
{
    public:
        ActionEat(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            sp->waitTime(0);
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
};

class ActionStandToDrink : public BrainTree::Node 
{
    public:
        ActionStandToDrink(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            std::cout << "En Stand to drink!" << std::endl;
            QPointF t;
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
        }
    private:
        SpecificWorker* sp;
};

class ActionGoToDrink : public BrainTree::Node 
{
    public:
        ActionGoToDrink(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            std::cout << "En go to drink!" << std::endl;
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
	        {
		        sp->differentialrobot_proxy -> setSpeedBase(500,0);
                qDebug() << "Go to drink --------> RUNNING";
                return Node::Status::Running;
	        }	
        }
    private:
        SpecificWorker* sp;
};

class ActionDrink : public BrainTree::Node 
{ 
    public:
        ActionDrink(SpecificWorker* x)
        {
            this->sp = x;
        }
        Status update() override 
        {
            std::cout << "Estoy bebiendo!" << std::endl;
            sp->waitTime(1);
            return Node::Status::Success;
        }
    private:
        SpecificWorker* sp;
};

class ActionWalk : public BrainTree::Node 
{
    public:
       /* ActionWalk(SpecificWorker* x)
        {
            this->sp = x;
        }*/
        Status update() override 
        {
            std::cout << "Estoy andando!" << std::endl;
            return Node::Status::Success;
        }       
    private:
      //  SpecificWorker* sp;
        
};

#endif
