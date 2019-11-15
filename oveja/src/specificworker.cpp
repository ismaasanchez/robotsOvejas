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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared< InnerModel > (innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{   
        readRobotState();
        loadPoints();

    try
    {
        switch(state){
		    case State::IDLE:
		    {
                waitTime();
                qDebug() << "Proceso terminado";
                exit(-1);
                break;
		    }
		    case State::Andar:
		    {
                sd1.stateInUse == State::Andar;
		    	walk();
		    	break;	
	    	}
	    	case State::Comer:
		    {
                sd1.stateInUse == State::Comer;
		    	eat();
		    	break;
		    }
            case State::Beber:
            {
                sd1.stateInUse == State::Beber;
                drink();
                break;
            }
            case State::Dormir:
            {
                sd1.stateInUse == State::Dormir;
                sleep();
                break;
            }
            case State::Colocarse:
            {

                break;
            }
            case State::IrHaciaTarget:
            {
                goTo();
                break;
            }

	}
        

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::walk()
{
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    //sort laser data from small to large distances using a lambda function.
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    if( ldata.front().dist < 200)
	{
		std::cout << ldata.front().dist << std::endl;
 		differentialrobot_proxy->setSpeedBase(5, 0.6);
		usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	}
    else
    {
        differentialrobot_proxy->setSpeedBase(200, 0);
    }
}

void SpecificWorker::eat()
{
    standTo();
}

void SpecificWorker::drink()
{
    standTo();
}

void SpecificWorker::sleep()
{
   
}

void SpecificWorker::standTo(){
    
    QPointF t;
    if(sd1.stateInUse == State::Comer)
    {
        t = foodDispenser;
    }else
    {
        t = waterDispenser;
    }

    float angle = 90;
   
    //Paso el punto, de coord del mundo al robot
    QVec p = innerModel->transform("base", QVec::vec3(t.x(),0,t.y()), "world");
    angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
    if( fabs(angle) < 0.01)
    {
        differentialrobot_proxy -> setSpeedBase(0,0);
        state = State::IrHaciaTarget;
    }
        differentialrobot_proxy -> setSpeedBase(0,angle);    
}

void SpecificWorker::readRobotState()
{
    try
    {
        differentialrobot_proxy->getBaseState(bState);  
        innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
    }      
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Laser" << e << std::endl;
    }
} 

void SpecificWorker::loadPoints(){
    foodDispenser.setX(2068.15);
    foodDispenser.setY(1973.39);
    waterDispenser.setX(-2078.81);
    waterDispenser.setY(2105.77);
}

void SpecificWorker::goTo(){

    float coordX;
    float coordY;
    if(sd1.stateInUse == State::Comer){
        coordX = foodDispenser.x();
        coordY = foodDispenser.y();
    }else{
        coordX = waterDispenser.x();
        coordX = waterDispenser.y();
    }
    if(coordX - bState.x < 3 && coordY - bState.z < 3)
	{
		differentialrobot_proxy -> setSpeedBase(0,0);
		qDebug() << "He llegado";
		state = State::IDLE;
	}
	else
	{
		differentialrobot_proxy -> setSpeedBase(500,0);
	}	
}

void SpecificWorker::chooseAction(){
    int num = rand() % 11;
    if(num < 5){
        state = State::Comer;
    }else
    {
        state = State::Beber;
    }
}

void SpecificWorker::showAction(){
    if(sd1.stateInUse == State::Comer)
    {
        qDebug() << "Comiendo ...";	#include <QTime>
    else if (sd1.stateInUse == State::Beber)
    {
        qDebug() << "Bebiendo ...";
    }
    else if(sd1.stateInUse == State::Andar)
    {;
        qDebug() << "Andando ...";
    }
    else if(sd1.stateInUse == State::Dormir)
    {
        qDebug() << "Durmiendo ...";
    }
    else
    {
        qDebug() << "Error en variable StateInUse";
    }
}

void SpecificWorker::waitTime(){
    Qtime start = currentTime();
    qDebug() << start;
}