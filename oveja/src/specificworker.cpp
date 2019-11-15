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
                chooseAction();
                break;
		    }
		    case State::Andar:
		    {
                QTime sTime;
                if(stateInUse != State::Andar){
                    sTime.start();
                }
                if(sTime.elapsed() < 15000)
                {
                    walk();    
                }else
                {
                    state = State::IDLE;
                }
		    	break;	
	    	}
	    	case State::Comer:
		    {
		    	eat();
		    	break;
		    }
            case State::Beber:
            {
                stateInUse = State::Beber;
                drink();
                break;
            }
            case State::Dormir:
            {
                stateInUse = State::Dormir;
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
            case State::RealizarAccion:
            {
                waitTime();
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
    stateInUse = State::Andar;
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    //sort laser data from small to large distances using a lambda function.
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    qDebug() << "Estoy en walk";
    
    if( ldata.front().dist < 200)
	{
		std::cout << ldata.front().dist << std::endl;
 		differentialrobot_proxy->setSpeedBase(5, 0.6);
		usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	}
    else
    {
        differentialrobot_proxy->setSpeedBase(700, 0);
    }
}

void SpecificWorker::eat()
{
    qDebug() << "Estoy en eat";
    stateInUse = State::Comer;
    standTo();
}

void SpecificWorker::drink()
{
    qDebug() << "Estoy en drink";
    stateInUse = State::Beber;
    standTo();
}

void SpecificWorker::sleep()
{
   qDebug() << "Estoy en sleep"; 
   stateInUse = State::Dormir; 
   state = State::RealizarAccion;
}

void SpecificWorker::standTo(){
    qDebug() << "Estoy en standTo";
    QPointF t;
    if(stateInUse == State::Comer)
    {
        qDebug() << "toca comer";
        t = foodDispenser;
    }else
    {
        qDebug() << "toca beber";
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
    }else
    {
        differentialrobot_proxy -> setSpeedBase(0,angle);  
    }
    state = State::IrHaciaTarget;

}

void SpecificWorker::readRobotState()
{
    //qDebug() << "Estoy en readRobotState";
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
   // qDebug() << "Estoy en loadPoints";
    foodDispenser.setX(2068.15);
    foodDispenser.setY(1973.39);
    waterDispenser.setX(-2078.81);
    waterDispenser.setY(2105.77);
}

void SpecificWorker::goTo(){
    qDebug() << "Estoy en goTo";
    float coordX;
    float coordY;
    if(stateInUse == State::Comer){
        qDebug() << "Cargo coordenadas del comedero";
        coordX = foodDispenser.x();
        coordY = foodDispenser.y();
    }else if(stateInUse == State::Beber){
        qDebug() << "Cargo coordenadas de bebedero";
        coordX = waterDispenser.x();
        coordX = waterDispenser.y();
    }else{
        qDebug() << "Error al cargar stateInUse en -> goTo";
    }
    if(coordX - bState.x < 3 && coordY - bState.z < 3)
	{
		differentialrobot_proxy -> setSpeedBase(0,0);
		qDebug() << "He llegado";
		state = State::RealizarAccion;
	}
	else
	{
		differentialrobot_proxy -> setSpeedBase(500,0);
	}	
}

void SpecificWorker::chooseAction(){
    qDebug() << "Estoy en chooseAction";
    int num = rand() % 10;
    if(num < 3)
    {
        state = State::Andar;
    }
    else if(num > 2 && num < 5)
    {
        state = State::Comer;
    }
    else if(num > 4 && num < 7)
    {
        state = State::Beber;
    }
    else if(num > 6)
    {
        state = State::Andar;
    }
    else
    {
        qDebug() << "Error al elegir accion a relizar en el metodo -> chooseAction";
    }
}

void SpecificWorker::waitTime(){
    qDebug() << "Estoy en waitTime";
    QTime tStart;
    tStart.start();
    int waitingTime = 0;
    QString msg;

    switch(stateInUse){
        case State::Comer:
        {
            waitingTime = 7000; // 7 secs
            msg = "Comiendo ...";
            break;
        }
        case State::Beber:
        {
            waitingTime = 4000; // 4 secs
            msg = "Bebiendo ...";
            break;
        }
        case State::Dormir:
        {
            msg = "Durmiendo ...";
            waitingTime = 20000; // 20 secs
            break;
        }
        default:
        {
            qDebug() << "Error al cargar vble stateInUse en metodo -> waitTime";
        }
    }
    while(tStart.elapsed() < waitingTime){
        qDebug() << msg;
    }
    int seg = tStart.elapsed() / 1000;
    qDebug() << "He estado " << msg << " durante " << seg << " segundos.";

    state = State::IDLE;

}