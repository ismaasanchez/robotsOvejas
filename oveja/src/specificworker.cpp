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
    srand(time(0));

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
                walk();
		    	break;	
	    	}
	    	case State::Comer:
		    {
		    	eat();
		    	break;
		    }
            case State::Beber:
            {
                drink();
                break;
            }
            case State::Dormir:
            {
                sleep();
                break;
            }
            case State::Colocarse:
            {
                standTo();
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
    QTime sTime;
    if(stateInUse != State::Andar){
        sTime.start();
    }
    if(sTime.elapsed() < 15000)
    {
        andar();    
    }else
    {
        state = State::IDLE;
    }    
}

void SpecificWorker::andar(){
    stateInUse = State::Andar;
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
        differentialrobot_proxy->setSpeedBase(700, 0);
    }
}

void SpecificWorker::eat()
{
    stateInUse = State::Comer;
    state = State::Colocarse;
}

void SpecificWorker::drink()
{
    stateInUse = State::Beber;
    state = State::Colocarse;
}

void SpecificWorker::sleep()
{ 
   stateInUse = State::Dormir; 
   state = State::RealizarAccion;
}

void SpecificWorker::standTo(){
    QPointF t;
    if(stateInUse == State::Comer)
    {
        t = foodDispenser;
    }else if(stateInUse == State::Beber){
        t = waterDispenser;
    }else
    {
        qDebug() << "Error al cargar la vble stateInUse en el metodo -> standTo";
    }
    float angle = 0;
    //Paso el punto, de coord del mundo al robot
    QVec p = innerModel->transform("base", QVec::vec3(t.x(),0,t.y()), "world");
    angle = qAtan2(p.x(),p.z()); // calculo angulo en rads
    if( fabs(angle) < 0.001)
    {
        differentialrobot_proxy -> setSpeedBase(0,0);
        state = State::IrHaciaTarget;
    }else
    {
        differentialrobot_proxy -> setSpeedBase(0,angle);  
    }
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
    waterDispenser.setX(-2068.15);
    waterDispenser.setY(1973.39);
}

void SpecificWorker::goTo(){
    float coordX;
    float coordY;
    if(stateInUse == State::Comer){
        coordX = foodDispenser.x();
        coordY = foodDispenser.y();
    }else if(stateInUse == State::Beber){
        coordX = waterDispenser.x();
        coordY = waterDispenser.y();
    }else{
        qDebug() << "Error al cargar stateInUse en -> goTo";
    }
    if((((coordX - bState.x) < 20) && (coordX - bState.x) > -20) && (((coordY - bState.z) < 20) && (coordY - bState.z) > -20))
	{
		differentialrobot_proxy -> setSpeedBase(0,0);
		state = State::RealizarAccion;
	}
	else
	{
		differentialrobot_proxy -> setSpeedBase(500,0);
	}	
}

void SpecificWorker::chooseAction(){
    int num = rand() % 10;
    lastStateUsed = stateInUse;
   
    if(num < 3)
    {
        state = State::Dormir;
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
        state = State::Dormir;
    }
    else
    {
        qDebug() << "Error al elegir accion a relizar en el metodo -> chooseAction";
    }
    while (lastStateUsed == state)
    {
        chooseAction();
    }

}

void SpecificWorker::waitTime(){
    QTime tStart;
    tStart.start();
    int waitingTime = 0;
    QString msg;

    switch(stateInUse){
        case State::Comer:
        {
            waitingTime = 7000; // 7 secs
            msg = "Comiendo ...(7 segundos)";
            break;
        }
        case State::Beber:
        {
            waitingTime = 4000; // 4 secs
            msg = "Bebiendo ...(4 segundos)";
            break;
        }
        case State::Dormir:
        {
            msg = "Durmiendo ...(20 segundos)";
            waitingTime = 20000; // 20 secs
            break;
        }
        default:
        {
            qDebug() << "Error al cargar vble stateInUse en metodo -> waitTime";
        }
    }
    qDebug() << msg;
    while(tStart.elapsed() < waitingTime){
      
    }
    int seg = tStart.elapsed() / 1000;
    qDebug() << "He estado " << msg << " durante " << seg << " segundos.";

    state = State::IDLE;

}