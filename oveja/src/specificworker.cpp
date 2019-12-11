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
		innerModel = std::make_shared<InnerModel> (innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

    srand(time(0));
    createTreeManually(btree);
    loadPoints();
    timeAction.start();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{   
    readRobotState();
    btree.update();
}

void SpecificWorker::createTreeManually(BrainTree::BehaviorTree btree)
{
    auto mainSequence = std::make_shared<BrainTree::Sequence>();
    auto sleepSequence = std::make_shared<BrainTree::Sequence>();
    auto eatSequence = std::make_shared<BrainTree::Sequence>();
    auto drinkSequence = std::make_shared<BrainTree::Sequence>();
    auto walkSequence = std::make_shared<BrainTree::Sequence>();

    auto eatAction = std::make_shared<BrainTree::Sequence>();
    auto drinkAction = std::make_shared<BrainTree::Sequence>();
    
    auto initSleep = std::make_shared<ActionInitSleep>(this);
    auto initEat = std::make_shared<ActionInitEat>(this);
    auto initDrink = std::make_shared<ActionInitDrink>(this);
    auto initWalk = std::make_shared<ActionInitWalk>(this);

    auto realizarAccionDormir = std::make_shared<ActionSleep>(this); 
    auto realizarAccionBeber = std::make_shared<ActionDrink>(this); 
    auto realizarAccionComer = std::make_shared<ActionEat>(this); 

    auto colocarseComer = std::make_shared<ActionStandToEat>(this); 
    auto colocarseBeber = std::make_shared<ActionStandToDrink>(this); 

    auto irComer = std::make_shared<ActionGoToEat>(this); 
    auto irBeber = std::make_shared<ActionGoToDrink>(this); 

    auto andar = std::make_shared<ActionWalk>(this); 

    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(drinkSequence);
    mainSequence->addChild(walkSequence);

    sleepSequence->addChild(initSleep);
    sleepSequence->addChild(realizarAccionDormir);

    eatSequence->addChild(initEat);
    eatSequence->addChild(colocarseComer);
    eatSequence->addChild(eatAction);
        eatAction->addChild(irComer);
        eatAction->addChild(realizarAccionComer);

    drinkSequence->addChild(initDrink);
    drinkSequence->addChild(colocarseBeber);
    drinkSequence->addChild(drinkAction);
        rinkAction->addChild(irBeber);
        drinkAction->addChild(realizarAccionBeber);
    
    walkSequence->addChild(initWalk);
    walkSequence->addChild(andar);

    btree.setRoot(mainSequence);
    btree.update();
    
}
void SpecificWorker::walk()
{
    QTime sTime;
    sTime.start();
    if(sTime.elapsed() < 15000)
    {
        andar();    
    }else
    {
  //      return node::Status::Success;
    }    
}

void SpecificWorker::andar(){
    
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

void SpecificWorker::standTo(int x){
    QPointF t;
    if(x == 0)
    {
        t = foodDispenser;
    }else if(x == 1){
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
    //    return node::Status::Success;
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
        //innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
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

void SpecificWorker::goTo(int x){
    float coordX;
    float coordY;
    if(x == 0){
        coordX = foodDispenser.x();
        coordY = foodDispenser.y();
    }else if(x == 1){
        coordX = waterDispenser.x();
        coordY = waterDispenser.y();
    }else{
        qDebug() << "Error al cargar stateInUse en -> goTo";
   //     return node::Status::Failure;
    }
    if((((coordX - bState.x) < 20) && (coordX - bState.x) > -20) && (((coordY - bState.z) < 20) && (coordY - bState.z) > -20))
	{
		differentialrobot_proxy -> setSpeedBase(0,0);
	//	return node::Status::Success;
	}
	else
	{
		differentialrobot_proxy -> setSpeedBase(500,0);
	}	
}

void SpecificWorker::waitTime(int x){
    QTime tStart;
    tStart.start();
    int waitingTime = 0;
    QString msg;

    switch(x){
        case 0:
        {
            waitingTime = 3000; // 7 secs
            msg = "Comiendo ...(7 segundos)";
            break;
        }
        case 1:
        {
            waitingTime = 3000; // 4 secs
            msg = "Bebiendo ...(4 segundos)";
            break;
        }
        case 2:
        {
            msg = "Durmiendo ...(20 segundos)";
            waitingTime = 3000; // 20 secs
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

    //return node::Status::Success;
}

int SpecificWorker::getCoordXFood()
{
    return foodDispenser.x();
}

int SpecificWorker::getCoordYFood()
{
    return foodDispenser.y();
}

int SpecificWorker::getCoordXWater()
{
    return waterDispenser.x();
}

int SpecificWorker::getCoordYWater()
{
    return waterDispenser.y();
}

QPointF SpecificWorker::getFoodDispenser()
{
    return foodDispenser;
}
QPointF SpecificWorker::getWaterDispenser()
{
    return waterDispenser;
}