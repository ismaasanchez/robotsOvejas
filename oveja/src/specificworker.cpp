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

        robotName = params.at("RobotName").value;
	}
	catch(std::exception e) { qFatal("Error reading config params"); }
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
    createTreeManually(btree);
    loadPoints();
    timeAction.start();
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{   
   // if(robotName == "fbase")
  //  {
        readRobotState();
        btree.update();
  //  }
}

void SpecificWorker::createTreeManually(BrainTree::BehaviorTree &btree)
{
    qDebug() << "Creando arbol";
    auto mainSequence = std::make_shared<BrainTree::Sequence>();
    
    //Dormir
    auto sleepSequence = std::make_shared<BrainTree::Sequence>();
    auto initSleep = std::make_shared<ActionInitSleep>(this);
 
    //Comer
    auto eatSequence = std::make_shared<BrainTree::Sequence>();
    auto colocarseComer = std::make_shared<ActionStandToEat>(this);
    auto irComer = std::make_shared<ActionGoToEat>(this); 
    auto initEat = std::make_shared<ActionInitEat>();

    //Beber
    auto drinkSequence = std::make_shared<BrainTree::Sequence>();
    auto colocarseBeber = std::make_shared<ActionStandToDrink>(this); 
    auto irBeber = std::make_shared<ActionGoToDrink>(this); 
    auto initDrink = std::make_shared<ActionInitDrink>(); 

    //Andar
    auto walkSequence = std::make_shared<BrainTree::Sequence>();
    auto initWalk = std::make_shared<ActionInitWalk>(this);
    auto andar = std::make_shared<ActionWalk>(this); 


    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(drinkSequence);
    mainSequence->addChild(walkSequence);

    sleepSequence->addChild(initSleep);
   

    
    eatSequence->addChild(colocarseComer);
    eatSequence->addChild(irComer);
    eatSequence->addChild(initEat);
      
    drinkSequence->addChild(colocarseBeber);
    drinkSequence->addChild(irBeber);
    drinkSequence->addChild(initDrink);  
    walkSequence->addChild(initWalk);
    walkSequence->addChild(andar);
    
    btree.setRoot(mainSequence);
    btree.update();    
    
}


void SpecificWorker::readRobotState()
{
    try
    {
        qDebug() << "EN METODO READROBOTSTATE";
        qDebug() << "y: " << bState.z;
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues(robotName.c_str(), bState.x, 0, bState.z, 0, bState.alpha, 0);
        qDebug() << "EN METODO READROBOTSTATE fin";
        qDebug() << "y: " << bState.z;
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