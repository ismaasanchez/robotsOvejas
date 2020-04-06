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
    crearArboles();
    loadPoints();
    timeAction.start();
	this->Period = period;
	timer.start(Period);
  //  std::thread t1(&SpecificWorker::initCron);
  //  t1.join();
}

void SpecificWorker::compute()
{   
        readRobotState();
        escribirCoords(robotName,bState.x,bState.z);
        if(robotName == "base"){
             btree.update();  
        }
        else if(robotName == "base1")
        {
             btree1.update();  
        }
        else if(robotName == "base2")
        {
            btree2.update();
        }
        else if(robotName == "base3")
        {
            btree3.update();
        }else if(robotName == "base4")
        {
            btree4.update();
        }
        
        //qDebug() << "Nombre del robot: " << robotName.c_str();
        
       

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

    
    
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(drinkSequence); 
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(eatSequence);
    //mainSequence->addChild(walkSequence);
    
    

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
   // btree.update();    
    
}

void SpecificWorker::createTree1(BrainTree::BehaviorTree &btree)
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

    mainSequence->addChild(drinkSequence);
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(eatSequence);  

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
  //  btree.update();    
    
}

void SpecificWorker::createTree2(BrainTree::BehaviorTree &btree)
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

    
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(drinkSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    

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
  //  btree.update();    
    
}

void SpecificWorker::createTree3(BrainTree::BehaviorTree &btree)
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

    mainSequence->addChild(walkSequence);
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(sleepSequence); 
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(drinkSequence);
    

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
  //  btree.update();    
    
}


void SpecificWorker::createTree4(BrainTree::BehaviorTree &btree)
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
    
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(drinkSequence);
    
    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(walkSequence);
    mainSequence->addChild(eatSequence);
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
  //  btree.update();    
    
}

void SpecificWorker::readRobotState()
{
    try
    {
      //  qDebug() << "EN METODO READROBOTSTATE";
      //  qDebug() << "y: " << bState.z;
        differentialrobot_proxy->getBaseState(bState);
        innerModel->updateTransformValues(robotName.c_str(), bState.x, 0, bState.z, 0, bState.alpha, 0);
       // qDebug() << "EN METODO READROBOTSTATE fin";
      //  qDebug() << "y: " << bState.z;
    }      
    catch(const Ice::Exception &e)
    {
        std::cout << "Error reading from Laser" << e << std::endl;
    }
} 

void SpecificWorker::loadPoints(){
    foodDispenser.setX(1868.15);
    foodDispenser.setY(1773.39);
    waterDispenser.setX(-1868.15);
    waterDispenser.setY(-1773.39);
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

void SpecificWorker::escribirCoords(std::string nRobot, float x, float y)
{
    ofstream file;
    int h = getIntName(nRobot);
    qDebug() << "El numero es : " << h;
    switch(h){
        case 0:
            file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot1.txt", ios::app);
            break;
        case 1:
            file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot2.txt", ios::app);
            break;
        case 2:
            file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot3.txt", ios::app);
            break;
        case 3:
            file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot4.txt", ios::app);
            break;
        case 4:
            file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot5.txt", ios::app);
            break;
    };
    //file.open("/home/ismael/robocomp/components/robotsOvejas/oveja/ficherosCoords/archivoRobot1", ios::app);

    if(file.fail())
    {
        qDebug() << "Error al abrir el archivo";
    }

    file <<  x << "#" << y << endl;

    file.close();
}


int SpecificWorker::getIntName(std::string nRobot)
{
    if(nRobot == "base") return 0;
    if(nRobot == "base1") return 1;
    if(nRobot == "base2") return 2;
    if(nRobot == "base3") return 3;
    if(nRobot == "base4") return 4;
}

void SpecificWorker::crearArboles()
{
    if(robotName == "base") createTreeManually(btree);;
    if(robotName == "base1") createTree1(btree1);
    if(robotName == "base2") createTree2(btree2);
    if(robotName == "base3") createTree3(btree3);
    if(robotName == "base4") createTree4(btree4);

    
   // differentialrobot_proxy -> setOdometerPose(500,0,bState.alpha);
   // createTree1(btree1);
   // createTree2(btree2);
   // createTree2(btree3);
}

void SpecificWorker::initCron()
{

    int nHilos = 12;

    Bosma::Scheduler s(nHilos);

    s.interval(std::chrono::seconds(3), []() {
      std::cout << "KJSABFHJABKJCBSDAKJBJKDASBFJKDBSAKJHBJHKLADSBCIUJHWBKJHCBASDKJBKJLHDABSFLJUHWASBVLHV" << std::endl;
    }
    );
    std::this_thread::sleep_for(std::chrono::minutes(1));
}