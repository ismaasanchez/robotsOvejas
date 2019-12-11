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
    createTreeManually(this.btree);
    loadPoints();
	return true;
}

class ActionSleep : public BrainTree::Node 
{
    public:
        ActionSleep(SpecificWorker* x)
        {
            this->sp = x;
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
            std::cout << "Durmiendo!" << std::endl;
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
            qDebug() << "Coordenada x = " << t.x();
            qDebug() << "Coordenada y = " << t.y();
            t = sp->foodDispenser;
            qDebug() << "Coordenada x = " << t.x();
            qDebug() << "Coordenada y = " << t.y();
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
            qDebug() << "Coordenada x = " << coordX;
            qDebug() << "Coordenada y = " << coordY;
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
      /*  ActionStandToDrink(SpecificWorker* x)
        {
            this->sp = x;
        }*/
        Status update() override 
        {
            std::cout << "Posicionandome para ir al bebedero!" << std::endl;
            return Node::Status::Success;
        }
    private:
       // SpecificWorker* sp;
};

class ActionGoToDrink : public BrainTree::Node 
{
    public:
      /*  ActionGoToDrink(SpecificWorker* x)
        {
            this->sp = x;
        }*/
        Status update() override 
        {
            std::cout << "De camino hacia el bebedero!" << std::endl;
            return Node::Status::Success;
        }
    private:
       // SpecificWorker* sp;
};

class ActionDrink : public BrainTree::Node 
{ 
    public:
       /* ActionDrink(SpecificWorker* x)
        {
            this->sp = x;
        }*/
        Status update() override 
        {
            std::cout << "Estoy bebiendo!" << std::endl;
            return Node::Status::Success;
        }
    private:
      //  SpecificWorker* sp;
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

void SpecificWorker::createTreeBuilders()
{
   /* auto btree = BrainTree::Builder()
        .composite<BrainTree::Sequence>() // Dormir
            .leaf(waitTime(2))
        .composite<BrainTree::Sequence>() // Comer
            .leaf(standTo(0))
            .leaf(goTo(0))
            .leaf(waitTime(0))
        .composite<BrainTree::Sequence>() // Beber
            .leaf(standTo(1))
            .leaf(goTo(1))
            .leaf(waitTime(1))
        .composite<BrainTree::Sequence>() // Andar
            .leaf(walk())
        .end()
        .build(); */
}   

void SpecificWorker::createTreeManually(BrainTree::BehaviorTree btree)
{
    
    
    auto mainSequence = std::make_shared<BrainTree::Sequence>();
    auto sleepSequence = std::make_shared<BrainTree::Sequence>();
    auto eatSequence = std::make_shared<BrainTree::Sequence>();
    auto drinkSequence = std::make_shared<BrainTree::Sequence>();
    auto walkSequence = std::make_shared<BrainTree::Sequence>();

    auto realizarAccionDormir = std::make_shared<ActionSleep>(this); //waitTime(2);
    auto realizarAccionBeber = std::make_shared<ActionDrink>(); //waitTime(1);
    auto realizarAccionComer = std::make_shared<ActionEat>(this); //waitTime(0);

    auto colocarseComer = std::make_shared<ActionStandToEat>(this); //standTo(0);
    auto colocarseBeber = std::make_shared<ActionStandToDrink>(); //standTo(1);

    auto irComer = std::make_shared<ActionGoToEat>(this); //goTo(0);
    auto irBeber = std::make_shared<ActionGoToDrink>(); //goTo(1);

    auto andar = std::make_shared<ActionWalk>(); //walk();

    mainSequence->addChild(sleepSequence);
    mainSequence->addChild(eatSequence);
    mainSequence->addChild(drinkSequence);
    mainSequence->addChild(walkSequence);

    sleepSequence->addChild(realizarAccionDormir);
    eatSequence->addChild(colocarseComer);
    eatSequence->addChild(irComer);
    eatSequence->addChild(realizarAccionComer);
    drinkSequence->addChild(colocarseBeber);
    drinkSequence->addChild(irBeber);
    drinkSequence->addChild(realizarAccionBeber);
    walkSequence->addChild(andar);

    btree.setRoot(mainSequence);
    
}

void SpecificWorker::compute()
{   
 //     btree->update();
    readRobotState();
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