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
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);


	return true;
}

void SpecificWorker::compute()
{
    const float threshold = 200; // millimeters
   // float rot = 0.6;  // rads per second
  
        foodDispenser.setX(2068.15);
        foodDispenser.setY(1973.39);
        waterDispenser.setX(-2078.81);
        waterDispenser.setY(2105.77);
        
        differentialrobot_proxy->getBaseState(bState);  
	    innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        // read laser data
	    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        //sort laser data from small to large distances using a lambda function.
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    try
    {
        differentialrobot_proxy->getBaseState(bState);
        int x = 1;
        switch (x){
            case 0: // andar
                if( ldata.front().dist < threshold)
	            {
		             std::cout << ldata.front().dist << std::endl;
 		             differentialrobot_proxy->setSpeedBase(5, 0.6);
		             usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
	             }
                else
                {
        	    	 differentialrobot_proxy->setSpeedBase(200, 0);
                }
                walk();
                break;
            case 1: // comer
                eat();
                break;
            case 2: // beber
                drink();
                break;
            case 3: // dormir
                sleep();
                break;
        }

    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::walk()
{
    
}

void SpecificWorker::eat()
{
    goToXY(foodDispenser);
}

void SpecificWorker::drink()
{

}

void SpecificWorker::sleep()
{

}

void SpecificWorker::goToXY(QPointF t){
    float SpeedRotation = 0.6; //rads per second
    float angle = 0;
    QVec p = innerModel->transform("base", QVec::vec3(t.x(),0,t.y()), "world");
    angle = qAtan2(p.z(),p.x());

    float gradeToRadian = (angle * M_PI)/180;
    gradeToRadian = gradeToRadian * 1000000;
    float time = gradeToRadian / SpeedRotation;
    if(angle > 0)
        differentialrobot_proxy -> setSpeedBase(0,SpeedRotation);
    else
        differentialrobot_proxy -> setSpeedBase(0,-SpeedRotation);
    usleep(time); 
}




