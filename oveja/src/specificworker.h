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

    void walk();
    void compute();
    
    void andar();
    void standTo(int x);
    void readRobotState();
    void loadPoints();
    void goTo(int x);
    void waitTime(int x);
    void createTreeBuilders();
    void createTreeManually();
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
private:
    
    


    
};

#endif
