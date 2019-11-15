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
#include <math.h>
#include <QTime>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);


public slots:


private:
    RoboCompGenericBase::TBaseState bState;
	std::shared_ptr<InnerModel> innerModel;
    QPointF foodDispenser;
    QPointF waterDispenser;

    void compute();
    void walk();
    void eat();
    void drink();
    void sleep();
    void standTo();
    void readRobotState();
    void loadPoints();
    void goTo();
    void chooseAction();
    void showAction();
    void waitTime();

    enum class State{IDLE,Andar,Comer,Beber,Dormir,Colocarse,IrHaciaTarget};
	State state = State::IDLE;

    struct SharedData{
        State stateInUse;
        
    };
    struct SharedData sd1;
};

#endif
