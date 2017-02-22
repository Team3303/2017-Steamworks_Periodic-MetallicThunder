/*
 * TalonTest.cpp
 *
 *  Created on: Jan 17, 2017
 *      Author: Owner
 */
#include <Talon.h>
#include "WPILib.h"
#include <Timer.h>

class Shooter{
	private:
		frc::Talon shoot1 {100};
		frc::Timer timer;
	public:
		Shooter(){}
	void shoot(double speed, double time){
		timer.Reset();
		timer.Start();
		shoot1.Set(speed);
		if(timer.HasPeriodPassed(time)){
			shoot1.Set(0.0);
		}
		timer.Stop();
	}
};
