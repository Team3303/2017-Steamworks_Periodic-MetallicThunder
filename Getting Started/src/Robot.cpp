#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <TalonTest.cpp>

class Robot: public frc::IterativeRobot {
public:
	Robot() {
		myRobot.SetExpiration(0.1);
		timer.Start();
	}

private:
	frc::RobotDrive myRobot { 1, 2, 0, 9 };  // Robot drive system
	frc::Joystick controller { 0 }, joystick_R {1}, joystick_L { 2 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Talon omniwheels1{3}, omniwheels2{4};
	Shooter kapow;

	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
	}

	void AutonomousPeriodic() override {
		// Drive for 2 seconds
		if (timer.Get() < 2.0) {
			myRobot.Drive(-0.5, 0.0);  // Drive forwards half speed
		} else {
			myRobot.Drive(0.0, 0.0);  // Stop robot
		}
	}

	void TeleopInit() override {

	}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)
		myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
		omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
