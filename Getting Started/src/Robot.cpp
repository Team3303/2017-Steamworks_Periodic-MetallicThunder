#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <TalonTest.cpp>
#include <GRIP.cpp>

class Robot: public frc::IterativeRobot {
public:
	Robot() {
		myRobot.SetExpiration(0.1);
		timer.Start();
	//	CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture(0);
//		grip::GripPipeline thingy;

		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Blur", 640, 480);
//		thingy.Process(CameraServer::GetInstance()->GetVideo(0));
	}

private:
	frc::RobotDrive myRobot { 1, 2, 0, 9 };  // Robot drive system
	frc::Joystick controller { 0 }, joystick_R {1}, joystick_L { 2 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Talon omniwheels1{3}, omniwheels2{4};

	frc::Compressor *c = new Compressor(0);
	//get rid of later
	frc::DoubleSolenoid soldub {0,1};

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
		c->SetClosedLoopControl(true);

	}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)
		myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
		omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		// get rid of later
		if (controller.GetRawButton(1)){
			soldub.Set(DoubleSolenoid::Value::kForward);
		}
		else if (controller.GetRawButton(2)){
			soldub.Set(DoubleSolenoid::Value::kReverse);
		}
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
