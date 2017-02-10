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
//random comment to test branch pushing
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

	frc::Talon shooter{5};
	frc::Talon intake{6};
	frc::Talon climber{7};
	frc::DoubleSolenoid piston{0, 1};

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
<<<<<<< HEAD
		c->SetClosedLoopControl(true);

=======
>>>>>>> 7c21205... figuring out the vision tracking
	}

	void TeleopPeriodic() override {
		// Drive with arcade style (use right stick)
		myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
		omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);

		if (controller.GetRawButton(1)){
			piston.Set(DoubleSolenoid::Value::kForward);
		}
		else if (controller.GetRawButton(2)){
			piston.Set(DoubleSolenoid::Value::kReverse);
		}

		//Basic Shooter
		if(controller.GetRawButton(5)){
			shooter.Set(1.0);
		}
		else if(controller.GetRawButton(4)){
			shooter.Set(0.0);
		}

		//intake
		if(controller.GetRawButton(2)){
			intake.Set(1.0);
		}
		else if (controller.GetRawButton(3)){
			intake.Set(0.0);
		}

		/* Climber */
		if(controller.GetRawButton(0)){
			climber.Set(1.0);
		} else if(controller.GetRawButton(1)){
			climber.Set(0.0);
		}

		//Piston (not sticky)
		if (controller.GetRawButton(1)){
			piston.Set(DoubleSolenoid::Value::kForward);
		}
		else if (controller.GetRawButton(2)){
			piston.Set(DoubleSolenoid::Value::kReverse);
		}
	}

	//Chickendances
	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
