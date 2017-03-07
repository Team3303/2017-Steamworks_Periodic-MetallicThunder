#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>

class Robot: public frc::IterativeRobot {
public:
	Robot() {
		myRobot.SetExpiration(0.1);
		timer.Start();
	//	CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture(0);
// 		grip::GripPipeline thingy;

		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Blur", 640, 480);
//		thingy.Process(CameraServer::GetInstance()->GetVideo(0));

	}

private:
	frc::RobotDrive myRobot { 0, 1, 2, 3 };  // Robot drive system
	frc::Joystick controller { 0 }, joystick_R {1}, joystick_L { 2 };         // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Timer regulator_timer;
	frc::Talon omniwheels1{4}, omniwheels2{5};
	frc::Talon shooter{6};
	frc::Talon regulator{8};
	frc::Talon climber{7};
	frc::DoubleSolenoid piston{0, 1};
	frc::Compressor* compressor = new Compressor(0);

	bool isShooting = false;
	bool wasRbPressed = false;
	bool isRbPressed = false;
	bool isIntaking = false;
	bool wasLbPressed = false;
	bool isLbPressed = false;
	bool isClimbing = false;
	bool wasAPressed = false;
	bool isAPressed = false;
	bool isPistonOut = false;
	bool wasBPressed = false;
	bool isBPressed = false;
	bool isRegOn = false;

	bool d_pad_up() {
		if ( (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45 ) || controller.GetPOV(0) == 315 ) {
			return true;
		} else {
			return false;
		}
	}
	bool d_pad_down(){
		if ( controller.GetPOV(0) >= 135 && controller.GetPOV(0) <= 215 ) {
			return true;
		} else {
			return false;
		}
	}

	bool B(){ return controller.GetRawButton(2); }
	bool Rb(){ return controller.GetRawButton(6); }
	bool Lb(){ return controller.GetRawButton(5); }
	bool A(){ return controller.GetRawButton(1); }
	bool Y(){ return controller.GetRawButton(4); }

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
		timer.Stop();
		timer.Reset();
		regulator_timer.Stop();
		regulator_timer.Reset();
		compressor->SetClosedLoopControl(true);
	}

	void TeleopPeriodic() override {
		// Omnidrive
		if(joystick_R.GetRawButton(2)){
			myRobot.TankDrive(joystick_R.GetY(),joystick_L.GetY());
			omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
			omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		} else {
			myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
			omniwheels1.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
			omniwheels2.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
		}
		double regSpeed = SmartDashboard::GetNumber("DB/Slider 0", 0.5);
		double regTime = SmartDashboard::GetNumber("DB/Slider 1", 0.5);
		SmartDashboard::PutBoolean("DB/LED 0", isShooting);
		SmartDashboard::PutBoolean("DB/LED 1", isClimbing);
		// Shooter with Regulator controller
		wasRbPressed = isRbPressed;
		isRbPressed = Rb();
		if(!wasRbPressed && isRbPressed) {
			if (!isShooting) {
				shooter.Set (1.0);
				isShooting = true;
				std::cout << "[SHOOTER] On\n";
				regulator_timer.Start();
			}
			else {
				shooter.Set(0.0);
				isShooting = false;
				std::cout << "[SHOOTER] Stopped.\n";
				regulator_timer.Reset();
			}
		}
		if(isShooting) {
			if(regulator_timer.HasPeriodPassed(regTime)){
				isRegOn = !isRegOn;
				regulator.Set(isRegOn*regSpeed);
			}
		} else {
			regulator.Set(0);
			isRegOn = false;
		}

		// Climber Controls
		wasAPressed = isAPressed;
		isAPressed = A();
		if(!wasAPressed && isAPressed){
			if (!isClimbing) {
				climber.Set (1.0);
				isClimbing = true;
			}
			else {
				climber.Set(0.0);
				isClimbing = false;
			}
		}
		// Climber reverse
		if(!isClimbing) {
			if(Y()){
				climber.Set(-1.0);
			}else{
				climber.Set(0.0);
			}
		}
		// Piston Controls
		if(B()){
			piston.Set(DoubleSolenoid::Value::kReverse);
		}else{
			piston.Set(DoubleSolenoid::Value::kForward);
		}
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
