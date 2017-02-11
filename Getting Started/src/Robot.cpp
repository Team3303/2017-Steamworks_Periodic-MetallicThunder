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

	bool d_pad_up() {
		if ( (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45 )
				|| controller.GetPOV(0) == 315 ) {
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

	bool B(){
		return controller.GetRawButton(2);
	}
	bool Rb(){
		return controller.GetRawButton(6);
	}
	bool Lb(){
		return controller.GetRawButton(5);
	}
	bool A(){
		return controller.GetRawButton(1);
	}


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
		//Omnidrive
		myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
		omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
		omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);

		//Basic Shooter
		wasRbPressed = isRbPressed;
		isRbPressed = Rb();
		if(!wasRbPressed && isRbPressed){
			if (!isShooting){
				shooter.Set (1.0);
				isShooting = true;
			}
			else{
				shooter.Set(0.0);
				isShooting = false;
			}

		}
		//Intake
		wasLbPressed = isLbPressed;
		isLbPressed = Lb();
		if(!wasLbPressed && isLbPressed){
			if (!isIntaking){
				intake.Set (1.0);
				isIntaking = true;
			}
			else{
				intake.Set(0.0);
				isIntaking = false;
			}

		}
		//Climber
		wasAPressed = isAPressed;
		isAPressed = A();
		if(!wasAPressed && isAPressed){
			if (!isClimbing){
				climber.Set (1.0);
				isClimbing = true;
			}
			else{
				climber.Set(0.0);
				isClimbing = false;
			}

		}
		//Piston On/Off
		wasBPressed = isBPressed;
		isBPressed = B();
		if(!wasBPressed && isBPressed){
			if (!isPistonOut){
				piston.Set(DoubleSolenoid::Value::kForward);
				isPistonOut = true;
			}
			else{
				piston.Set(DoubleSolenoid::Value::kReverse);
				isPistonOut = false;
			}

		}
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
