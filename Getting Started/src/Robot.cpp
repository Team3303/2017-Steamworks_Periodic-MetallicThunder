#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <math.h>
#include "wpilib.h"
#include <sstream>

class Robot: public frc::IterativeRobot {
public:
	std::shared_ptr<NetworkTable> networkTable;

	Robot() {
		networkTable = NetworkTable::GetTable("GRIP/myContoursReport");

		myRobot.SetExpiration(0.1);
		timer.Start();

		CameraServer::GetInstance()->StartAutomaticCapture(0);
//		cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
//		cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Blur", 640, 480);

		CameraServer::GetInstance()->AddAxisCamera("10.33.3.19");
	}

private:
	frc::RobotDrive myRobot { 0, 1, 2, 3 };  // Robot drive system
	frc::Joystick controller { 0 }, joystick_R { 1 }, joystick_L { 2 };  // Only joystick
	frc::LiveWindow* lw = frc::LiveWindow::GetInstance();
	frc::Timer timer;
	frc::Timer regulatorTimer;
	frc::Talon omniwheels1{ 4 }, omniwheels2{ 5 };
	frc::Talon shooter{ 6 };
	frc::Talon regulator{ 8 };
	frc::Talon climber{ 7 };
	frc::DoubleSolenoid piston{ 0, 1 };
	frc::Compressor* compressor = new Compressor( 0 );
	frc::AnalogGyro gyro{ 1 };
	
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
	bool isXPressed = false;
	bool wasXPressed = false;
	bool isCompressing = false;

	// camera 
	double centerPixel = 400.0;  //find correct value
	double FOV = 67;  //degrees
	double focalLength = centerPixel / tan((FOV / 2.0) * (3.14159 / 180.0)); 
	
	bool d_pad_up() {
		if ( (controller.GetPOV(0) >= 0 && controller.GetPOV(0) <= 45) || controller.GetPOV(0) == 315 ) {
			return true;
		} else {
			return false;
		}
	}

	bool d_pad_down() {
		if ( controller.GetPOV(0) >= 135 && controller.GetPOV(0) <= 215 ) {
			return true;
		} else {
			return false;
		}
	}

	bool A(){ return controller.GetRawButton(1); }
	bool B(){ return controller.GetRawButton(2); }
	bool X(){ return controller.GetRawButton(3); }
	bool Y(){ return controller.GetRawButton(4); }
	bool Lb(){ return controller.GetRawButton(5); }  // current gyro loop break
	bool Rb(){ return controller.GetRawButton(6); }
	
	void TargetHook() {
		
		//calculate angle offset
		std::vector<double> centerXarr = networkTable->GetNumberArray("centerX", llvm::ArrayRef<double>());
		double centerX =(centerXarr[0] + centerXarr[1]) / 2;
		double pixelOffset = centerX - centerPixel;
		double angleOffset = atan(pixelOffset / focalLength) * (180 / 3.14159);  // degrees
		
	 /* gyro.Reset();
		while( !(gyro.GetAngle() > (angleOffset - 1) && gyro.GetAngle() < (angleOffset + 1)) && !controller.GetRawButton(button) ) 
		{
			double angleRemaining = angleOffset - gyro.GetAngle();
			double turnSpeed =  angleRemaining / (FOV / 2.0);
			myRobot.TankDrive(turnSpeed, -turnSpeed);
		}
		
		myRobot.Drive(0.0, 0.0); */
		
		Align(angleOffset, FOV / 2.0);
		
	}
	
	//rotate bot to the right by angle
	void Align(double angle, double scale){   //takes angle in degrees, and scale in max degrees
		gyro.Reset();
		while( !(gyro.GetAngle() > (angle - 1) && gyro.GetAngle() < (angle + 1)) && !Lb() ) 
		{
			double angleRemaining = angle - gyro.GetAngle();
			double turnSpeed =  angleRemaining / scale;
			myRobot.TankDrive(turnSpeed, -turnSpeed);
		}
		
		myRobot.Drive(0.0, 0.0);
	}
	
	void AutonomousInit() override {
		timer.Reset();
		timer.Start();
		gyro.InitGyro();
	}

	void AutonomousPeriodic() override {
		
		std::stringstream stream;
		std::string gyroValue;
		stream << gyro.GetAngle();
		stream >> gyroValue;
		SmartDashboard::PutString("DB/String 0", gyroValue);
		double driveTime = SmartDashboard::GetNumber("DB/Slider 2", 2.0);
		
		// Drive for driveTime seconds
		if (timer.Get() < driveTime) {
			myRobot.Drive(-0.5, 0.0);  // Drive forwards half speed
		} else if (timer.Get() < (driveTime + 0.5)){
			myRobot.Drive(0.0, 0.0);  // Stop robot
		} else if(SmartDashboard::GetBoolean("DB/Button 0", false)){
			if (timer.Get() < (driveTime + 0.5 + 0.5)){
				piston.Set(DoubleSolenoid::Value::kReverse);            //fire piston
			} else if (timer.Get() < (driveTime + 0.5 + 0.5 + 0.5)){
				piston.Set(DoubleSolenoid::Value::kReverse);            //retract piston
			}
		}else{
			myRobot.Drive(0.0, 0.0);
		}
	}

	void TeleopInit() override {
		timer.Stop();
		timer.Reset();
		regulatorTimer.Stop();
		regulatorTimer.Reset();
		gyro.InitGyro();
	}

	void TeleopPeriodic() override {
		/*
		 * FIXME: A Network of Tables
		 */

		std::vector<double> arr = networkTable->GetNumberArray("area", llvm::ArrayRef<double>());
//		for(unsigned int i = 0; i < arr.size(); i++){
//			std::cout << arr[i] << std::endl;
//		}

		//gyro to dashboard
		std::stringstream stream;
		std::string gyroValue;
		stream << gyro.GetAngle();
		stream >> gyroValue;
		SmartDashboard::PutString("DB/String 0", gyroValue);

		// TODO: Omnidrive hdrive
		if(joystick_R.GetRawButton(2)){
			myRobot.TankDrive(joystick_R.GetY(),joystick_L.GetY());
			omniwheels1.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
			omniwheels2.Set(-(joystick_R.GetX()+joystick_L.GetX())/2);
		} else {
			myRobot.TankDrive(-joystick_L.GetY(),-joystick_R.GetY());
			omniwheels1.Set((joystick_R.GetX()+joystick_L.GetX())/2);
			omniwheels2.Set((joystick_R.GetX()+joystick_L.GetX())/2);
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
				regulatorTimer.Start();
			}
			else {
				shooter.Set(0.0);
				isShooting = false;
				std::cout << "[SHOOTER] Stopped.\n";
				regulatorTimer.Reset();
			}
		}
		if(isShooting) {
			if(regulatorTimer.HasPeriodPassed(regTime)){
				isRegOn = !isRegOn;
				regulator.Set(isRegOn*regSpeed);
			}
		} else {
			regulator.Set(0);
			isRegOn = false;
		}

		// Climber Controls
		//double hangSpeed = SmartDashboard::GetNumber( "DB/Slider 2", 0 );
		// Climber reverse and Hold
		if(Y()) {
			climber.Set( -1.0 );
		} else if (d_pad_up()) {
			climber.Set( 0.2 );
		} else {
			climber.Set( 0.0 );
		}

		// Piston Controls
		if(B()){
			piston.Set(DoubleSolenoid::Value::kReverse);
		}else{
			piston.Set(DoubleSolenoid::Value::kForward);
		}

		// Compressor Controls
		wasXPressed = isXPressed;
		isXPressed = X();
		if(!wasXPressed && isXPressed){
			if (!isCompressing) {
				compressor->SetClosedLoopControl(true);
				isCompressing = true;
			}
			else {
				compressor->SetClosedLoopControl(false);
				isCompressing = false;
			}
		}
		
		// gyro and vision testing
		if(A()){
			Align(45.0, 45.0);
			//TargetHook();
		}
	}

	void TestPeriodic() override {
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
