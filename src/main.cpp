#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-3, -5, -16},pros::MotorGearset::blue); // left motor group - ports 4 (reversed), 1 (reversed), 14 (reversed)
pros::MotorGroup rightMotors({17, 10, 14}, pros::MotorGearset::blue); // right motor group - ports 6, 3, 12 
pros::Motor firststage(-6); 
pros::Motor secondstage(-18);
pros::Motor roller(-12);
pros::Distance left(8);
pros::Distance right(9);
pros::Distance front(7);
pros::Imu imu(1); 

// pneumatics
static pros::adi::DigitalOut matchloader('D');
static pros::adi::DigitalOut ramp('C');


//variables
int intakeTime = 0;
bool antiJam = false;
int outake = 0;
int top = 0;

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.625, // 11.625 inch track width
                              3.3062522077, // using new 3.25" omnis + offset
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(11.25, // proportional gain (kP)
                                            0.01, // integral gain (kI)
                                            23.5, // derivative gain (kD)
                                            3, // anti windup
                                            0.5, // small error range, in inches
                                            50, // small error range timeout, in milliseconds
                                            1.5, // large error range, in inches
                                            250, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             26, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             250, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);



// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
//imu scale
class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    virtual double get_rotation() const {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

CustomIMU my_imu(1, 1.00285535204);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &my_imu);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
//helper variable for intake
int ta = 0;
int asx = 127;
bool skb = false;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            printf("(&%f,%f),\n",chassis.getPose().x, chassis.getPose().y);

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    pros::Task anti_jam([&](){
        int stuckStart = -1;   // time when stall was first detected

        while (true) {
            if (intakeTime > 0) {
                double vel = fabs(firststage.get_actual_velocity());

                if (vel < 50) {  // possible jam
                    if (stuckStart == -1) stuckStart = pros::millis();

                    if (pros::millis() - stuckStart >= 50) {
         
                        antiJam = true;
                        firststage.move(-127);   // reverse to clear jam
                        pros::delay(100);
                        firststage.move(127);    // resume forward
                        pros::delay(100);
                        antiJam = false;
                        stuckStart = -1;           // reset timer
                    }
                } 
                else {
                    stuckStart = -1;
                }
            } 
            else {
                stuckStart = -1; // intake off, reset timer
            }

            pros::delay(10);
        }
    });   
    pros::Task intake1([&]() {
        while (true){
            if (!antiJam){
                if (intakeTime > 0 && outake == 1){
                    firststage.move(-127);
                    intakeTime -= 10;
                    secondstage.move(127);
                }
                else if (intakeTime > 0 && top == 1){
                    firststage.move(127);
                    secondstage.move(-127);
                    roller.move(-127);
                    intakeTime -= 10;
                }
                else if (intakeTime > 0 && top == 0){
                    firststage.move(127);
                    secondstage.move(0);
                    roller.move(0);
                    intakeTime-=10;
                } 
                else if (intakeTime > 0 && top == 4){
                    if(skb == false){
                        firststage.move(-127);
                        secondstage.move(127);
                        pros::delay(100);
                        firststage.move(127);
                        secondstage.move(-10);
                        skb = true;
                    }
                    
                    //int ta = 0;
                    roller.move(asx);
                    //secondstage.move(asx);
                    intakeTime-=10;
                    ta+=10;
                    if (ta == 1000 ){
                        asx = 60;
                    }
                }
                else if (intakeTime > 0 && top == 5){
                    firststage.move(127);
                    secondstage.move(-10);
                    roller.move(100);
                    intakeTime-=10;
                }           
                else{
                    firststage.move(0);
                    secondstage.move(0);
                }

            }               
            pros::delay(10);
        } 
    });
    
}

// Structure to organize sensor physical data
struct SensorConfig {
    float x_off;      // Forward/Backward from center (inches)
    float y_off;      // Left/Right from center line (inches, + is Right)
    float mount_side; // Relative to robot front: Front=0, Right=90, Back=180, Left=270
};
SensorConfig leftSensorConfig = {5, 4.5, 270}; // Example config for left sensor

SensorConfig rightSensorConfig = {5, -4.5, 90}; // Example config for right sensor

SensorConfig frontSensorConfig = {7, -5, 0}; // Example config for front sensor
// 1. Automatically finds the closest wall heading (0, 90, 180, or 270)
float get_wall_angle(float robot_heading, float mount_side) {
    float global_sensor_direction = robot_heading + mount_side;
    // Round to nearest 90   degrees
    float wall = std::round(global_sensor_direction / 90.0) * 90.0;
    return wall;
}

// 2. The upgraded distance formula
float get_dist_to_wall(float raw_mm, SensorConfig config, float current_theta) {
    float d_sensor = raw_mm / 25.4; // Convert to inches
    
    // Calculate how tilted the sensor is relative to the wall it's facing
    float wall_angle = get_wall_angle(current_theta, config.mount_side);
    float theta_rel = lemlib::degToRad(current_theta + config.mount_side - wall_angle);

    // X-off and Y-off projection
    // This accounts for the sensor "swinging" as the robot rotates
    float perp_dist = (d_sensor + config.x_off) * std::cos(theta_rel) - (config.y_off) * std::sin(theta_rel);

    return std::abs(perp_dist);   
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

//movestraight function
void moveStraight(float length, int timeout, lemlib::MoveToPointParams params) {
    if (chassis.isInMotion()) chassis.waitUntilDone();
    params.forwards = length > 0;
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPoint(pose.x + length * sin(lemlib::degToRad(pose.theta)),
                           pose.y + length * cos(lemlib::degToRad(pose.theta)), timeout, params);
}


void pidtest(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    //pros::delay(10000000);
    intakeTime = 3000000;
    chassis.setPose(0,0,-90);
    top = 0;
    matchloader.set_value(false);              
    antiJam = false;

    ramp.set_value(false);
    
    //moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
    chassis.moveToPoint(-33, 25, 2500, {.maxSpeed = 80, .minSpeed = 1});
    chassis.swingToHeading(0 , lemlib::DriveSide::RIGHT, 300);
    //chassis.turnToHeading(-20, 100)   
    moveStraight(200, 900, {.maxSpeed = 127});
    pros::delay(2300);
    

    //moveStraight(-100, 200, {.maxSpeed = 127});

   // 1.22, 25.65
   //pros::delay(10000000000);
}
 


void fastskills(){

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        ramp.set_value(false);

    intakeTime = 1500000;
        top = 0;
    top = 0;
    matchloader.set_value(false);              
    antiJam = false;
    chassis.setPose(0,0, 90);
    chassis.moveToPoint(29, 2, 3000, {.maxSpeed = 80});
    chassis.turnToHeading(-42, 600);
    //pros::delay(1000000);
    chassis.moveToPoint(36, -7.0, 1000, {.forwards = false});
    moveStraight(-10, 200, {.maxSpeed = 40});
    chassis.turnToHeading(-45, 400);

    ramp.set_value(true);
    //idk.set_value(false);
    chassis.waitUntilDone();
   // top = 5;
    // pros::delay(10000000000);

    top = 5;
    pros::delay(1000);
        matchloader.set_value(true);

    chassis.moveToPoint(-1.6, 31.34, 1500);
    top = 0;
    chassis.turnToHeading(-90, 600);
    chassis.waitUntilDone();
    //pros::delay(50);
    chassis.setPose((72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);
    //chassis.setPose(72-(left.get()/25.4), 72-(front.get()/25.4), chassis.getPose().theta);
   
    top = 0;
    ramp.set_value(false);
    chassis.moveToPoint(25, 48.5, 800, {.maxSpeed = 60});

    //pros::delay(10000000000);
    
   

       // chassis.waitUntilDone();
    //pros::delay(150);
    chassis.turnToHeading(-90, 200);
    moveStraight(2, 1100, {.maxSpeed = 40});
    //
    pros::delay(1300);
    chassis.moveToPoint(48, 66, 500, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
    //pros::delay(50);
    chassis.setPose(-(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);
    matchloader.set_value(false);
    chassis.moveToPoint(30, 68, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(180, 800, {.minSpeed = 1, .earlyExitRange = 3});
    moveStraight(9, 1000,{.maxSpeed = 40});  
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    //pros::delay(50);
    chassis.setPose(chassis.getPose().x, 72-get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta), chassis.getPose().theta);
    chassis.moveToPoint(20, 49.25, 800, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-270, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2300);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-270, 200);
    chassis.resetLocalPosition();  
  //  pros::delay(10000000000);
    matchloader.set_value(true);   
    chassis.moveToPoint(29.5, -0.25, 800, {.maxSpeed = 60});
    //chassis.waitUntilDone();
    moveStraight(6000, 1850, {.maxSpeed = 30});
    //pros::delay(1500); 
    chassis.moveToPoint(-2, -0.25, 1200, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(90, 100);
    matchloader.set_value(false);
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    pros::delay(2000);
   // top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 100);
    chassis.resetLocalPosition();
    //pros::delay(1000000);
   // moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
    chassis.moveToPoint(33, -25, 2500, {.maxSpeed = 80, .minSpeed =1 });
    top = 0;
    chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 300);
    //chassis.turnToHeading(160, 200);
    moveStraight(2000, 600, {.maxSpeed = 127});
    chassis.waitUntilDone();
        while (front.get() > 900){
        chassis.tank(95, 100);
        if (front.get() < 1450 && front.get() > 1375){
            matchloader.set_value(true);
        }
    }  
    //chassis.driveToHeading(-20, 180, 300);
    chassis.moveToPose(chassis.getPose().x, chassis.getPose().y+17, 180, 800, {.forwards = false, .lead = 0.1});
        chassis.turnToHeading(180, 800);
        chassis.waitUntilDone();
       // pros::delay(50);
        chassis.setPose(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta)), chassis.getPose().y, chassis.getPose().theta);
        
        chassis.turnToHeading(270, 800);
        matchloader.set_value(false);
    //moveStraight(5, 700, {.maxSpeed = 127});
    matchloader.set_value(false);
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone(); 
   // pros::delay(50);  
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    //pros::delay(10000000000);
    chassis.moveToPoint(31, -18, 2000);
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
        chassis.turnToHeading(135, 300);
    chassis.moveToPoint(10.5, -11.5, 2000, {.forwards = false});
    chassis.turnToHeading(135, 400);
   
    moveStraight(-20, 200, {.maxSpeed = 50});
    chassis.waitUntilDone();
    //chassis.turnToHeading(135, 400);
    ramp.set_value(true);
    //idk.set_value(false);   
    //pros::delay(200);
    //chassis.waitUntilDone();
    top = 5;
    
    //top = 5;
    pros::delay(2300);
        chassis.turnToHeading(135, 300);
        moveStraight(-2, 200, {.maxSpeed = 40});

    chassis.moveToPoint(46.3, -45, 2000);
    chassis.turnToHeading(90, 800);
    matchloader.set_value(true);  
    ramp.set_value(false);
    top = 0;
    chassis.waitUntilDone();
    //pros::delay(50);
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);

   // pros::delay(10000000000);
    
    chassis.moveToPoint(66, -48, 2000, {.maxSpeed = 60});

            chassis.waitUntilDone();
    pros::delay(150);
    moveStraight(2, 300, {.maxSpeed = 80});
    chassis.turnToHeading(90, 300);
    moveStraight(2000, 800, {.maxSpeed = 40});
    chassis.moveToPoint(37, -68, 500, {.forwards = false});
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
   // pros::delay(50);
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    matchloader.set_value(false);

    chassis.moveToPoint(-20, -67, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(0, 800, {.minSpeed =1, .earlyExitRange = 3});
    moveStraight(9, 1000,{.maxSpeed = 127});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
   // pros::delay(50);
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    chassis.moveToPoint(-5, -54, 800, {.forwards = false});
    chassis.turnToHeading(-90, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2500);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();  
    matchloader.set_value(true);
    chassis.moveToPoint(-32.5, 0, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    moveStraight(1000, 1000, {.maxSpeed = 40});  
    chassis.moveToPoint(2, -0.25, 1500, {.forwards = false, .maxSpeed = 75});
    chassis.turnToHeading(-90, 100);
    matchloader.set_value(false);   
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    pros::delay(2300);
    top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 100);
    chassis.resetLocalPosition();

    //moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
    chassis.moveToPoint(-33, 25, 2500, {.maxSpeed = 80, .minSpeed = 1});
    chassis.swingToHeading(-15 , lemlib::DriveSide::RIGHT, 300);
    //chassis.turnToHeading(-20, 100);
    moveStraight(200, 800, {.maxSpeed = 127});

    


    pros::delay(1000000);
}

void skills(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        ramp.set_value(false);

    intakeTime = 1500000;
        top = 0;
    top = 0;
    matchloader.set_value(false);              
    antiJam = false;
    chassis.setPose(0,0, 90);
    chassis.moveToPoint(29, 2, 3000, {.maxSpeed = 80});
    chassis.turnToHeading(-42, 600);
    //pros::delay(1000000);
    chassis.moveToPoint(36, -7.0, 1000, {.forwards = false});
    moveStraight(-10, 200, {.maxSpeed = 40});
    chassis.turnToHeading(-45, 400);

    ramp.set_value(true);
    //idk.set_value(false);
    chassis.waitUntilDone();
   // top = 5;
    // pros::delay(10000000000);

    top = 5;
    pros::delay(1000);
        matchloader.set_value(true);

    chassis.moveToPoint(-1.6, 31.34, 1500);
    top = 0;
    chassis.turnToHeading(-90, 600);
    chassis.waitUntilDone();
    //pros::delay(50);
    chassis.setPose((72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);
    //chassis.setPose(72-(left.get()/25.4), 72-(front.get()/25.4), chassis.getPose().theta);
   
    top = 0;
    ramp.set_value(false);
    chassis.moveToPoint(25, 48.5, 800, {.maxSpeed = 60});

    //pros::delay(10000000000);
    
   

       // chassis.waitUntilDone();
    //pros::delay(150);
    chassis.turnToHeading(-90, 200);
    moveStraight(2, 1100, {.maxSpeed = 40});
    //
    pros::delay(1300);
    chassis.moveToPoint(48, 66, 550, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
    //pros::delay(50);  
    chassis.setPose(-(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);
    matchloader.set_value(false);
    chassis.moveToPoint(30, 68, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(180, 800);
    moveStraight(9, 1000,{.maxSpeed = 40});  
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
    //pros::delay(50);
    chassis.setPose(chassis.getPose().x, 72-get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta), chassis.getPose().theta);
    chassis.moveToPoint(20, 49.25, 800, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-270, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2300);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-270, 200);
    chassis.resetLocalPosition();  
  //  pros::delay(10000000000);
    matchloader.set_value(true);   
    chassis.moveToPoint(29.5, 0, 800, {.maxSpeed = 60});
    //chassis.waitUntilDone();
    moveStraight(6000, 2000, {.maxSpeed = 30});
    //pros::delay(1500); 
    chassis.moveToPoint(-2, -0.25, 1200, {.forwards = false, .maxSpeed = 100});
    chassis.turnToHeading(90, 100);
    matchloader.set_value(false);
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    pros::delay(2000);
   // top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 100);
    chassis.resetLocalPosition();
    //pros::delay(1000000);
    moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
    chassis.moveToPoint(33, -25, 2500, {.maxSpeed = 80});
    top = 0;
    chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 400);
    chassis.turnToHeading(160, 200);
    moveStraight(2000, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();
        while (front.get() > 900){
        chassis.tank(95, 100);
        if (front.get() < 1450 && front.get() > 1375){
            matchloader.set_value(true);
        }
    }  
    //chassis.driveToHeading(-20, 180, 300);
    chassis.moveToPose(chassis.getPose().x, chassis.getPose().y+17, 180, 800, {.forwards = false, .lead = 0.1});
        chassis.turnToHeading(180, 800);
        chassis.waitUntilDone();
       // pros::delay(50);
        chassis.setPose(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta)), chassis.getPose().y, chassis.getPose().theta);
        
        chassis.turnToHeading(270, 800);
        matchloader.set_value(false);
    moveStraight(5, 700, {.maxSpeed = 127});
    matchloader.set_value(false);
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone(); 
   // pros::delay(50);  
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    //pros::delay(10000000000);
    chassis.moveToPoint(31, -18, 2000);
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(20.5, -14.5, 2000, {.forwards = false});
    chassis.turnToHeading(135, 400);
   
    moveStraight(-20, 400, {.maxSpeed = 50});
    chassis.waitUntilDone();
    //chassis.turnToHeading(135, 400);
    ramp.set_value(true);
    //idk.set_value(false);   
    //pros::delay(200);
    //chassis.waitUntilDone();
    top = 4;
    
    //top = 5;
    pros::delay(2300);
        chassis.turnToHeading(135, 800);
        moveStraight(-2, 200, {.maxSpeed = 40});

    chassis.moveToPoint(46.3, -44, 2000);
    chassis.turnToHeading(90, 800);
    matchloader.set_value(true);  
    ramp.set_value(false);
    top = 0;
    chassis.waitUntilDone();
    //pros::delay(50);
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);

   // pros::delay(10000000000);
    
    chassis.moveToPoint(66, -48, 2000, {.maxSpeed = 60});

            chassis.waitUntilDone();
    pros::delay(150);
    moveStraight(2, 300, {.maxSpeed = 80});
    chassis.turnToHeading(90, 300);
    moveStraight(2000, 1000, {.maxSpeed = 40});
    chassis.moveToPoint(37, -68, 600, {.forwards = false});
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
   // pros::delay(50);
    chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    matchloader.set_value(false);

    chassis.moveToPoint(-30, -67, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(0, 800);
    moveStraight(9, 1000,{.maxSpeed = 127});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
   // pros::delay(50);
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    chassis.moveToPoint(-5, -54, 800, {.forwards = false});
    chassis.turnToHeading(-90, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2500);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();  
    matchloader.set_value(true);
    chassis.moveToPoint(-32.5, 0, 2000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    moveStraight(1000, 1000, {.maxSpeed = 40});  
    chassis.moveToPoint(2, -0.25, 1500, {.forwards = false, .maxSpeed = 75});
    chassis.turnToHeading(-90, 100);
    matchloader.set_value(false);   
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    pros::delay(2300);
    top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 100);
    chassis.resetLocalPosition();

    //moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
        chassis.moveToPoint(-33, 25, 2500, {.maxSpeed = 80, .minSpeed = 1});
    chassis.swingToHeading(0 , lemlib::DriveSide::RIGHT, 300);
    //chassis.turnToHeading(-20, 100)   
    moveStraight(200, 800, {.maxSpeed = 127});


    


    pros::delay(1000000);
}
void autonomous(){

  skills();

}

//chassis.leftMotors->move(127);
/**
 * Runs in driver control
 */
bool down = false;
void matchload(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        down = !down;
        matchloader.set_value(down);
        pros::delay(250);
    }


    //static pros::adi::DigitalOut clamp('A'); // Replace 'A' with your actual digital port

}
void first(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        intakeTime = 1500000;
        ramp.set_value(true);
        pros::delay(100);
        top = 5;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intakeTime = 1500000;
        ramp.set_value(false);
        pros::delay(100);
        top = 0;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        intakeTime = 1500000;
        outake =1;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        intakeTime = 15000000;
        ramp.set_value(false);
        pros::delay(100);
        top = 1;
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        intakeTime = 0;
        roller.move(0);
    }
}
void dingus(){
    matchloader.set_value(false);
    ramp.set_value(false);
    intakeTime = 15000000;
    top = 0;
    chassis.setPose(0, 0, 90);
    chassis.resetLocalPosition();
    //pros::delay(1000000);
    moveStraight(3, 500, {.maxSpeed = 127, .minSpeed = 1});
    chassis.moveToPoint(33, -25, 2500, {.maxSpeed = 80});
    top = 0;
    chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 400);
    chassis.turnToHeading(160, 200);
    moveStraight(2000, 600, {.maxSpeed = 127});
    chassis.waitUntilDone();
        while (front.get() > 900){
        chassis.tank(95, 100);
        if (front.get() < 1450 && front.get() > 1375){
            matchloader.set_value(true);
        }
    }  
    //chassis.driveToHeading(-20, 180, 300);
    chassis.moveToPose(chassis.getPose().x, chassis.getPose().y+17, 180, 800, {.forwards = false, .lead = 0.1});
        chassis.turnToHeading(180, 800);
        chassis.waitUntilDone();
       // pros::delay(50);
        chassis.setPose(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta)), chassis.getPose().y, chassis.getPose().theta);
        
        chassis.turnToHeading(270, 800);
        matchloader.set_value(false);
    moveStraight(5, 700, {.maxSpeed = 127});
    matchloader.set_value(false);
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone(); 
   // pros::delay(50);  
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    //pros::delay(10000000000);
    chassis.moveToPoint(31, -18, 2000);
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(20.5, -14.5, 2000, {.forwards = false});
    chassis.turnToHeading(135, 400);
   
    moveStraight(-20, 400, {.maxSpeed = 50});
    chassis.waitUntilDone();
    //chassis.turnToHeading(135, 400);
    ramp.set_value(true);
    //idk.set_value(false);   
    //pros::delay(200);
    //chassis.waitUntilDone();
    top = 5;
    
    //top = 5;
    pros::delay(2300);
        chassis.turnToHeading(135, 800);
        moveStraight(-2, 200, {.maxSpeed = 40});

}
void dib(){
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        dingus();
    }
}
void opcontrol() {
    // controller
    // loop to continuously update motors
    //skills();
    pidtest();   



    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with curvature drive
        chassis.tank(leftY, rightX);
            matchload();
        first();
        dib();
        // delay to save resources
        pros::delay(10);
    }
}
