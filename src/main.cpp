#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-10, -9, -1},pros::MotorGearset::blue); // left motor group - ports 4 (reversed), 1 (reversed), 14 (reversed)
pros::MotorGroup rightMotors({7, 8, 2}, pros::MotorGearset::blue); // right motor group - ports 6, 3, 12 
pros::Motor li(20);
pros::Motor ri(19);
pros::Motor roller(-12);
pros::Distance left(16);
pros::Distance right(17);
pros::Distance front(18);
pros::Imu imu(6); 

// pneumatics
static pros::adi::DigitalOut matchloader('C');
static pros::adi::DigitalOut topp('H');
static pros::adi::DigitalOut bottom('G');
static pros::adi::DigitalOut descore('D');


//variables
int intakeTime = 0;
bool antiJam = false;
int outake = 0;
int top = 0;
int count = 0;  

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

CustomIMU my_imu(6, 1.00889791915);

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
            pros::lcd::print(3, "pitch: %f", fabs(my_imu.get_pitch()));
            pros::lcd::print(4, "roll: %f", fabs(my_imu.get_roll()));
            printf("(&%f,%f),\n",chassis.getPose().x, chassis.getPose().y);

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    pros::Task dingus([&](){
        if (count == 1){
            while (true){
                if ((my_imu.get_pitch() > 10) || (my_imu.get_roll() > 10)){
                    chassis.tank(80, 80);
                }
                pros::delay(10);
            }
        }
        

    });
    pros::Task anti_jam([&](){
        int stuckStart = -1;   // time when stall was first detected

        while (true) {
            if (intakeTime > 0 && top !=3) {
                double vel1 = fabs(li.get_actual_velocity());
                double vel2 = fabs(ri.get_actual_velocity());

                if (vel1 < 50 || vel2 < 50) {  // possible jam
                    if (stuckStart == -1) stuckStart = pros::millis();

                    if (pros::millis() - stuckStart >= 50) {
         
                        antiJam = true;
                        li.move(127);
                        ri.move(-127);   // reverse to clear jam
                        pros::delay(100);
                        li.move(-127); 
                        ri.move(127);   // resume forward
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

                if (intakeTime > 0 && top == 1){
                    topp.set_value(false);
                    bottom.set_value(false);
                    ri.move(127);
                    li.move(-127);


                    intakeTime -= 10;
                }
                else if (intakeTime > 0 && top == 0){
                    topp.set_value(true);
                    bottom.set_value(false);

                    ri.move(127);
                    li.move(-127);


                    intakeTime -= 10;
                }
                else if (intakeTime > 0 && top == 2){
                    topp.set_value(true);
                    bottom.set_value(true);
                    ri.move(84);
                    li.move(-84);


                    intakeTime -= 10;
                }
                else if (intakeTime > 0 && top == 3){
                    topp.set_value(true);
                    bottom.set_value(false);

                    ri.move(0);
                    li.move(0);


                    intakeTime -= 10;
                }
            }               
            pros::delay(10);
        } 
    });
    
}
void driveforward(){
    chassis.tank(95, 100);
    pros::delay(50);
    chassis.waitUntilDone();
}

struct SensorConfig {
    float x_off;      // Robot-frame forward offset (positive = toward front)
    float y_off;      // Robot-frame lateral offset (positive = toward left)
    float mount_side; // Direction the sensor faces, in robot-frame degrees (0=front, 90=left, 180=back, 270=right)
};

// Robot-frame offsets: x = forward, y = left
SensorConfig leftSensorConfig  = {5,    4,   90};  // Faces left
SensorConfig rightSensorConfig = {1.5,   -5.3, 270}; // Faces right
SensorConfig frontSensorConfig = {4,   -6.5, 0};   // Faces front

// 1. Finds the closest cardinal wall heading (0, 90, 180, 270)
//    Returns a value in [0, 360)
float get_wall_angle(float robot_heading, float mount_side) {
    float global_sensor_direction = robot_heading + mount_side;
    // Normalize before rounding to avoid negative or >360 results
    global_sensor_direction = std::fmod(global_sensor_direction, 360.0f);
    if (global_sensor_direction < 0) global_sensor_direction += 360.0f;

    float wall = std::fmod(std::round(global_sensor_direction / 90.0f) * 90.0f, 360.0f);
    return wall;
}

// 2. Corrected distance formula using robot-frame offsets
float get_dist_to_wall(float raw_mm, SensorConfig config, float current_theta) {
    float d_sensor = raw_mm / 25.4f; // mm -> inches

    float mount_rad = lemlib::degToRad(config.mount_side);

    // Sensor reading tip in robot frame:
    // Start at sensor mount position (x_off, y_off), extend d_sensor in mount direction
    float tip_x = config.x_off + d_sensor * std::cos(mount_rad);
    float tip_y = config.y_off + d_sensor * std::sin(mount_rad);

    // Wall normal direction in robot frame
    float wall_angle     = get_wall_angle(current_theta, config.mount_side);
    float wall_normal_rad = lemlib::degToRad(wall_angle - current_theta);

    // Project the tip onto the wall normal to get perpendicular distance
    float perp_dist = tip_x * std::cos(wall_normal_rad) + tip_y * std::sin(wall_normal_rad);

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
    intakeTime = 300000;
    chassis.setPose(0,0,90);
    antiJam = false;
    top = 0;
    chassis.moveToPose(28, -22, 160, 2500, {.lead = 0.4,.minSpeed = 127});

    
    moveStraight(2000, 1200, {.maxSpeed = 127});
    chassis.waitUntilDone();
    bool passed = false;
    while (front.get() > 976){ 


        if (front.get() > 1626) {
            chassis.tank(110, 115);
        } 
        if (front.get() < 1626 && front.get() > 1526) {
            
                passed = true;
                matchloader.set_value(true);
                chassis.tank(90, 90);
            
        } 
        if (front.get() < 1526 && passed) {
            matchloader.set_value(true);
            chassis.tank(80, 80);
        }
    }
    
    chassis.moveToPose(chassis.getPose().x, chassis.getPose().y+15, 180,    800, {.forwards = false, .lead = 0.1});
        chassis.turnToHeading(180, 800);
        chassis.waitUntilDone();

        chassis.setPose(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta)), chassis.getPose().y, chassis.getPose().theta);
        
        chassis.turnToHeading(270, 800);
        matchloader.set_value(false);
    moveStraight(5, 700, {.maxSpeed = 127});
    matchloader.set_value(false);
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone(); 
 
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    //+1 ball from 4 stack
    chassis.moveToPoint(31, -21, 2000);
    chassis.turnToHeading(90, 900);
    chassis.waitUntilDone();
    chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(13.5, -14.5, 2000, {.forwards = false});
    chassis.turnToHeading(135, 400);
    //score middle
   
    moveStraight(-20, 200, {.maxSpeed = 50});
    chassis.waitUntilDone();

    top = 2;


    pros::delay(2700);

        chassis.turnToHeading(135, 800);
    //go loader

    chassis.moveToPoint(46.3, -42.5, 2000);
    pros::delay(1000);
    descore.set_value(true);
    chassis.turnToHeading(90, 800);
    matchloader.set_value(true);  

        top = 1;

    chassis.waitUntilDone();
    descore.set_value(false);

        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);


   top = 0;
    
    chassis.moveToPoint(66, -48, 2000, {.maxSpeed = 50});

            chassis.waitUntilDone();
    pros::delay(150);
    moveStraight(2, 300, {.maxSpeed = 80});
    chassis.turnToHeading(90, 300);
    moveStraight(2000, 300, {.maxSpeed = 40});
    chassis.moveToPoint(37, -68, 600, {.forwards = false});
    // alley
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();

    chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);

    chassis.moveToPoint(-29, -67, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(0, 800);
    moveStraight(9, 1000,{.maxSpeed = 127});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();

    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    chassis.moveToPoint(-5, -54, 600, {.forwards = false});
    //long goal score
    chassis.turnToHeading(-90, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2200);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();  
    matchloader.set_value(true);
    chassis.moveToPoint(-30.5, 0, 2000, {.maxSpeed = 58});
    //matchloader 
    chassis.waitUntilDone();
    moveStraight(1000, 1000, {.maxSpeed = 40});  
    chassis.moveToPoint(4, -0.25, 1200, {.forwards = false, .maxSpeed = 110});
    chassis.turnToHeading(-90, 100);
    matchloader.set_value(false);   
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    //long goal score
    pros::delay(2300);
    top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();

    //go to park
    chassis.moveToPose(-29, 26,5, 2500, {.lead = 0.4,.minSpeed = 80});

    moveStraight(100000, 1000, {.maxSpeed = 127});
    matchloader.set_value(true);
    pros::delay(1000);
    matchloader.set_value(false);

    
    


    pros::delay(1000000);
    
    


    pros::delay(1000000);




 

    //chassis.swingToHeading(0 , lemlib::DriveSide::RIGHT, 500);
    

    //chassis.tank(90, 100);
    //pros::delay(600);

    //chassis.tank(0, 0);
    

    //moveStraight(-100, 200, {.maxSpeed = 127});

   // 1.22, 25.65
   pros::delay(10000000000);
}

void skills(){
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    

    intakeTime = 1500000;

    top = 0;
    matchloader.set_value(false);              
    antiJam = false;                
    descore.set_value(false);
    chassis.setPose(0,0, 90);
    //get 1 more red from first 4 stack
    chassis.moveToPoint(26.5, 2, 3000, {.maxSpeed = 80});
    chassis.waitUntil(23.5);
    top = 3;
    chassis.turnToHeading(-42, 600);
    //pros::delay(1000000);
    chassis.moveToPoint(31.8, -5.0, 300, {.forwards = false});
    top = 2;
    //score middle
    moveStraight(-10, 250, {.maxSpeed = 40});
    chassis.turnToHeading(-45, 250);



    chassis.waitUntilDone();

    pros::delay(1000);
        matchloader.set_value(true);

    // go loader
    chassis.moveToPoint(-1.6, 23.5, 1500);
    top = 0;
    descore.set_value(true);
    chassis.turnToHeading(-90, 600);
    chassis.waitUntilDone();
    descore.set_value(false);

    chassis.setPose((72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);

   


    chassis.moveToPoint(25, 48.5, 800, {.maxSpeed = 58});

    chassis.turnToHeading(-90, 200);
    moveStraight(2, 1100, {.maxSpeed = 40});
    //
    pros::delay(1300);
    //alley
    moveStraight(-2, 500, {.maxSpeed = 127, .minSpeed = 127});
    chassis.moveToPoint(48, 69, 650, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();
  
    chassis.setPose(-(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta))), 72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta)), chassis.getPose().theta);
    matchloader.set_value(false);
    chassis.moveToPoint(30, 68, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(180, 800);
    moveStraight(9, 1000,{.maxSpeed = 40});  
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();
  
    chassis.setPose(chassis.getPose().x, 72-get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta), chassis.getPose().theta);
    chassis.moveToPoint(20, 49.25, 700, {.forwards = false, .maxSpeed = 80});
    // long score
    chassis.turnToHeading(-270, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2300);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-270, 200);
    chassis.resetLocalPosition();  

    matchloader.set_value(true);   
    //loader
    chassis.moveToPoint(30.5, 0, 800, {.maxSpeed = 60});

    moveStraight(6000, 2000, {.maxSpeed = 30});

    chassis.moveToPoint(-4, -0.25, 1000, {.forwards = false, .maxSpeed = 110});
    chassis.turnToHeading(90, 100);
    matchloader.set_value(false);
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    pros::delay(2300);
   //score long
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 100);
    
    chassis.resetLocalPosition();
    //go to park
    chassis.moveToPose(28, -22, 160, 2500, {.lead = 0.4,.minSpeed = 127});

    top = 0;
    moveStraight(2000, 1200, {.maxSpeed = 127});
    chassis.waitUntilDone();
    bool passed = false;
    while (front.get() > 976){ 


        if (front.get() > 1626) {
            chassis.tank(110, 115);
        } 
        if (front.get() < 1626 && front.get() > 1526) {
            
                passed = true;
                matchloader.set_value(true);
                chassis.tank(90, 90);
            
        } 
        if (front.get() < 1526 && passed) {
            matchloader.set_value(true);
            chassis.tank(80, 80);
        }
    }
    
    chassis.moveToPose(chassis.getPose().x, chassis.getPose().y+15, 180,    800, {.forwards = false, .lead = 0.1});
        chassis.turnToHeading(180, 800);
        chassis.waitUntilDone();

        chassis.setPose(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta)), chassis.getPose().y, chassis.getPose().theta);
        
        chassis.turnToHeading(270, 800);
        matchloader.set_value(false);
    moveStraight(5, 700, {.maxSpeed = 127});
    matchloader.set_value(false);
    chassis.turnToHeading(270, 800);
    chassis.waitUntilDone(); 
 
    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    //+1 ball from 4 stack
    chassis.moveToPoint(31, -21, 2000);
    chassis.turnToHeading(90, 900);
    chassis.waitUntilDone();
        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)),chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(13.5, -14.5, 2000, {.forwards = false});
    chassis.turnToHeading(135, 400);
    //score middle
   
    moveStraight(-20, 200, {.maxSpeed = 50});
    chassis.waitUntilDone();

    top = 2;
    

    pros::delay(2700);

        chassis.turnToHeading(135, 800);
    //go loader

    chassis.moveToPoint(46.3, -42.5, 2000);
    pros::delay(1000);
    descore.set_value(true);
    chassis.turnToHeading(90, 800);
    matchloader.set_value(true);  

        top = 1;

    chassis.waitUntilDone();
    descore.set_value(false);

        chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);


   top = 0;
    
    chassis.moveToPoint(66, -48, 2000, {.maxSpeed = 50});

            chassis.waitUntilDone();
    pros::delay(150);
    moveStraight(2, 300, {.maxSpeed = 80});
    chassis.turnToHeading(90, 300);
    moveStraight(2000, 300, {.maxSpeed = 40});
    chassis.moveToPoint(37, -68, 600, {.forwards = false});
    // alley
    chassis.turnToHeading(90, 800);
    chassis.waitUntilDone();

    chassis.setPose(72-(get_dist_to_wall(front.get(), frontSensorConfig, chassis.getPose().theta)), -(72-(get_dist_to_wall(right.get(), rightSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);

    chassis.moveToPoint(-29, -67, 3000, {.forwards = false, .minSpeed = 1});
    chassis.turnToHeading(0, 800);
    moveStraight(9, 1000,{.maxSpeed = 127});
    chassis.turnToHeading(-90, 800);
    chassis.waitUntilDone();

    chassis.setPose(chassis.getPose().x, -(72-(get_dist_to_wall(left.get(), leftSensorConfig, chassis.getPose().theta))), chassis.getPose().theta);
    chassis.moveToPoint(-5, -54, 600, {.forwards = false});
    //long goal score
    chassis.turnToHeading(-90, 100);
    moveStraight(-8, 2000, {.maxSpeed = 127});


    top = 1;
    pros::delay(2200);
    chassis.waitUntilDone();
    top = 0;
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();  
    matchloader.set_value(true);
    chassis.moveToPoint(-30.5, 0, 2000, {.maxSpeed = 58});
    //matchloader 
    chassis.waitUntilDone();
    moveStraight(1000, 1000, {.maxSpeed = 40});  
    chassis.moveToPoint(4, -0.25, 1200, {.forwards = false, .maxSpeed = 110});
    chassis.turnToHeading(-90, 100);
    matchloader.set_value(false);   
    moveStraight(-8, 2000, {.maxSpeed = 127});
    top = 1;
    //long goal score
    pros::delay(2300);
    top = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 200);
    chassis.resetLocalPosition();

    //go to park
    chassis.moveToPose(-29, 26,5, 2500, {.lead = 0.4,.minSpeed = 80});

    moveStraight(100000, 1000, {.maxSpeed = 127});
    matchloader.set_value(true);
    pros::delay(1000);
    matchloader.set_value(false);

    
    


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
 
        // delay to save resources
        pros::delay(10);
    }
}
