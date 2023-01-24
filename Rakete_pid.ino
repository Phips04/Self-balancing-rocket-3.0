#include <Servo.h>
#include <Wire.h>
#include <math.h>

//-----------UNPOWERED TESTS ONLY------------------

//IMU placement:
//X aligned with pitch
//Y aligned ith roll


//global variables
Servo s1, s2, s3, s4;

double rad_to_deg = 180.0 / M_PI;
double IMU_raw_to_g_force = 1 / 16384.0;
double IMU_raw_to_SI_angular_velocity = 1 / 131.0;

double k_acc_content = 0.05;
double k_gyro_content = 0.95;

bool invert_axis_orientation = true;

bool invert_pitch = true;
bool invert_roll = false;

int s1_leveling_trim = -5;
int s2_leveling_trim = 5;
int s3_leveling_trim = -2;
int s4_leveling_trim = 2;

double pitch_desired_val = 0.0;
double roll_desired_val = 0.0;

float elapsed, current_time, pre_time;

double pre_pitch = 0.0;
double pre_roll = 0.0;

int iteration = 0;

//aux functions
double pythagoras(double x, double y) //=> c
{
    return sqrt(x*x + y*y);
}

double remove_non_linear_servo_elongation(double angle)
{
    return 180 / M_PI * asin(angle / 90);
}

void move_to_position(double pitch, double roll)
{
  if(pitch >= -90 && pitch <= 90 && roll >= -90 && roll <= 90)
  {
    if(invert_pitch) { pitch = pitch * (-1); }
    if(invert_roll) { roll = roll * (-1); }

    if(invert_axis_orientation)
    {
      s1.write(92 + remove_non_linear_servo_elongation(roll));
      s2.write(92 - remove_non_linear_servo_elongation(roll));

      s3.write(92 + remove_non_linear_servo_elongation(pitch));
      s4.write(92 - remove_non_linear_servo_elongation(pitch));
    }
    else
    {
      s1.write(92 + remove_non_linear_servo_elongation(pitch));
      s2.write(92 - remove_non_linear_servo_elongation(pitch));

      s3.write(92 + remove_non_linear_servo_elongation(roll));
      s4.write(92 - remove_non_linear_servo_elongation(roll));
    }
  }
}

void setup()
{
    //setup serial com
    Serial.begin(9600);

    //setup i2c com
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    //setup servos
    s1.attach(6);
    s2.attach(9);
    s3.attach(10);
    s4.attach(11);

    s1.write(92);
    s2.write(92);
    s3.write(92);
    s4.write(92);

    //delay startup
    delay(3000);
}

void loop()
{
    pre_time = current_time;
    current_time = millis();
    elapsed = (current_time - pre_time) / 1000.0;

    //request from IMU
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);

    //extract raw data
    int16_t accX = Wire.read()<<8|Wire.read();
    int16_t accY = Wire.read()<<8|Wire.read();
    int16_t accZ = Wire.read()<<8|Wire.read();
    int16_t temp = Wire.read()<<8|Wire.read();
    int16_t gyrX = Wire.read()<<8|Wire.read();
    int16_t gyrY = Wire.read()<<8|Wire.read();
    int16_t gyrZ = Wire.read()<<8|Wire.read();

    //Scale acceleration => terrestrial acc
    double accX_g = accX * IMU_raw_to_g_force;
    double accY_g = accY * IMU_raw_to_g_force;
    double accZ_g = accZ * IMU_raw_to_g_force;

    //Compute acc dependend pitch / roll
    double acc_pitch_angle = atan(accY_g / pythagoras(accX_g, accZ_g)) * rad_to_deg;
    double acc_roll_angle = atan(-1.0 *(accX_g / pythagoras(accY_g, accZ_g))) * rad_to_deg;

    //Get gyro based angles
    double gyro_pitch_angle = pre_pitch + gyrX * IMU_raw_to_SI_angular_velocity * elapsed;
    double gyro_roll_angle = pre_roll + gyrY * IMU_raw_to_SI_angular_velocity * elapsed;

    //get final angles
    double final_pitch_angle = acc_pitch_angle * k_acc_content + gyro_pitch_angle * k_gyro_content;
    double final_roll_angle = acc_roll_angle * k_acc_content + gyro_roll_angle * k_gyro_content;

    pre_pitch = final_pitch_angle;
    pre_roll = final_roll_angle;

    double pitch_error = final_pitch_angle - pitch_desired_val;
    double roll_error = final_roll_angle - roll_desired_val;

    //PID gains
    double p_gain = 1.0;
    double i_gain = 0.0;
    double d_gain = 0.0;

    //PID vals
    double p_pitch_val = 0.0;
    double i_pitch_val = 0.0;
    double d_pitch_val = 0.0;

    double p_roll_val = 0.0;
    double i_roll_val = 0.0;
    double d_roll_val = 0.0;

    //PID computations
    p_pitch_val = pitch_error * 1.5;
    p_roll_val = roll_error * 1.5;

    //PID final computation
    double pitch_compensation = p_pitch_val * p_gain + i_pitch_val * i_gain + d_pitch_val * d_gain;
    double roll_compensation = p_roll_val * p_gain + i_roll_val * i_gain + d_roll_val * d_gain;

    //update servos / check for calibration state
    if(iteration < 100)
    {       
        iteration++;
        move_to_position(0.0, 0.0);
    }
    else if(iteration == 100)
    {
        iteration++;
        pitch_desired_val = final_pitch_angle;
        roll_desired_val = final_roll_angle;
    }
    else
    {
        move_to_position(pitch_compensation, roll_compensation);
    }

    //output results
    //Serial.print("; final_pitch: ");
    Serial.print(pitch_error);
    Serial.print(" ");
    //Serial.print("; final_roll: ");
    Serial.print(roll_error);
    Serial.print(" ");
    //Serial.print("; iteration: ");
    Serial.println(iteration);
    //Serial.println("###################");

    delay(50);
}