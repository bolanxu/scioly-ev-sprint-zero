#include <SPI.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Encoder.h>
#include <math.h>
#include <Servo.h>

// Define the display width and height in pixels
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64

#define RPWM_PIN 11
#define LPWM_PIN 10

#define BTN_PIN 7

#define distMin 7
#define distMax 10
#define distInterval 0.25f

#define timeMin 10
#define timeMax 20
#define timeInterval 0.5f

float trackWidth = 0.1275;

float COUNTS_PER_METER = 1720.0;

float run_kP = 80;
float run_kD = 10;
float run_kI = 3;

float previous_error = 0;
unsigned long previous_time = 0;
float error_sum = 0;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long base_time;

float target_time = 0;

float target_dist = 0;

float checkpoint1_x;
float checkpoint2_x;
float checkpoint_y;
float final_checkpoint_y;

float current_point_x;
float current_point_y;

float current_heading = 0;

float currrent_ev_x = 0;
float currrent_ev_y = 0;

float turn_offset = 0;

float steering_threshold = 3;

int speed = 0;

int waypoint = 0;

Encoder encRot(8, 6);

Encoder enc1(2, 4);
Encoder enc2(3, 5);

Servo steeringServo;

long previous_pos1 = 0;
long previous_pos2 = 0;

float steering_kD = 3;

int dir = 0;

void setMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)
  {
    analogWrite(RPWM_PIN, pwm);
    analogWrite(LPWM_PIN, 0);
  }
  else
  {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwm);
  }
}

void stopMotor() {
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

float set_param(String prompt, float start_num, float interval)
{
  float param = 0;
  
  encRot.write(0);
  int counter = 0;
  while (1)
  {
    counter++;
    if (counter > 1000)
    {
      display.clearDisplay();
      display.setCursor(0,0); 
      display.println(prompt);
      display.println(param, 4);
      display.display();
      counter = 0;
    }
    param = start_num + float(encRot.read()/4)*interval;
    if (digitalRead(BTN_PIN) == 0)
    {
      break;
    }
  }
  
  return param;
}

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  

  // Initialize the SSD1306 display with the I2C address 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear the display buffer
  display.clearDisplay();

  // Set text size (1 is the smallest)
  display.setTextSize(2);
  // Set text color (white is typically on for monochrome displays)
  display.setTextColor(SSD1306_WHITE); 
  // Set the starting cursor position

  steeringServo.attach(9);

  steeringServo.write(89);

  target_dist = set_param("Set Dist:", distMin, distInterval);
  delay(500);
  target_time = set_param("Set Time:", timeMin, timeInterval);
  delay(500);

  target_time-=0.5;

  /*
  run_kP = set_param("Set KP:", 80, 1);
  delay(1000);
  run_kD = set_param("Set KD:", 10, 1);
  delay(1000);
  run_kI = set_param("Set KI:", 3, 0.1);
  delay(1000);
  
  steering_kD = set_param("Set S_kD:", 3, 0.1);
  delay(1000);
  */
  
  COUNTS_PER_METER = set_param("Set CPM", 1725.0, 1);
  delay(500);
  checkpoint_y = set_param("Set Can D", -0.8, 0.05);
  delay(500);
  steering_threshold = set_param("Set SteerT", 3, 1);
  delay(500);
  trackWidth = set_param("Set T_W", 0.1275, 0.001);
  delay(500);
  final_checkpoint_y = set_param("Set FinalY", -0.3, 0.01);

  checkpoint1_x = target_dist/2 - 0.6;
  checkpoint2_x = target_dist/2 + 0.6;
  //checkpoint_y = 0.5;

  current_point_x = 0;
  current_point_y = 0;
  
  base_time = millis();
  previous_time = millis();

  //target_dist = target_dist*COUNTS_PER_METER;
  //delay(500);

  enc1.write(0);
  enc2.write(0);
  
}

void loop() {

  float time_since_start = float(millis()-base_time)/1000.0;
  
  float desired_position = target_dist * (time_since_start / target_time);

  long pos1 = abs(enc1.read());
  long pos2 = abs(enc2.read());

  long d_pos1 = pos1 - previous_pos1;
  long d_pos2 = pos2 - previous_pos2;

  previous_pos1 = pos1;
  previous_pos2 = pos2;
  
  float currentPos = float(pos1 + pos2)/2;

  float d_currentPos = float(d_pos1+d_pos2)/2/COUNTS_PER_METER;

  float dist_diff = float(d_pos1-d_pos2);

  float d_heading = (dist_diff/COUNTS_PER_METER) / trackWidth;

  current_heading += d_heading;

  currrent_ev_x += d_currentPos * cos(current_heading);
  currrent_ev_y += d_currentPos * sin(current_heading);

  display.clearDisplay();
  display.setCursor(0,0); 
  
  float dist_to_cp1 = sqrt(pow(checkpoint1_x - currrent_ev_x,2) + pow(checkpoint_y - currrent_ev_y,2));
  float dist_to_cp2 = sqrt(pow(checkpoint2_x - currrent_ev_x,2) + pow(checkpoint_y - currrent_ev_y,2));

  if (waypoint == 0)
  {
    current_point_x = checkpoint1_x;
    current_point_y = checkpoint_y;
    
    display.println("Goto 1");
  
    if (dist_to_cp1 < 0.25)
    {
      waypoint = 1;
    }
  }
  else if (waypoint == 1)
  {
    current_point_x = checkpoint2_x;
    current_point_y = checkpoint_y;

    display.println("Goto 2");
  
    if (dist_to_cp2 < 0.25)
    {
      waypoint = 2;
    }
  }
  else
  {
    current_point_x = target_dist;
    current_point_y = final_checkpoint_y;
    display.println("Goto end");
  }

  float target_heading = atan2((current_point_y-currrent_ev_y),(current_point_x-currrent_ev_x));

  float heading_error = target_heading - current_heading;

  /*
  if (heading_error > PI) 
    heading_error -= 2*PI;
  if (heading_error < -PI) 
    heading_error += 2*PI;
  */
  
  turn_offset = degrees(heading_error) * steering_kD;

  if (turn_offset > 30)
  {
    turn_offset = 30;
  }
  else if (turn_offset < -25)
  {
    turn_offset = -25;
  }
  
  /*
  if (turn_offset > 15 )
  {
    dir = 1;
  }
  else if (turn_offset < -15 )
  {
    dir = 0;
  }

  if (dir == 0)
    turn_offset+=3;
  else
    turn_offset-=3;
  */
  
  steeringServo.write(89-int(turn_offset));
  
  unsigned long dt = millis()-previous_time;
  previous_time = millis();
  
  float error = (desired_position - currrent_ev_x);
  float error_rate = (error - previous_error) / (float(dt)/1000.0);
  previous_error = error;

  error_sum += error * (float(dt)/1000.0);

  float motor_speed = run_kP * error + run_kD * error_rate + error_sum * run_kI;

  if (currrent_ev_x > target_dist)
  {
    stopMotor();
    while(1);
  }

  /*
  if (digitalRead(BTN_PIN) == 0)
  {
    stopMotor();
    while(1);
  }
  */

  if (abs(turn_offset) > steering_threshold)
  {
    if (motor_speed > 35)
    {
      motor_speed = 35;
    }
  }

  setMotor(motor_speed);

  //Serial.println(motor_speed);
  
  // Print the text
  //display.println("Time");
  //display.println(time_since_start);
  //display.println(dist_diff);
  //display.display();

  //display.clearDisplay();
  //display.setCursor(0,0); 
  // Print the text
  // display.println("Time");
  display.println(motor_speed);
  
  display.println(turn_offset);
  //display.println(degrees(current_heading));
  display.println(time_since_start);
  display.display();
  
  delay(1);
}
