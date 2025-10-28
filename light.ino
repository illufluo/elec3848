/*****************************************************
IDP Feedback and control of servo motor     (V20220120)
Usage of servo motor control library and ADC for tracking the light source 

*****************************************************/
#include <Servo.h> 

#define SERVO_PIN 9        //servo connected at pin 9
#define TOL   1           //tolerance for adc different, avoid oscillation
#define K   5              //Step size

Servo myservo;            //declare servo object

int int_position=90;      //Servo initial position

//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_m ;
int int_adc1, int_adc1_m;
float int_adc0_c,int_adc1_c;
float int_left, int_right;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  myservo.attach(SERVO_PIN);
  myservo.write(int_position);
  Serial.println();

  // measure the sensors reading at ambient light intensity  
  Serial.println("Calibration in progress, put the sensors under the light (~ 5 sec) ......");
  Serial.println("***********************");
  delay(5000);        // delay 5000 ms

  int_adc0=analogRead(A0);   // Left sensor at ambient light intensity
  int_adc1=analogRead(A1);   // Right sensor at ambient light intensity
  Serial.print("Left : ");
  Serial.println(int_adc0);
  Serial.print("Right : ");
  Serial.println(int_adc1);
  delay(1000); 

  Serial.println("\nCalibration in progress, cover the sensors with your fingers (~ 8 sec to set)......");
  Serial.println("************ Put Fingers *****************");
  delay(5000);        // delay 5000 ms
  Serial.println("********* START Calibration **************");

  // measure the sensors reading at zero light intensity  
  int_adc0_c=analogRead(A0);   // Left sensor at zero light intensity
  int_adc1_c=analogRead(A1);   // Right sensor at zero light intensity
  Serial.print("Left : ");
  Serial.println(int_adc0_c);
  Serial.print("Right : ");
  Serial.println(int_adc1_c);
  // calculate the slope of light intensity to ADC reading equations
  Serial.println(int_adc0);
  Serial.println(int_adc0_c);
  int_adc0_m=(int_adc0-int_adc0_c);
  int_adc1_m=(int_adc1-int_adc1_c);
  Serial.print("Left : ");
  Serial.println(int_adc0_m);
  Serial.print("Right : ");
  Serial.println(int_adc1_m);  
 
   
  Serial.println("\n******** Completed! Remove your hands ********");
  delay(4000);        // delay 4000 ms
}

void loop() 
{
  // put your main code here, to run repeatedly:

  // calculate the light intensity of the sensors
  // in the range of [0, 100]
  int_left=(analogRead(A0)-int_adc0_c)/int_adc0_m;
  int_right=(analogRead(A1)-int_adc1_c)/int_adc1_m;  

  Serial.print("Left sensor intensity = ");
  Serial.print(int_right);
  Serial.print(";  Right sensor intensity = ");
  Serial.print(int_left);
  Serial.print(";  Servo Speed (-90 to 90) = ");

  Serial.println(int_position-90);
  Serial.println(analogRead(A0));
  Serial.println(analogRead(A1));
  tracker();
  delay(100);        // delay 100 ms
  
}

void tracker()
{
 // If left sensor is brighter than right sensor, decrease servo angle and turn LEFT
  if (int_left>(int_right+TOL))
  {
    Serial.print("I turn left");

    return; 
       
  }

 // if right sensor is brighter than left sensor, increase servo angle and turn RIGHT
  if (int_left<(int_right-TOL))
  {
    Serial.print("I turn right");
    return;
      
  }
  Serial.print("I stay");
}
