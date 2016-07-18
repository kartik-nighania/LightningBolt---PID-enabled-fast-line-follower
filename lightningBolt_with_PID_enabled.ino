  #include <QTRSensors.h>
   //************************************************************************************************
  #define SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
  #define EMITTER_PIN      2  // for Polulu QTR-8A
  #define leftMotorPin     9  // PWM
  #define rightMotorPin    11 // PWM 
  #define clockwiseLeft    1  // for motor driver IC
  #define clockwiseRight   2 
  #define baud             9600 // baud for serial communication   
  
  QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},  6 , SAMPLES_PER_SENSOR , EMITTER_PIN ); // A0-5 PIN
  bool displayReadings = true ;
  bool loopSense= false;  int blackDetectValue = 600 ;
  //************ **************************************************************************************
  bool  usePID = false;
  float KP = 3 ; //3
  float KI = 25000; //50000 ;
  float KD = 8/1;// 16/1 ;
   //*************************************************************************************************
  const int maxSpeedValue = 160; //200 //255 max
  const int max_diffrence = 20; //20
  const int factor_diffrence = 2;
  //******___PID___**********************************************************************************
  int   newError               = 0;
  int   integral               = 0;
  int   derivative             = 0;
  int   previous_error         = 0;
  int   center                 = ( 5 * 1000) / 2; // for 6 sensors 2500 is the centre point
  int   motorCorrection = 0 ;
  unsigned int currentPosition ;
  unsigned int sensorValues[6] ; // variable having sensor values
  unsigned int rawValues[6]; 
  //**************************************************************************************************
 // FUNCTIONS IN CODE
 
 void calibrateSensors(void);
 void showCalibratedData(void);
 void printSensorValues(void);
 void PID(void);
 void runJohnsonMotors(void);
 void followSimple(void);

//*****************************************************************************************************

   void setup()
   {  
    
     digitalWrite(clockwiseRight, HIGH);
     digitalWrite(clockwiseLeft,  HIGH);
     Serial.begin(baud);
  calibrateSensors();
     if(displayReadings)  showCalibratedData();
    }
   
   void loop()
    {
    
    // qtra.read(sensorValues); // uncomment to get RAW SENSOR VALUES
    currentPosition = qtra.readLine(sensorValues); //line position from 0-5000
    
    if(displayReadings) printSensorValues();
    
    if(usePID)   PID();          // this is where the magic happens!!
    else         followSimple(); // agar PID ki lagi padi ho. apna desi :P
    
    runJohnsonMotors();
    
    }

//******__functions_defined__*********************************************************************
 
 void calibrateSensors(void)

    {
    
    delay(500);
    pinMode(13, OUTPUT); // led on
    pinMode(12, OUTPUT); // buzzer on
    // digitalWrite(12, HIGH);   
    digitalWrite(13, HIGH); 
    
        for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
        {
          qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
        }
    
    // CALIBRATION END 
    digitalWrite(12, LOW); // buzzer and led off
    digitalWrite(13, LOW);     
  }

 void showCalibratedData()
{
  
  // print the calibration minimum values measured when emitters were on
    Serial.print("*********************************************************************************************");
    Serial.println();
    Serial.print("LOW: ");
          for (int i = 0; i < 6; i++)
        { 
          Serial.print(qtra.calibratedMinimumOn[i]);
           Serial.print("  ");
        }
        
  Serial.println();
  Serial.print("HIGH: ");
  
  // print the calibration maximum values measured when emitters were on
        for (int i = 0; i < 6; i++)
        { 
          Serial.print(qtra.calibratedMaximumOn[i]);
          Serial.print("  ");
        }
        Serial.println();
  Serial.print("*********************************************************************************************");
  Serial.println();
  delay(1000);
  
}


void printSensorValues(void)
{
 // 0 means WHITE & 1000 means BLACK
  for (unsigned char i = 0; i < 6; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(currentPosition); // comment if using raw values
  
  }

  void PID(void){

    newError   = currentPosition - center  ;
    derivative = newError - previous_error ;
    integral   = integral + newError       ;
    
    //PID TIME
    motorCorrection = newError / KP + integral / KI + derivative * KD ;
    Serial.println(motorCorrection);
    
    previous_error = newError;
 }

 void runJohnsonMotors(void){

    //set high and low values of correction
    
     if(motorCorrection > maxSpeedValue)
        motorCorrection = maxSpeedValue;
     if(motorCorrection < - maxSpeedValue)
        motorCorrection = - maxSpeedValue;

   // GIVE CORRECTION TO LEFT OR RIGHT MOTORS 

    int leftMotorSpeed  = maxSpeedValue ;
    int rightMotorSpeed = maxSpeedValue - motorCorrection;

    if(motorCorrection < 0)
    {
        leftMotorSpeed  = maxSpeedValue + motorCorrection;
        rightMotorSpeed = maxSpeedValue;
    }

 // if difference is too much - decrease using a factor to a decent level
 
    if(leftMotorSpeed - rightMotorSpeed > max_diffrence)
    {
        leftMotorSpeed -= (leftMotorSpeed - rightMotorSpeed)/factor_diffrence;
    } 
    else if(rightMotorSpeed - leftMotorSpeed > max_diffrence)
    {
        rightMotorSpeed -= (rightMotorSpeed - leftMotorSpeed)/factor_diffrence;
    }


  // give OUTPUT to motors 
    analogWrite(leftMotorPin  , leftMotorSpeed ); 
    analogWrite(rightMotorPin , rightMotorSpeed); 
  }

  void followSimple(void)
  { 
    //loop detect
    if(loopSense) {
 qtra.read(rawValues); 
 if( (rawValues[0]> blackDetectValue) && (rawValues[1] > blackDetectValue) &&(rawValues[4] > blackDetectValue) &&(rawValues[5] > blackDetectValue) )
      { 
        bool notAlign=true;
        while(notAlign){
          if((rawValues[2] < blackDetectValue) && (rawValues[3] < blackDetectValue))
          {//move right
              
         analogWrite(leftMotorPin  , 40 ); 
         analogWrite(rightMotorPin , 0); 

            
          }
          else notAlign=false; //exit
        }
      }

    }
   newError  = currentPosition - center;
   int  leftMotorSpeed  =  maxSpeedValue;
   int rightMotorSpeed =  maxSpeedValue;

    if (newError < -(center/2)) // the line is on the left
    leftMotorSpeed = 0;  // turn left
    if (newError > (center/2))  // the line is on the right
    rightMotorSpeed = 0;  // turn right

    analogWrite(leftMotorPin  , leftMotorSpeed ); 
    analogWrite(rightMotorPin , rightMotorSpeed); 
  }

