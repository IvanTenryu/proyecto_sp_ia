#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo
 #include <AFMotor.h>
#define Echo  A1 
#define Trig  A0
#define IN11 1
#define IN22 2
#define IN33 3
#define IN44 4

AF_DCMotor IN1(IN11, MOTOR12_1KHZ); 
AF_DCMotor IN2(IN22, MOTOR12_1KHZ);
AF_DCMotor IN3(IN33, MOTOR34_1KHZ);
AF_DCMotor IN4(IN44, MOTOR34_1KHZ);
/******************************************************************
   Network Configuration
 ******************************************************************/
const int InputNodes = 3; // includes BIAS neuron
const int HiddenNodes = 4; //includes BIAS neuron
const int OutputNodes = 4;
int i, j;
double Accum;
double Hidden[HiddenNodes];
double Output[OutputNodes];
float HiddenWeights[3][4] = {{-0.5617644549564086, 1.0942893946497538, -1.0682572021248855, 1.304429946291011}, {0.8068076878837755, -4.010636506524299, 4.023499996501605, -0.3342896095417134}, {0.2122339991590681, -1.5699936019785983, -1.6506637473608512, 0.22538199584563273}};
float OutputWeights[4][4] = {{-1.371854399267592, -0.8997542122405253, -0.8681163843245009, 0.025905325213148457}, {-0.9808612938288105, -0.33757346882199263, -1.7677793863252733, 1.8239271557047205}, {-1.4958911354789663, 1.6436091911381543, 0.23187132309438943, 0.6119086932832044}, {1.0923727805142454, 1.2928498659973917, 1.3430060270009385, 1.5636986805649136}};
/******************************************************************
   End Network Configuration
 ******************************************************************/
 
void stop() {
  IN1.run(RELEASE);      
  IN2.run(RELEASE);
  IN3.run(RELEASE); 
  IN4.run(RELEASE);
  Serial.println("Stop!");
} 

//Measure distance in Centimeters
int Distance_test() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  float Fdistance = pulseIn(Echo, HIGH);
  Fdistance= Fdistance / 58;
  Serial.print("CM: ");
  Serial.println(Fdistance);
  return (int)Fdistance;
}

void setup() {
  myservo.attach(10);  // attach servo on pin 3 to servo object
  Serial.begin(9600);
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  stop();
  myservo.write(90);  //starting position in the center
  delay(500); 
} 
 
unsigned long previousMillis = 0;   // to measure time cycles
const long interval = 25;           // intervals every x milliseconds
int grados_servo = 90;              // position of the servo that moves the ultrasonic sensor
bool clockwise = true;              // direction of servo rotation
const long ANGULO_MIN = 30; 
const long ANGULO_MAX = 150; 
double ditanciaMaxima = 75.0;       // far distance from which the NN begins to act
int incrementos = 9;                // increments per cycle of servo position
int accionEnCurso = 1;              // number of cycles executing an action
int multiplicador = 1000/interval;  // multiply the number of cycles to give the car time to turn
const int SPEED = 110;              // speed of the car from all 4 wheels at a time
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
 
  /******************************************************************
       HANDLE SERVO TURN
  ******************************************************************/
    if(grados_servo<=ANGULO_MIN || grados_servo>=ANGULO_MAX){
      clockwise=!clockwise; // cambio de sentido
      grados_servo = constrain(grados_servo, ANGULO_MIN, ANGULO_MAX);
    }
    if(clockwise)
      grados_servo=grados_servo+incrementos;
    else
      grados_servo=grados_servo-incrementos;
    
    if(accionEnCurso>0){
      accionEnCurso=accionEnCurso-1;
    }else{
  /******************************************************************
        WE CALL THE DRIVING FUNCTION
  ******************************************************************/
    conducir();      
    }
    myservo.write(grados_servo);    
  }
}
 
//USE THE NEURONAL NETWORK ALREADY TRAINED
void conducir()
{
  double TestInput[] = {0, 0,0};
  double entrada1=0,entrada2=0;
    
  /******************************************************************
    GET SENSOR DISTANCE
  ******************************************************************/
  double distance = double(Distance_test());
  distance= double(constrain(distance, 0.0, ditanciaMaxima));
  entrada1= ((-2.0/ditanciaMaxima)*double(distance))+1.0; //use a linear function to get closeness
  accionEnCurso = ((entrada1 +1) * multiplicador)+1; // If it is very close to the obstacle, it need more time to react
 
  /******************************************************************
    GET DIRECTION ACCORDING TO THE SERVO ANGLE
  ******************************************************************/
  entrada2 = map(grados_servo, ANGULO_MIN, ANGULO_MAX, -100, 100);
  entrada2 = double(constrain(entrada2, -100.00, 100.00));
 
  /******************************************************************
    WE CALL THE FEED FORWARD NETWORK WITH THE TICKETS
  ******************************************************************/
  // Serial.print("Entrada1:");
  // Serial.println(entrada1);
  // Serial.print("Entrada2:");
  // Serial.println(entrada2/100.0);
 
  TestInput[0] = 1.0;//BIAS UNIT
  TestInput[1] = entrada1;
  TestInput[2] = entrada2/100.0;
 
  InputToOutput(TestInput[0], TestInput[1], TestInput[2]); //INPUT to ANN to obtain OUTPUT
 
  int out1 = round(abs(Output[0]));
  int out2 = round(abs(Output[1]));
  int out3 = round(abs(Output[2]));
  int out4 = round(abs(Output[3]));

  Serial.print("Decimales:  ");
  Serial.print(Output[0]);
  Serial.print(", ");
  Serial.print(Output[1]);
  Serial.print(", ");
  Serial.print(Output[2]);
  Serial.print(", ");
  Serial.println(Output[3]);  
  Serial.print("Salidas red: ");
  Serial.print(out1);
  Serial.print("  ,  ");
  Serial.print(out2);
  Serial.print("  ,  ");
  Serial.print(out3);
  Serial.print("  ,  ");
  Serial.println(out4);
 
  /******************************************************************
    POWERING MOTORS WITH THE NETWORK OUTPUT
  ******************************************************************/
  /*
  int carSpeed = SPEED; //forward or backward
  if((out1+out3)==2 || (out2+out4)==2){ // if it is twist, the motors need double power
    carSpeed = SPEED * 2;
  }*/
    IN1.setSpeed(SPEED);
    IN2.setSpeed(SPEED);
    IN3.setSpeed(SPEED);
    IN4.setSpeed(SPEED);
    
  if (out1==1&&out2==0&&out3==0&&out4==1)
  {
    IN1.run(FORWARD);      
    IN2.run(FORWARD);
    IN3.run(FORWARD); 
    IN4.run(FORWARD);
  }
  else if ((out1==0&&out2==1&&out3==1&&out4==0)||(out1==1&&out2==1&&out3==1&&out4==1))
  {
    IN1.run(BACKWARD);      
    IN2.run(BACKWARD);
    IN3.run(BACKWARD); 
    IN4.run(BACKWARD);
  }
  else if((out1==1&&out2==0&&out3==1&&out4==0)||(out1==1&&out2==1&&out3==1&&out4==0)||(out1==1&&out2==0&&out3==1&&out4==1))
  {
    // LEFT
    IN1.run(FORWARD);      
    IN2.run(BACKWARD);
    IN3.run(BACKWARD); 
    IN4.run(FORWARD);
  }
  else if((out1==0&&out2==1&&out3==0&&out4==1)||(out1==0&&out2==1&&out3==1&&out4==1)||(out1==1&&out2==1&&out3==0&&out4==1))
  {
    // RIGHT
    IN1.run(BACKWARD);      
    IN2.run(FORWARD);
    IN3.run(FORWARD); 
    IN4.run(BACKWARD);
  }
}

void InputToOutput(double In1, double In2, double In3)
{
  double TestInput[] = {0, 0,0};
  TestInput[0] = In1;
  TestInput[1] = In2;
  TestInput[2] = In3;
 
  /******************************************************************
    Calculate triggers on hidden layers
  ******************************************************************/
 
  for ( i = 0 ; i < HiddenNodes ; i++ ) {
    Accum = 0;//HiddenWeights[InputNodes][i] ;
    for ( j = 0 ; j < InputNodes ; j++ ) {
      Accum += TestInput[j] * HiddenWeights[j][i] ;
    }
    //Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ; //Sigmoid
    Hidden[i] = tanh(Accum) ; //tanh
  }
 
  /******************************************************************
    Calculate activation and error in the Output layer
  ******************************************************************/
 
  for ( i = 0 ; i < OutputNodes ; i++ ) {
    Accum = 0;//OutputWeights[HiddenNodes][i];
    for ( j = 0 ; j < HiddenNodes ; j++ ) {
        Accum += Hidden[j] * OutputWeights[j][i] ;
    }
    Output[i] = tanh(Accum) ;//tanh
  }
}
