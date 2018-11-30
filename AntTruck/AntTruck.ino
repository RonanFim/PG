#include <Servo.h>
#include <PID_v1.h> //Documentação: https://github.com/br3ttb/Arduino-PID-Library  & http://playground.arduino.cc/Code/PIDLibrary
#include <Thread.h> //Documentação: https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h> //Documentação: https://github.com/ivanseidel/ArduinoThread
#define ENCODER_USE_INTERRUPTS//Precisa ser definido antes de Encoder.h
#include <Encoder.h> //Documentação: https://www.pjrc.com/teensy/td_libs_Encoder.html#optimize
#include <Navigator.h> //Documentação: https://github.com/solderspot/NavBot/blob/master/NavBot_v1/docs/Navigator.md
#include <PulseInZero.h>  //Documentação: https://gist.github.com/mikedotalmond/6044960
#include <PulseInOne.h> //Documentação: https://github.com/mikedotalmond/arduino-pulseInWithoutDelay
#include <math.h>
// #include "Serialize.h"

#include <pb_encode.h>
#include <pb_decode.h>
#include "Robot.pb.h"

//----------------------------------------------------------------------------//
//**************************** CONFIGURATIONS ********************************//
//----------------------------------------------------------------------------//
  // INTELLIGENT SPACE ATSP
  #define CFG_IS_ATSP             1

  #define CFG_MINBATERRY          9.5 // Minimum Battery Voltage
  #define CFG_MAXSPEED            250.0 // [mm/s]
  #define CFG_MAXTURNRATE         150.0 // [theta/s]
  #define CFG_FORCEFIELD_RADIUS   250 // Minimum US distance [mm]

  // Battery Monitoring
  #define CFG_RESISTOR_R1       18790
  #define CFG_RESISTOR_R2       12830

  // Navigator defines
  #define WHEEL_BASE      nvMM(191)
  #define WHEEL_DIAMETER  nvMM(60)
  #define TICKS_PER_REV   3591.84

  //Final Position Controller Defines
  #define FPC_CONVERGENCERADIUS 30.0  // Outside circle radius in mm for FPController
  #define FPC_MAXRo   50.0 // Distance in mm when the FPC starts to decrease speed
  #define FPC_Kw      0.3 // Angular Velocity Equation Constant


  //Debugs:
  #define DEBUG_SETPIDINTERVAL  0     // PWM max
  #define DEBUG_ESP_FEEDBACK    1     // Usage of "Serial.print"

  //Testes:
  #define TST_MOTORS      0 // Nominal voltage test
  #define TST_COLLECTDATA   0 // Used to collect motors data
  #define TST_ESPsendData   0 // Test sending variables to ESP

  const int TENSAO_NOMINAL_MOTOR = 6; // Motors nominal power suply
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************** OBJECTS DECLARATION *****************************//
//----------------------------------------------------------------------------//
  /* Variables that had to be declared at the top of the code */

  double velLin = 0;
  double velAng = 0;

  /**** FPC vars *****/
  Navigator navigator;
  nvPose nav_Pose;
  double nav_pos_x, nav_pos_y, nav_heading;
  double nav_velLinear, nav_velAngular;
  double Ro, alpha, betha, gamma;  // angles for navigation kinematics
  /*******************/

  //Support vars for RUNNING state:
  double RUNNING_x, RUNNING_y;
  // TANGENTIAL ESCAPE
  double virtual_x, virtual_y;
  //------------------------------

  // US Distances:
    uint16_t dist_US_Frente=0, dist_US_Direito=0, dist_US_Esquerdo=0;
  //--------------------------------------------------------------

  //Threads:
  ThreadController TC_SERIAL;
  Thread T_Serial;
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************* PINS, INPUTS, OUTPUTS ****************************//
//----------------------------------------------------------------------------//
  //PINs motores:
  // May be inverted in case of wrong direction
  #define motorDireita_IN1  6
  #define motorDireita_IN2  5
  #define motorEsquerda_IN3 9
  #define motorEsquerda_IN4 10
  //---------------------------------

  //PINs ultrassom:
  #define US_Frente_TrigPin 39
  //----------------------------------------------
  #define US_Direito_TrigPin 49
  //----------------------------------------------
  #define US_Esquerdo_TrigPin 29
  //--------US Interrupt
  const int US_EchoPin = 2; //pino de interrupção dos US
  //----// Pins US Servo:
  #define US_servo_TrigPin 4
  #define pin_Servo 8
  const int US_servo_EchoPin = 3; //pino de interrupção dos US servo
////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//*********************** PID/FINAL POSITION CONTROLLER ************************
//----------------------------------------------------------------------------//

  // PID Variables:
  double  rPID_Setpoint,rPID_Input,rPID_Output,
  lPID_Setpoint,lPID_Input,lPID_Output;
   double  Kp= 0.55, Ki = 0.35, Kd=0.0;
  // double  Kp= 0.55, Ki = 0.006, Kd=0.0; // Constants for FPC
  //---------------------------------------------//
  double pid_max_PWM = 150;
  //---------------------------------------------//

  //Objetos PIDs:
  PID PID_right(&rPID_Input, &rPID_Output, &rPID_Setpoint, Kp, Ki, Kd, DIRECT);
  PID PID_left(&lPID_Input, &lPID_Output, &lPID_Setpoint, Kp, Ki, Kd, DIRECT);


  //Methods:
  void set_PID_Intervals(){ //Altera intervalo de saida PID:
    pid_max_PWM = (255*(TENSAO_NOMINAL_MOTOR + 0.7))/calcula_V_Bateria(0);
                                             //+0.7->queda de tensao diodos

    PID_right.SetOutputLimits(-pid_max_PWM, pid_max_PWM);
    PID_left.SetOutputLimits(-pid_max_PWM, pid_max_PWM);

  }

  /**  Speed Saturation Function **
   ** Returns a Speed Scale for the Speed saturator
   *  Replaces tanh(Ro) **/
   double FPC_Saturator(double Ro){
    if (Ro > FPC_MAXRo){
      return 1;
    }
    else{
      return (Ro/FPC_MAXRo);
    }
  }
  //------------------------------------------------------------

  //Final Position Controller (FPC):
  // void FPController(double fpos_x, double fpos_y){ //need to be called in every 'RUNNING' loop
  //   //Vars
  //   double fv, fw;

  //   //Calc Ro and alpha:
  //   double delta_y = fpos_y - nav_pos_y;
  //   double delta_x = fpos_x - nav_pos_x;
  //   Ro = sqrt( delta_y*delta_y + delta_x*delta_x );
  //     //x-axe forward , y-axe to the left:
  //     alpha = ( atan2(delta_y,delta_x) - nav_heading );

  //   //Calc v and w:
  //     fv = CFG_MAXSPEED*FPC_Saturator(Ro)*cos(alpha); // CFG_MAXSPEED*tanh(Ro)*cos(alpha)
  //     fw = FPC_Kw*alpha + fv*sin(alpha)/Ro; // fw = Kw*alpha + CFG_MAXSPEED*(tanh(Ro)/Ro)*sin(alpha)*cos(alpha)

  //     //Update new setpoints:
  //     driveRobot(fv,fw);
  // }

  // PIDs Calculations:
  void computePID(){
    PID_right.Compute();
    PID_left.Compute();
  }
  //-------------------------------------------------------------

  //Thread para alterar os intervalos:
  Thread T_set_PID_Intervals;

  //Thread Controller:
  ThreadController TC_BATERIA;
////////////////////////////////////////////////////////////////////////////////

//----------------------------------------------------------------------------//
//************************ TANGENCIAL ESCAPE CONTROLLER ************************
//----------------------------------------------------------------------------//
  // Calc virtual target for Tangecial Escape Controler
  void calcVirtualTarget(){
    int8_t sign;

    //Look for minimum distance:
      if( (dist_US_Frente<dist_US_Direito) && (dist_US_Frente<dist_US_Esquerdo)){
        betha = 0;
        sign = 1;
      }
      else if( (dist_US_Direito<dist_US_Frente) && (dist_US_Direito<dist_US_Esquerdo) ){
        betha = -(M_PI/4);
        sign = -1;
      }
      else if( (dist_US_Esquerdo<dist_US_Frente) && (dist_US_Esquerdo<dist_US_Direito) ){
        betha = M_PI/4;
        sign = 1;
      }


    //Calc rotation angle
    // gamma = (sign*(abs(betha)-M_PI/2)) - alpha; //FRODO
    gamma = (sign*(M_PI/2)) - (betha - alpha); // FABRICIO

    //Coordinates transformation
    // ->  Robot referential
    double theta = nav_heading; // Readable equations
    double robot_Xd = (cos(theta)*RUNNING_x) + (sin(theta)*RUNNING_y) - (cos(theta)*nav_pos_x)-(sin(theta)*nav_pos_y);
    double robot_Yd = -(sin(theta)*RUNNING_x) + (cos(theta)*RUNNING_y) + (sin(theta)*nav_pos_x)-(cos(theta)*nav_pos_y);

    // Calc Robot Virtual Destination
    // Rotate by the angle 'gamma' calculated:
    double robot_VirX = (cos(gamma)*robot_Xd) + (sin(gamma)*robot_Yd);
    double robot_VirY = -(sin(gamma)*robot_Xd) + (cos(gamma)*robot_Yd);

    //Calc Virtual Target:
    // Robot Reference -> World Reference
    virtual_x = cos(theta)*robot_VirX - sin(theta)*robot_VirY + nav_pos_x;
    virtual_y = sin(theta)*robot_VirX + cos(theta)*robot_VirY + nav_pos_y;
  }


////////////////////////////////////////////////////////////////////////////////


//----------------------------------------------------------------------------//
//************************ QUADRATURE ENCODER  *******************************//
//----------------------------------------------------------------------------//
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder enc_Direita(20, 21);
  Encoder enc_Esquerda(19, 18);
  //   avoid using pins with LEDs attached

  ///////Variáveis de posição:
  double encDIR_Position  = 0, encDIR_lastPosition = 0, encDIR_lastTIME=0;
  double encESQ_Position  = 0, encESQ_lastPosition = 0, encESQ_lastTIME=0;
  /////////////////////////////////////////////////////////////////////////

  double calcVel_Direita(bool _debug){
    double _vel, tempoAtual;
    tempoAtual = millis();

  encDIR_Position = enc_Direita.read(); //leitura posição atual

  _vel = ((encDIR_Position - encDIR_lastPosition)/(tempoAtual - encDIR_lastTIME) )* 52.491105; // v = deltaS/deltaT
  encDIR_lastTIME = tempoAtual;
  encDIR_lastPosition = encDIR_Position;

  if(_debug){
    Serial.println("**** Debug Calculo Velocidade Direito *****");
    Serial.print("Tempo entre uma leitura e outra:  ");
    Serial.println(tempoAtual - encDIR_lastTIME);
    Serial.print("encDIR_lastPosition:  ");
    Serial.println(encDIR_lastPosition);
    Serial.print("Velocidade Direito:  ");
    Serial.println(_vel);
    Serial.println("*********************************************");
    Serial.println(" ");
  }

  return _vel;
  }

  double calcVel_Esquerda(bool _debug){
    double _vel, tempoAtual;
    tempoAtual = millis();

  encESQ_Position = enc_Esquerda.read(); //leitura posição atual

  _vel = ((encESQ_Position - encESQ_lastPosition)/(tempoAtual - encESQ_lastTIME) )* 52.491105; // v = deltaS/deltaT
  encESQ_lastTIME = tempoAtual;
  encESQ_lastPosition = encESQ_Position;

  if(_debug){
    Serial.println("**** Debug Calculo Velocidade Esquerdo *****");
    Serial.print("Tempo entre uma leitura e outra:  ");
    Serial.println(tempoAtual - encESQ_lastTIME);
    Serial.print("encESQ_lastPosition:  ");
    Serial.println(encESQ_lastPosition);
    Serial.print("Velocidade Esquerdo:  ");
    Serial.println(_vel);
    Serial.println("*********************************************");
    Serial.println(" ");
  }

  return _vel;
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
************************************ ODOMETRY **********************************
*******************************************************************************/

  double lticks=0, last_rticks=0, rticks=0, last_lticks=0;
  double t_rticks, t_lticks; //Total ticks


  void odometria(bool debug){
    // Current encoders counters:
    t_rticks = enc_Direita.read();
    t_lticks = enc_Esquerda.read();

    //Counter differencial:
    rticks = t_rticks - last_rticks;
    lticks = t_lticks - last_lticks;

    // Update Navigator parameters:
    navigator.UpdateTicks( lticks, rticks, millis() );

    // Update last encoders counters:
    last_rticks = t_rticks;
    last_lticks = t_lticks;
    //------------------------------------------------

    //Update PID inputs:
    rPID_Input = navigator.RightSpeed();
    lPID_Input = navigator.LeftSpeed();
    //-------------------------------------------------

    // Update Atual Position for the FPC:
    nav_Pose = navigator.Pose();
    nav_pos_x = nav_Pose.position.x;
    nav_pos_y = nav_Pose.position.y;
    nav_heading = nav_Pose.heading;
    nav_velLinear = navigator.Speed(); //Debug only
    nav_velAngular = navigator.TurnRate(); //Debug only
    //-------------------------------------------

    if(debug){
      Serial.print("enc_Direita.read(): " );
      Serial.print(enc_Direita.read());
      Serial.print("\trticks: ");
      Serial.print(rticks);
      Serial.print("\tenc_Esquerda.read(): " );
      Serial.print(enc_Esquerda.read());
      Serial.print("\tlticks: " );
      Serial.println(lticks);
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
******************************** MOVEMENT FUNCTIONS ****************************
*******************************************************************************/

  bool parado = 0;

  void driveRobot(double v, double w){
    //Saturation:
    // v = (v>CFG_MAXSPEED) ? CFG_MAXSPEED : v;
    // w = (w>CFG_MAXTURNRATE) ? CFG_MAXTURNRATE : w;

    if(v==0 && w==0)
    {
      stopAll();
      parado = 1;
    }
    else
    {
      //Inverse calculation of RifhtSpeed and LeftSpeed:
      rPID_Setpoint = (v + (w*WHEEL_BASE/2));
      lPID_Setpoint = (v - (w*WHEEL_BASE/2));
      parado = 0;
    }
  }

  void motorHandler(){ //Must be called constantly

    //Set motors new values:
    if(parado)
    {
      stopAll();
      parado = 0;
    }
    else if( (rPID_Output>=0) && (lPID_Output>=0) ){
      analogWrite(motorDireita_IN1, LOW);
      analogWrite(motorDireita_IN2, rPID_Output);
      analogWrite(motorEsquerda_IN3, lPID_Output);
      analogWrite(motorEsquerda_IN4, LOW);
    }
    else if( (rPID_Output<=0) && (lPID_Output>=0) ){
      analogWrite(motorDireita_IN1, abs(rPID_Output));
      analogWrite(motorDireita_IN2, LOW);
      analogWrite(motorEsquerda_IN3, lPID_Output);
      analogWrite(motorEsquerda_IN4, LOW);
    }
    else if( (rPID_Output>=0) && (lPID_Output<=0) ){
      analogWrite(motorDireita_IN1, LOW);
      analogWrite(motorDireita_IN2, rPID_Output);
      analogWrite(motorEsquerda_IN3, LOW);
      analogWrite(motorEsquerda_IN4, abs(lPID_Output));
    }
    else if( (rPID_Output<=0) && (lPID_Output<=0) ){
      analogWrite(motorDireita_IN1, abs(rPID_Output));
      analogWrite(motorDireita_IN2, LOW);
      analogWrite(motorEsquerda_IN3, LOW);
      analogWrite(motorEsquerda_IN4, abs(lPID_Output));
    }
  }

  void stopAll(){
    // Serial.println("Parou!");
    analogWrite(motorDireita_IN1, LOW);
    analogWrite(motorDireita_IN2, LOW);
    analogWrite(motorEsquerda_IN3, LOW);
    analogWrite(motorEsquerda_IN4, LOW);
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
*************************** ULTRASOUNDS VARS AND FUNCTIONS *********************
*******************************************************************************/

  //Constantes de Cálculos:
  const float SpeedOfSound      = 343.2; // ~speed of sound (m/s) in air, at 20°C
  const float MicrosecondsPerMillimetre   = 1000.0 / SpeedOfSound; // microseconds per millimetre - sound travels 1 mm in ~2.9us
  const float  MicrosecondsToMillimetres  = (1.0 / MicrosecondsPerMillimetre);
  const float  MicrosecondsToMillimetres2 = MicrosecondsToMillimetres / 2.0; // beam travels the distance twice... so halve the time.
  //--------------------------------------------------------------------------

  //Variáveis de controle:
  uint8_t _swch = 1;
  boolean _done = true;
  //----------------------------------------------------------------------------

  //Threads:
  Thread T_USinterrupt_Read;
  //-------------------
  ThreadController TC_US;
  //-------------------------

  void us_Read() {
    if (_done){
      uint8_t trig_pin;

      switch(_swch){
        case 1:
        trig_pin = US_Frente_TrigPin;
        break;
        case 2:
        trig_pin = US_Direito_TrigPin;
        break;
        case 3:
        trig_pin = US_Esquerdo_TrigPin;
        break;
        default:
        _swch=1;
        break;
      }

      //seta o pino TRIG com pulso alto "HIGH"
      digitalWrite(trig_pin, HIGH);
      //delay de 10 microssegundos
      delayMicroseconds(11);
      //seta o pino TRIG com pulso baixo novamente
      digitalWrite(trig_pin, LOW);

      //start listening out for the echo pulse on interrupt 0 (pino 2):
      PulseInZero::begin();

      _done = false;
    }
  }

  void us_PulseComplete(unsigned long duration){
    switch(_swch){
      case 1:
      dist_US_Frente = duration * MicrosecondsToMillimetres2;
      _swch = 2;
      break;
      case 2:
      dist_US_Direito = duration * MicrosecondsToMillimetres2;
      _swch = 3;
      break;
      case 3:
      dist_US_Esquerdo = duration * MicrosecondsToMillimetres2;
      _swch = 1;
      break;
    }
    _done = true;
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
********************************* SERVO ULTRASOUND *****************************
*******************************************************************************/
  //Servo
  Servo servoUS;

  //Threads:
  Thread T_USi_Servo_Read;

  //Variaveis:
  uint16_t dist_US_servo=0;
  //-----------------------------------------------------------

  // Funções de leitura do US:

  //Read:
  void us_servoRead() {
  //seta o pino TRIG com pulso alto "HIGH"
  digitalWrite(US_servo_TrigPin, HIGH);
  //delay de 10 microssegundos
  delayMicroseconds(11);
  //seta o pino TRIG com pulso baixo novamente
  digitalWrite(US_servo_TrigPin, LOW);

  //start listening out for the echo pulse on interrupt 1 (pino 3):
  PulseInOne::begin();
  }
  //----------------------------------------------

  //Gatilho da interrupção:
  void us_servo_PulseComplete(unsigned long duration){
    dist_US_servo = MicrosecondsToMillimetres2 * duration;
  }
  //

  // Funções de Movimento - Servo:
  void servo_varrer(){
    byte i;

    if (i < 90){
      for(i; i>=180; i++){
        servoUS.write(i);
        delay(50);
      }
    }
    else{
      for(i; i<=0; i--){
        servoUS.write(i);
        delay(50);
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
********************************* BATTERY MONITORING ***************************
*******************************************************************************/

  //Valor dos resistores (Divisor de tensão):
  const int resistor_R1 = CFG_RESISTOR_R1, resistor_R2 = CFG_RESISTOR_R2;
  const int pin_AnalogDivisor = A0;
  /////////////////////////////////////////////

  //Encontrar a tensão da bateria:
  double calcula_V_Bateria(bool _debug){
    double tensao_Vo[10], tensao_Vi[10], tensao_Vin=0;

    for(int i=0; i<10; i++){
      tensao_Vo[i] = (analogRead(pin_AnalogDivisor)*5.00)/1023.00;
    tensao_Vi[i] = (tensao_Vo[i]*(resistor_R1+resistor_R2))/resistor_R2; //divisor de tensão.
    tensao_Vin += tensao_Vi[i];
    }

    tensao_Vin = (tensao_Vin / 10.0);

    //Debug:
    if(_debug){
      Serial.println("*****Debug calcula_V_Bateria():");
      Serial.println(">>tensao_Vo: ");
      for(int i=0; i<10; i++){
        Serial.println(tensao_Vo[i]);
      }
      Serial.println(">>tensao_Vi: ");
      for(int i=0; i<10; i++){
        Serial.println(tensao_Vi[i]);
      }
      Serial.print(">>tensao_Vin: ");
      Serial.println(tensao_Vin);
      Serial.println(" ");
      Serial.println(" ");
    }

    return tensao_Vin - 0.15; //(0.15 margem de erro)
  }

  Thread T_alertaBateria;
  //Tensão abaixo do limite minimo:
  boolean warn=false;
  void alertaBateria(){
    if( calcula_V_Bateria(0) > CFG_MINBATERRY ){
      warn=false;
      }else{
        warn=true;
      }
  //Desabilita Robô caso tensão baixa:
    while (warn){ //loop infinito
      //changeState("stopAll");
      driveRobot(0,0);
      stopAll();

      servoUS.write(90);
      delay(200);
      servoUS.write(180);
      delay(800);
      servoUS.write(0);
      delay(800);
      servoUS.write(90);

      //Serial.println("=======> BATERIA FRACA!! <=========");
      delay(3000);
    }
  }
////////////////////////////////////////////////////////////////////////////////










Odometry pckout = Odometry_init_zero;
PIDs pckin = PIDs_init_zero;

char tx_buffer[128]; //128
char rx_buffer[128];
uint8_t msg_length;

uint8_t nova_leitura = 0;

void build_package()
{
  /*pckout.pos_x = 12500;
  pckout.pos_y = 13620;
  pckout.heading = 4500;
  pckout.dist_US_Frente = 250;
  pckout.dist_US_Direito = 100;
  pckout.dist_US_Esquerdo = 70;*/
  pckout.pos_x = (int32_t)(nav_pos_x*1000);
  pckout.pos_y = (int32_t)(nav_pos_y*1000);
  pckout.heading = (int32_t)(nav_heading*1000);
  pckout.dist_US_Frente = dist_US_Frente;
  pckout.dist_US_Direito = dist_US_Direito;
  pckout.dist_US_Esquerdo = dist_US_Esquerdo;
  pckout.vel_linear = (int32_t)(nav_velLinear*1000);
  pckout.vel_angular = (int32_t)(nav_velAngular*1000);
}

void unpack()
{
  if (pckin.velLin==0 && pckin.velAng==1)
  {
    velLin = 0;
    velAng = 0;
  }
  else
  {
    velLin = pckin.velLin/1000.0;
    velAng = pckin.velAng/1000.0;
  }

}




void com_serial()
{

  while(Serial3.available() > 0)
  {
    char ch = Serial3.read();

    if(ch == '#')
    {
      ch = Serial3.read();

      if(ch == '#')
      {
        ch = Serial3.read();

        if(ch == '*')
        {
          ch = Serial3.read();
          
          for(int i = 0; i < ch; i++)
          {
            rx_buffer[i] = Serial3.read();
          }
          nova_leitura = 1;
        }
        else if(ch == '#')
        {
          Serial3.write((char)msg_length);
          Serial3.write(tx_buffer, msg_length);
        }
      }
    }
  }
}











void setup() {

  //Inicia Serial:
  Serial.begin(115200);
  Serial3.begin(115200);
  //----------------------

  //Pinos I/O dos motores:
  pinMode(motorDireita_IN1, OUTPUT);
  pinMode(motorDireita_IN2, OUTPUT);
  pinMode(motorEsquerda_IN3, OUTPUT);
  pinMode(motorEsquerda_IN4, OUTPUT);
  //--------------------------------------

  //Pinos Ultrassom (Interrupt):
  pinMode(US_Frente_TrigPin, OUTPUT);
  digitalWrite(US_Frente_TrigPin, LOW);
  pinMode(US_Direito_TrigPin, OUTPUT);
  digitalWrite(US_Direito_TrigPin, LOW);
  pinMode(US_Esquerdo_TrigPin, OUTPUT);
  digitalWrite(US_Esquerdo_TrigPin, LOW);
  pinMode(US_servo_TrigPin, OUTPUT);
  digitalWrite(US_servo_TrigPin, LOW);
  //---//
  pinMode(US_EchoPin,INPUT);
  pinMode(US_servo_EchoPin,INPUT);
  //---//
  PulseInOne::setup(us_servo_PulseComplete);
  PulseInZero::setup(us_PulseComplete);
  //----------------------------------------------------


  //Servo:
  servoUS.attach(pin_Servo);
  //-----------------------------

  // Threads' setups:
    //Thread Controllers:
    TC_BATERIA.add(&T_set_PID_Intervals);
    TC_BATERIA.add(&T_alertaBateria);
    //-------------------------------------
    TC_US.add(&T_USinterrupt_Read);
    TC_US.add(&T_USi_Servo_Read);
    //-------------------------------------
    TC_SERIAL.add(&T_Serial);

    //Setting time and function to call:
    T_set_PID_Intervals.setInterval(5000); // in milisseconds
    T_set_PID_Intervals.onRun(set_PID_Intervals);

    T_alertaBateria.setInterval(10000); // in milisseconds
    T_alertaBateria.onRun(alertaBateria);
    //--------------------------------------------
    T_USinterrupt_Read.setInterval(20); // in milisseconds
    T_USinterrupt_Read.onRun(us_Read);

    T_USi_Servo_Read.setInterval(50); // in milisseconds
    T_USi_Servo_Read.onRun(us_servoRead);

    T_Serial.setInterval(15); // in milisseconds
    T_Serial.onRun(com_serial);
  /****************************************************/

  //Inicia a Odometria e Navegação:
  // set up navigation
  navigator.InitEncoder( WHEEL_DIAMETER, WHEEL_BASE, TICKS_PER_REV );
  navigator.Reset( millis() );
  //-------------------------------------------------------------------

  //Inicialize PID:
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(100);

  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(100);
  //------------------------------------------------------------------

  //Realiza a primeira leitura da bateria:
  //Serial.println("##### Iniciando leitura de bateria ########");
  set_PID_Intervals();
  //Serial.println(pid_max_PWM);
  //Serial.println("###########################################");
  //-------------------------------------------------------------------
  // driveRobot(2000,2100);
  // computePID(); // Calculates PID's parameters
  // motorHandler();
  // delay(1000);
  // driveRobot(0,0);
  // computePID(); // Calculates PID's parameters
  // motorHandler();
  analogWrite(motorDireita_IN1, LOW);
  analogWrite(motorDireita_IN2, 150);
  analogWrite(motorEsquerda_IN3, 150);
  analogWrite(motorEsquerda_IN4, LOW);
  delay(1000);
  stopAll();

}

void loop() {
  TC_BATERIA.run(); // Controls battery monitoring functions
  TC_US.run(); // Controls ultrasound sensors readings
  TC_SERIAL.run();

  /* Create a stream that will write to our buffer. */
  pb_ostream_t msg_out = pb_ostream_from_buffer(&tx_buffer[0], sizeof(tx_buffer));

  /* Create a stream that reads from the buffer. */
  // Starts on 2nd byte because of the LRC first byte
  //pb_istream_t msg_in = pb_istream_from_buffer(&rx_buffer[1], sizeof(rx_buffer)-1);
  pb_istream_t msg_in = pb_istream_from_buffer(&rx_buffer[0], sizeof(rx_buffer));

  odometria(0); // Calculates position, velocity, orientation, etc

  build_package();
  pb_encode(&msg_out, Odometry_fields, &pckout);
  msg_length = msg_out.bytes_written;

  // Serial3.print('#');
  // Serial3.print('#');
  // Serial3.print('#');
  // Serial3.write((char)msg_length);
  // Serial3.write(tx_buffer, msg_length);

  if(nova_leitura == 1)
  {
    pb_decode(&msg_in, PIDs_fields, &pckin);
    unpack();
    Serial.println(velLin);
    Serial.println(velAng);
    nova_leitura = 0;
  }

  driveRobot(velLin, velAng);
  computePID(); // Calculates PID's parameters
  motorHandler();
  delay(10);

}
