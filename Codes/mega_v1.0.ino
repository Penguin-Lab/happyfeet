#include <SoftwareSerial.h>
#include <PID_v1.h> //Documentação: https://github.com/br3ttb/Arduino-PID-Library  & http://playground.arduino.cc/Code/PIDLibrary
#include <math.h>

//-------- SoftwareSerial ---------
#define rxPin 11                            // D1-------------------tirar
#define txPin 3                             // D2-------------------tirar
SoftwareSerial Serial_ArdEsp(rxPin, txPin);
//---------------------------------


//-------- Robot parameters -------
#define WHEEL_BASE                      197 // mm
#define WHEEL_DIAMETER                  67  // mm
#define TICKS_PER_REV                   237 // ticks/rev
#define WHEEL_RAY       (WHEEL_DIAMETER/2)  // mm
#define NAV_A                     WHEEL_RAY // mm (cte)
//- Battery monitoring parameters -
#define pin_AnalogDivisor               A0  // Leitura bateria
const int CFG_RESISTOR_R1 =          3900;  // Ohm
const int CFG_RESISTOR_R2 =          10000; // Ohm
const int TENSAO_NOMINAL_MOTOR =       6.6; // V
//---------------------------------

//----- Controller parameters -----
#define FPC_CONVERGENCERADIUS         50.0  // Outside circle radius in mm for FPController
#define CFG_MINBATERRY                  5.5 // Minimum Battery Voltage
#define CFG_MAXSPEED                  250.0 // [mm/s]
#define FPC_Kw                          0.3 // Angular Velocity Equation Constant 
//---------------------------------

//------------- Pins --------------
// Motores:
#define motorDireita_IN1  5
#define motorDireita_IN2  6
#define motorEsquerda_IN3 10
#define motorEsquerda_IN4 9

// Encoders (pinos definem sentido de rotacao):
#define motorDireita_C1  20   // C1 = amarelo = 20_ARDUINO
#define motorDireita_C2  21   // C2 = verde = 21_ARDUINO
#define motorEsquerda_C1 18   // C1 = amarelo = 18_ARDUINO
#define motorEsquerda_C2 19   // C2 = verde = 19_ARDUINO

//---------------------------------

//----- Robot variables -----------
// PID motores:
double rPID_Setpoint, rPID_Input, rPID_Output;
double lPID_Setpoint, lPID_Input, lPID_Output;
double Kp = 0.6, Ki = 0.7, Kd = 0.0;
int pid_max_PWM = 255;

// Objects PIDs:
PID PID_right(&rPID_Input, &rPID_Output, &rPID_Setpoint, Kp, Ki, Kd, DIRECT);
PID PID_left(&lPID_Input, &lPID_Output, &lPID_Setpoint, Kp, Ki, Kd, DIRECT);

// Odometry:
// - Time and ticks:
double dt, now; // ms
double last_tick_time = millis(); // ms
double lticks = 0, last_rticks = 0, rticks = 0, last_lticks = 0;
double m_rticks = 0, m_lticks = 0; // Total mean ticks

// - Robot odometry:
double sr, sl, s; // mm
double nav_pos_x = 0.0, nav_pos_y = 0.0; // mm
double nav_heading = 0.0; // rad
double Rspeed = 0.0, Lspeed = 0.0, v_speed = 0.0; // mm/s
double w_speed = 0.0; // rad/s
double Rho;

// - Robot convertions:
double nav_ClipRadians(double rad) {
  rad -= ( ((int32_t)(rad / (2.0f * M_PI))) * 2 * M_PI);
  return rad < 0.00f ? rad + 2.0f * M_PI : rad;
}

double nav_PitoPi(double rad) {
  if (rad > M_PI) {
    return (rad - 2 * M_PI);
  } else return rad;
}

// Velocities:
double new_v, new_w;
//---------------------------------


//----------- Functions -----------
// - Debug:
#define DEBUG false

// - Máquina de estados principal:
int state = 0;

// - Máquina de estados recepção de chars:
int state_char = 0;
int id_msg = 0;
String velocidades = "";
double time_test = millis(); // ms

// - Mqtt:
void get_velocities() {
  char caractere = Serial_ArdEsp.read();  //read character
  if (caractere == '<')
    state_char++;
  if (state_char > 1)
    state_char = 0;
  if (state_char == 1 && caractere != '<' && caractere != '>') {
    if (isDigit(caractere) || (caractere == '.') || (caractere == ';') || (caractere == '-'))
      velocidades.concat(caractere);
    else {
      state++;
      state_char = 0;
      velocidades = "";
    }
  }
  if (caractere == '>') {
    state++;
    state_char = 0;
    velocidades = "<" + velocidades + ">";
    int inic;
    int fim;
    inic = velocidades.indexOf("<");
    fim = velocidades.indexOf(">");
    int sep1 = velocidades.indexOf(';', inic);
    int sep2 = velocidades.indexOf(';', sep1 + 1);
    if (((millis() - time_test) < 100.0) && (sep1 > 0) && (sep2 > 0) ) {
      int id = velocidades.substring(inic + 1, sep1).toInt();
      double v = velocidades.substring(sep1 + 1, sep2).toDouble();
      double w = velocidades.substring(sep2 + 1, fim).toDouble();
#if DEBUG
      String conteudo;
      Serial.println(velocidades);                             //---------------------------------Tirar
      Serial.println(velocidades.substring(sep1 + 1, sep2));   //---------------------------------Tirar
      Serial.println(v);                                       //---------------------------------Tirar
      conteudo = "inic: " + String(inic) +  " sep1: " + String(sep1) + " sep2: " + String(sep2) + "\n";
      conteudo = conteudo + "id: " + String(id) + " v: " + String(v) + " w: " + String(w) + "\n";
      Serial.print(conteudo);                                  //---------------------------------Tirar
      Serial.print("dt: ");                                    //---------------------------------Tirar
      Serial.println(millis() - time_test);                    //---------------------------------Tirar
#endif
      new_v = v;
      new_w = w;
    }
    else {
#if DEBUG
      Serial.println("DESCARTADO");                            //---------------------------------Tirar
#endif
    }
    time_test = millis();
    velocidades = "";
  }

}

void send_odometry(double x, double y, double heading, double battery) {
  //----Send signal to NodeMCU via SoftwareSerial port----
  // "< ID ; X ; Y ; THETA ; BAT >"
  Serial_ArdEsp.print('<');                   //<(start mark)
  Serial_ArdEsp.print(id_msg);
  Serial_ArdEsp.print(';');                   //,(data seperator)
  Serial_ArdEsp.print((float)x, 2);
  Serial_ArdEsp.print(';');                   //,(data seperator)
  Serial_ArdEsp.print((float)y, 2);
  Serial_ArdEsp.print(';');                   //,(data seperator)
  Serial_ArdEsp.print((float)heading, 2);
  Serial_ArdEsp.print(';');                   //,(data seperator)
  Serial_ArdEsp.print((float)battery, 2);
  Serial_ArdEsp.print('>');                   //>(end mark)
  Serial_ArdEsp.println();
  id_msg++;
}


// Bateria:
double calcula_V_Bateria() {
  double tensao_Vo[10], tensao_Vi[10], tensao_Vin = 0;
  for (int i = 0; i < 10; i++) {
    tensao_Vo[i] = (analogRead(pin_AnalogDivisor) * 5.00) / 1023.00;
    tensao_Vi[i] = (tensao_Vo[i] * (CFG_RESISTOR_R1 + CFG_RESISTOR_R2)) / CFG_RESISTOR_R2; // Divisor de tensao ao contrario
    tensao_Vin += tensao_Vi[i];
  }
  tensao_Vin = (tensao_Vin / 10.0) + 0.7;

  return tensao_Vin - 0.15; // 0.15: margem de erro
}

// PIDs:
void set_PID_Intervals() { //Altera intervalo de saida PID:
  //pid_max_PWM = (255*(TENSAO_NOMINAL_MOTOR + 0.7))/calcula_V_Bateria(0); //+0.7->queda de tensao diodos
  PID_right.SetOutputLimits(-pid_max_PWM, pid_max_PWM);
  PID_left.SetOutputLimits(-pid_max_PWM, pid_max_PWM);
}

void computePID() {
  PID_right.Compute();
  PID_left.Compute();
}

// Controllers:
double FPC_Saturator(double Ro) {
  return 1 - (2 / (1 + pow(M_E, 2 * Ro)));
}

void driveRobot(double v, double w) {
  rPID_Setpoint = (v + (w * WHEEL_BASE / 2));
  lPID_Setpoint = (v - (w * WHEEL_BASE / 2));
}

void driveRobot(double v, double w, double Ro) {
  //Saturation:
  if (Ro < FPC_CONVERGENCERADIUS) {
    rPID_Setpoint = 0;
    lPID_Setpoint = 0;
  }
  else {
    //Inverse calculation of RightSpeed and LeftSpeed:
    rPID_Setpoint = (v + (w * WHEEL_BASE / 2));
    lPID_Setpoint = (v - (w * WHEEL_BASE / 2));
  }
}

#define TORQUE_V 50
void motorHandler(double rOut, double lOut) {     // Must be called constantly
  //Set motors new values:
  if ((rOut < TORQUE_V) && (rOut > -TORQUE_V)) {  // Como o motor não tem torque para
    analogWrite(motorDireita_IN1, LOW);           // velocidades abaixo de 50,
    analogWrite(motorDireita_IN2, LOW);           // é melhor que fique desligado.
  }
  else if (rOut >= TORQUE_V) {
    analogWrite(motorDireita_IN1, rOut);
    analogWrite(motorDireita_IN2, LOW);
  }
  else {
    analogWrite(motorDireita_IN1, LOW);
    analogWrite(motorDireita_IN2,  abs(rOut));
  }
  if ((lOut < TORQUE_V) && (lOut > -TORQUE_V)) {
    analogWrite(motorEsquerda_IN3, LOW);
    analogWrite(motorEsquerda_IN4, LOW);
  }
  else if (lOut >= TORQUE_V) {
    analogWrite(motorEsquerda_IN3, lOut);
    analogWrite(motorEsquerda_IN4, LOW);
  }
  else {
    analogWrite(motorEsquerda_IN3, LOW);
    analogWrite(motorEsquerda_IN4, abs(lOut));
  }
}


// - Encoders:
volatile long int encoder_pos_Dir = 0;
volatile long int encoder_pos_Esq = 0;

void encoder_D() {
  if (digitalRead(motorDireita_C2) == HIGH) {
    encoder_pos_Dir++;
  } else {
    encoder_pos_Dir--;
  }
}

void encoder_E() {
  if (digitalRead(motorEsquerda_C2) == LOW) {
    encoder_pos_Esq++;
  } else {
    encoder_pos_Esq--;
  }
}

// - Odometria:
bool done_odo = false;
bool odometria() {
  done_odo = false;
  // Variacao de ticks (atual - anterior):
  rticks = encoder_pos_Dir - last_rticks;
  lticks = encoder_pos_Esq - last_lticks;

  // Update Navigator parameters:
  now = millis();
  dt += now - last_tick_time;
  m_rticks += rticks;
  m_lticks += lticks;
  last_tick_time = now;

  if (dt >= 10) {
    // Distancia andada por cada roda e pelo robo:
    sr = m_rticks * (WHEEL_DIAMETER * M_PI / TICKS_PER_REV); // s = mtick*2pi*r/tpr
    sl = m_lticks * (WHEEL_DIAMETER * M_PI / TICKS_PER_REV);
    s = (sr + sl) / 2.0;
    // Angulo girado pelo robo:
    double theta = (sr - sl) / WHEEL_BASE;
    // Atualizacao da velocidade:
    v_speed = (s * 1000.0f) / dt; // s/(dt/1000), s = mm, dt = ms, v_speed = mm/s
    Rspeed = (sr * 1000.0f) / dt;
    Lspeed = (sl * 1000.0f) / dt;
    w_speed = (theta * 1000.0f) / dt;
    // Atualizar pose:
    nav_heading = nav_ClipRadians(nav_heading + theta);
    nav_heading = nav_PitoPi(nav_heading);
    nav_pos_x += s * cos(nav_heading);
    nav_pos_y += s * sin(nav_heading);
    // Zera as variaveis da media:
    dt = 0.0;
    m_rticks = 0.0;
    m_lticks = 0.0;
    done_odo = true;
  }

  // Update last encoders counters:
  last_rticks = encoder_pos_Dir;
  last_lticks = encoder_pos_Esq;
  //------------------------------------------------

  //Update PID inputs:
  rPID_Input = Rspeed;
  lPID_Input = Lspeed;
  //-------------------------------------------------
  return done_odo;
  // Updated data for FPC:
  // nav_pos_x  // Parcial position X
  // nav_pos_y  // Parcial position Y
  // nav_heading  // Angle between robot and the north
  // v_speed  // Linear velocity
  // w_speed  // Angular velocity
}

//---------------------------------


void setup() {
  // Serial communication with PC
  Serial.begin(115200);
  // Serial communication with ESP
  Serial_ArdEsp.begin(115200);

  // Encoders:
  attachInterrupt(digitalPinToInterrupt(motorDireita_C1), encoder_D, RISING);
  attachInterrupt(digitalPinToInterrupt(motorEsquerda_C1), encoder_E, RISING);

  // Motores:
  pinMode(motorDireita_IN1, OUTPUT);
  pinMode(motorDireita_IN2, OUTPUT);
  pinMode(motorEsquerda_IN3, OUTPUT);
  pinMode(motorEsquerda_IN4, OUTPUT);

  // PIDs:
  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(100);
  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(100);
  set_PID_Intervals();
  // Bateria:
  Serial.println("\n##### Iniciando leitura de bateria ########");
  Serial.print("                     ");
  Serial.print(calcula_V_Bateria()); Serial.println("V");
  Serial.println("###########################################");
}
void loop() {
  bool check_odo = odometria();
  double bat_lvl = calcula_V_Bateria();
  if (state == 0) {
    // Estimate odometry and battery voltage:
    send_odometry(nav_pos_x, nav_pos_y, nav_heading, bat_lvl);
    state++;
  }
  else if (state == 1) {
    if (Serial_ArdEsp.available()) {
      get_velocities();
      
      // Update PID setpoints:
      driveRobot(new_v, new_w);
    }
    else {
      state++;
    }
  }
  else {
    computePID();
    if (bat_lvl > CFG_MINBATERRY) {
      motorHandler(rPID_Output, lPID_Output);
    }
    else {
      Serial.print("Nível de bateria crítico: ");
      Serial.print(bat_lvl);
      Serial.println("V!!");
      motorHandler(0.0, 0.0);
    }
    state = 0;
  }
}
