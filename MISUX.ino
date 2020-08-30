//-------------------------------------------------------------------MAGNETOMETRO-----------------------------------------------------
// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
//---------------------------------------------------------------------BATERÍA--------------------------------------------------------
float bateria=0;
int bateria_alto=53;
int bateria_medio=51;
int bateria_bajo=49;
int lectorvolt=A4;
//---------------------------------------------------------------------MOTORES--------------------------------------------------------
int BRAKE=0;
int CW=1;
int CCW=2;
int CS_THRESHOLD=15;   // Definition of safety current (Check: "1.3 Monster Shield Example").
float ajuste=4.5;
//----------------MOTOR 1----------------
int MOTOR_A1_PIN=7;//22;     //PIN DE MOTOR A1   HORARIO 1           DIGITAL
int MOTOR_B1_PIN=8;//23;     //PIN DE MOTOR B1   ANTIHORARIO 1       DIGITAL
//----------------MOTOR 2----------------
int MOTOR_A2_PIN=4;//28;     //PIN DE MOTOR A2   HORARIO 2           DIGITAL
int MOTOR_B2_PIN=9;//29;     //PIN DE MOTOR B2   ANTIHORARIO 2       DIGITAL
int PWM_MOTOR_1=5;//16;      //PIN DE PWM MOTOR 1                    PWM
int PWM_MOTOR_2=6;//17;      //PIN DE PWM MOTOR 2                    PWM
int CURRENT_SEN_1=A2;//A14;   //SENSOR DE CORRIENTE 1                 ANALOG INPUT
int CURRENT_SEN_2=A3;//A15;   //SENSOR DE CORRIENTE 2                 ANALOG INPUT
int EN_PIN_1=A0;//A12;        //ENABLE MOTOR 1                        ANALOG INPUT
int EN_PIN_2=A1;//A13;        //ENABLE MOTOR 2                        ANALOG INPUT
//-----------------------------------------
int MOTOR_1=0;
int MOTOR_2=1;
short usSpeed = 20;                        //Velocidad por default
unsigned short usMotor_Status = BRAKE;      //Condicion inicial
//-------------------------------------------------------------------ULTRASONICO------------------------------------------------------
int eco=2;               //PIN ECO
int trg=3;              // PIN TRG            
float temp=0;                       
float dist = 11;
int obst=30;
int vecesobst=0;
//--------------------------------------------------------------------BLUETOOTH-------------------------------------------------------
char inData[33];
char inChar; 
char *p;
int index = 0; 
int16_t signed_val=0.0;                        
float distancia_centro;
float distancia_derecha;
float distancia_izquierda;
String direccion="E458:B8:D6961C";      // celular:E458:B8:D6961C laptop: D85D:E2:ABE1C8 // celular:5A020C laptop: A010C
String direccion_obtenida;

void setup(){
  Serial.begin(9600);
  //-------------------------------------------------------------------MAGNETOMETRO----------------------------------------------------

  //---------------------------------------------------------------------BATERÍA-------------------------------------------------------
  pinMode(lectorvolt,INPUT);
  pinMode(bateria_alto,OUTPUT);
  pinMode(bateria_medio,OUTPUT);
  pinMode(bateria_bajo,OUTPUT);
  //------------------------------------------------------------EVITADOR DE OBSTÁCULOS-------------------------------------------------
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(CURRENT_SEN_1, OUTPUT);
  pinMode(CURRENT_SEN_2, OUTPUT);  
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  pinMode(eco, INPUT);              
  pinMode(trg, OUTPUT);
  digitalWrite(EN_PIN_1, HIGH); //activar motor 1
  digitalWrite(EN_PIN_2, HIGH); //activar motor 2             
  //------------------------------------------------------------INICIALIZACIÓN BLUETOOTH-----------------------------------------------
  //-------------------------------------------------------Comandos de inicialización SENSOR CENTRO------------------------------------
  Serial1.begin(38400);
  delay(100); 
  Serial1.write("AT\r\n");
  delay(100); 
  Serial1.write("AT+INIT\r\n");
  delay(100); 
  Serial1.write("AT+IAC=9e8b33\r\n");
  delay(100); 
  Serial1.write("AT+CLASS=0\r\n");
  delay(100); 
  Serial1.write("AT+INQM=1,5,5\r\n");
  delay(100);
  //-------------------------------------------------------Comandos de inicialización SENSOR DERECHA------------------------------------
  Serial2.begin(38400);
  delay(100); 
  Serial2.write("AT\r\n");
  delay(100); 
  Serial2.write("AT+INIT\r\n");
  delay(100); 
  Serial2.write("AT+IAC=9e8b33\r\n");
  delay(100); 
  Serial2.write("AT+CLASS=0\r\n");
  delay(100); 
  Serial2.write("AT+INQM=1,5,5\r\n");
  delay(100); 
  //-------------------------------------------------------Comandos de inicialización SENSOR IZQUIERDA-----------------------------------
  Serial3.begin(38400);
  delay(100); 
  Serial3.write("AT\r\n");
  delay(100); 
  Serial3.write("AT+INIT\r\n");
  delay(100); 
  Serial3.write("AT+IAC=9e8b33\r\n");
  delay(100); 
  Serial3.write("AT+CLASS=0\r\n");
  delay(100); 
  Serial3.write("AT+INQM=1,5,5\r\n");
  delay(100);
}

void loop() {
 evitador();
 sensorCentro();
 evitador();
 sensorDerecha();
 evitador();
 sensorIzquierda();
 evitador();
 rastreo();
 evitador();
 battery();
}

void battery(){
  bateria=analogRead(A4);
  bateria=(bateria*1.0*(5/1023.0));
  
  if(bateria>=2.70)
  {
    digitalWrite(bateria_alto, HIGH);
    digitalWrite(bateria_medio, HIGH);
    digitalWrite(bateria_bajo, HIGH);
  }
  if(bateria<=2.69&&bateria>=2.39)
  {
    digitalWrite(bateria_medio, HIGH);
    digitalWrite(bateria_alto, LOW);
    digitalWrite(bateria_bajo, HIGH);
  }
  if(bateria<=2.38)
  {
    digitalWrite(bateria_bajo, HIGH);
    digitalWrite(bateria_medio, LOW);
    digitalWrite(bateria_alto, LOW);
  }
}

void sensorCentro(){
  //Repetir ciclo hasta que direccion_obtenida==direccion
  direccion_obtenida=" ";
  while(direccion_obtenida.indexOf(direccion)==-1)  
  {
    Serial1.write("AT+INQ\r\n");
    index=0;
    while(Serial1.available()==0);
    while(Serial1.available())
      {
        inChar = Serial1.read(); 
        inData[index] = inChar; 
        index++; 
        inData[index] = '\0'; 
        delay(1);
      }
    direccion_obtenida=inData;
  }

  //Recortar array y obtener valor
  p = strtok(&inData[index-6]," "); 
  while(p != NULL)
  {
    signed_val = strtoul( p, NULL, 16 );
    p = strtok(NULL, " ");
    delay(1);
  }
  distancia_centro = pow(10.0,(-63.0-signed_val)/20);

  Serial.print("Sensor centro - ");
  Serial.print(distancia_centro);
  Serial.println(" metros");
}

void sensorDerecha(){
  //Repetir ciclo hasta que direccion_obtenida==direccion
  direccion_obtenida=" ";
  while(direccion_obtenida.indexOf(direccion)==-1)  
  {
    Serial2.write("AT+INQ\r\n");
    index=0;
    while(Serial2.available()==0);
    while(Serial2.available())
      {
        inChar = Serial2.read(); 
        inData[index] = inChar; 
        index++; 
        inData[index] = '\0'; 
        delay(1);
      }
    direccion_obtenida=inData;  
  }

  //Recortar array y obtener valor
  p = strtok(&inData[index-6]," "); 
      while(p != NULL)
      {
        signed_val = strtoul( p, NULL, 16 );
        p = strtok(NULL, " ");
      }
    distancia_derecha = pow(10.0,(-65.0-signed_val)/20);
      
    Serial.print("Sensor derecha - ");
    Serial.print(distancia_derecha);
    Serial.println(" metros");
}

void sensorIzquierda(){
  //Repetir ciclo hasta que direccion_obtenida==direccion
  direccion_obtenida=" ";
  while(direccion_obtenida.indexOf(direccion)==-1)  
  {
    Serial3.write("AT+INQ\r\n");
    index=0;
    while(Serial3.available()==0);
    while(Serial3.available())
      {
        inChar = Serial3.read(); 
        inData[index] = inChar; 
        index++; 
        inData[index] = '\0'; 
        delay(1);
      }
    direccion_obtenida=inData;
  }

  //Recortar array y obtener valor
  p = strtok(&inData[index-6]," "); 
      while(p != NULL)
      {
        signed_val = strtoul( p, NULL, 16 );
        p = strtok(NULL, " ");
      }
  distancia_izquierda = pow(10.0,(-63.0-signed_val)/20);
    
    Serial.print("Sensor izquierda - ");
    Serial.print(distancia_izquierda);
    Serial.println(" metros");
}

void rastreo(){
    if(distancia_centro<=1||distancia_derecha<=1||distancia_izquierda<=1)                  //Si algún sensor marca menos que 1m no hacer nada
  {
    Serial.print("ya encontré mi lugar en el mundo");
  }else{                                                                                //Si los 3 sensores marcan más de 1m empezar el rastreo
    if(distancia_centro<=distancia_derecha&&distancia_centro<=distancia_izquierda)
    {
          //motorGo(MOTOR_1, CW, usSpeed);
          //motorGo(MOTOR_2, CW, usSpeed);
          Serial.println("Recto");                                                                              //Hacer una función para que siga derecho
    }
  
    if(distancia_derecha<=distancia_centro&&distancia_derecha<=distancia_izquierda)
    {
          //motorGo(MOTOR_1, CW, usSpeed);
          //motorGo(MOTOR_2, CCW, usSpeed);
          Serial.println("Derecha");//Hacer una función para que gire 90° a la derecha y camine
    }
  
    if(distancia_izquierda<=distancia_centro&&distancia_centro<=distancia_derecha)
    {
          //motorGo(MOTOR_1, CCW, usSpeed);
          //motorGo(MOTOR_2, CW, usSpeed);
          Serial.println("Izquierda");//Hacer una función para que gire 90° a la izquierda y camine
    } 
  }
}

void medirObst(){
    digitalWrite(trg,HIGH);           //se escribe un valor digital alto para trg
    delayMicroseconds(10);            //retraso de 10 microsegundos
    digitalWrite(trg,LOW);            //se escribe un valor digital bajo para trg
    temp = pulseIn(eco, HIGH);        
    dist = ((temp/1000000*343.2/2)*100);
}
void evitarIzq(){
  medirObst();
  while (dist<=obst)
  {
    usSpeed = usSpeed - 1;
    if (usSpeed<20){
    usSpeed=20;
    }
    motorGo(MOTOR_1, CW, usSpeed);
    motorGo(MOTOR_2, CCW, usSpeed);
    medirObst();
  }
  vecesobst+=1;
}

void evitarDer(){
   medirObst();
   while (dist<=obst)
   {
    usSpeed = usSpeed - 1;
    if (usSpeed <20){
      usSpeed=20;
    }
    motorGo(MOTOR_1, CCW, usSpeed);
    motorGo(MOTOR_2, CW, usSpeed);
    medirObst();   
   }
    vecesobst+=1;
}

void evitarRec(){
   while (usSpeed<30)
   {
     usSpeed = usSpeed + 0.2;
    }
     motorGo(MOTOR_1, CW, usSpeed);
     motorGo(MOTOR_2, CW, usSpeed);    
}

void evitador(){
  if(vecesobst&2==1){
    evitarIzq();
  }else{
    evitarDer();
  }
  evitarRec();
}
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if(motor == MOTOR_1)
  {
    if(direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }    
    analogWrite(PWM_MOTOR_1, pwm-4);            //pwm a ese al motor 1
  }
  else if(motor == MOTOR_2)
  {
    if(direct == CW)                          //CLOCKWISE
    {
      digitalWrite(MOTOR_A2_PIN, LOW);        
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW)                    //ANTIHORARIO
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else                                      //DETENIDO
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    analogWrite(PWM_MOTOR_2, pwm);
  }
}


