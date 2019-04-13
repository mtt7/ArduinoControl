//Program odpowiedzialny za sterowanie robotem,
//i wykonywanie instrukcji odebranych przez ESP8266
#include <AFMotor.h>

char buf[255];
int LeftLED = A0;
int UpLED = A1;
int RightLED = A2;
int DownLED = A3;
boolean bufRDY = false;
boolean followMode = false;
boolean lineMode = false;
boolean controlMode = true;

//Nazwy pinów odpowiedzialnych za sterowanie dwoma czujnikami
//ultradźwiękowymi
const unsigned int TRIG1 = A0;
const unsigned int ECHO1 = A1;
const unsigned int TRIG2 = A2;
const unsigned int ECHO2 = A3;
unsigned int dist_left;
unsigned int dist_right;
volatile int search = 0;

//Timer1 stałe
const uint16_t timer1_load1 = 47628;
const uint16_t timer1_load2 = 37628;

//Ustawienie komunikacji z MotorShield
AF_DCMotor motor_left(2,MOTOR12_1KHZ);
AF_DCMotor motor_right(3,MOTOR12_1KHZ);

unsigned int sensorSonicRead(unsigned int trig,unsigned int echo);
void followObj();
//Funkcja czytająca dane przesłane przez ESP8266
int readSerial(int readchar,char *buffer,int len)
{
  static int pos = 0;
  int rpos;

  if(readchar>0)
  {
    switch(readchar)
    {
      case '\r':
        break;
      case '\n':
        rpos = pos;
        pos = 0;
        return rpos;
      default:
        if(pos<len-1)
        {
          buffer[pos++] = readchar;
          buffer[pos] = 0;    
        }
    }
  }
  return 0;
}


void setup() {
  Serial.begin(115200);

  //Inicjalizacja silników napędzających koła robota
  motor_left.setSpeed(150);
  motor_right.setSpeed(150);
  motor_left.run(RELEASE);
  motor_right.run(RELEASE);

  
  pinMode(TRIG1,OUTPUT);
  pinMode(ECHO1,INPUT);
  pinMode(TRIG2,OUTPUT);
  pinMode(ECHO2,INPUT);

  pinMode(13,OUTPUT);
  //Wyzerowanie rejestru TCCR1A
  TCCR1A = 0;
  //Ustawienie overflow interrupt dla Timer1
  TIMSK1 = (1 << TOIE1);

  //Właczenie global interrupts
  sei();
}

void loop() {
  //Wpis danych z ESP8266 do bufora bez blokowania pętli
  if (readSerial(Serial.read(), buf, 255) > 0)
  {
        //Serial.print("You entered: >");
        //Serial.print(buf);
        //Serial.println("<");
        bufRDY = true;
  }

  //Wybor trybu:
  //Kontrola komórką
  //Podążanie za obiektem
  //Podążanie za linią
  if(bufRDY == true)
  {
    if(strcmp(buf,"CONTROL") == 0)
    {
      controlMode = true;
      followMode = false;
      lineMode = false;
    }
    else if(strcmp(buf,"FOLLOW") == 0)
    {
      controlMode = false;
      followMode = true;
      lineMode = false;
    }
    else if(strcmp(buf,"LINE") == 0)
    {
      controlMode = false;
      followMode = false;
      lineMode = true;
    }

    //Tryb Kontrola komórką
    if(controlMode == true)
    {
      motor_left.run(RELEASE);
      motor_right.run(RELEASE);
      if(strcmp(buf,"LEFTON") == 0)
      {
        motor_right.run(FORWARD);
      }
      else if(strcmp(buf,"UPON") == 0)
      {
        motor_left.run(FORWARD);
        motor_right.run(FORWARD);
      }
      else if(strcmp(buf,"RIGHTON") == 0)
      {
        motor_left.run(FORWARD);
      }
      else if(strcmp(buf,"DOWNON") == 0)
      {
        motor_left.run(BACKWARD);
        motor_right.run(BACKWARD);
      }
      else if(strcmp(buf,"DOWNON") == 0)
      {
        motor_left.run(BACKWARD);
        motor_right.run(BACKWARD);
      }
      else if(strcmp(buf,"FOLLOW") == 0)
      {
        controlMode = false;
        followMode = true;
      }
      else if(strcmp(buf,"LINE") == 0)
      {
        controlMode = false;
        lineMode = true;
      }
    }
    bufRDY = false;
  }

  //Tryb Podążanie za obiektem
  if(followMode == true)
  {    
    followObj();
  }

  //Tryb Podążanie za linią
  if(lineMode == true)
  {    
    //TODO
  }

}    


//Funkcja odpowiedzialna za pobranie i obróbkę
//danych z czyjnika ultradźwiękowego
unsigned int sensorSonicRead(unsigned int trig,unsigned int echo)
{
  unsigned long duration = 0;
  unsigned int distance;
  int i;
  digitalWrite(trig,LOW);
  for(i=0;i<4;++i)
  {
    delayMicroseconds(2);
    digitalWrite(trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(trig,LOW);
    duration += pulseIn(echo,HIGH,20000);
  }
  duration = duration/4;
  distance = duration*0.0173;
  return distance;
}

//Funkcja odpowiedzialna za podążanie
//robota za obiektem
void followObj()
{
  motor_left.run(RELEASE);
  motor_right.run(RELEASE);
  
  //Odczyt odległości z czyjników ultradźwiękowych
  dist_left = sensorSonicRead(TRIG1,ECHO1);
  dist_right = sensorSonicRead(TRIG2,ECHO2);

  //Logika podążania za obiektem
  if((dist_left < 20) && (dist_right < 20))
  {
    motor_left.run(BACKWARD);
    motor_right.run(BACKWARD);
  }
  else if((dist_left > 30) && (dist_right > 30) && (dist_left < 50) && (dist_right < 50))
  {
    motor_left.run(FORWARD);
    motor_right.run(FORWARD);
  }
  else
  {
    //Kiedy powyższe warunki if nie są spełnione
    //robot nie widzi obiektu
    //obraca się o około 70 stopni w lewo i prawo
    search = 1;
    
    //Ustawienie prescaler 256 i włączenie Timer1
    TCCR1B = 4;
    //Załadowanie wartości do Timer1
    TCNT1 = timer1_load1;
    
    while(search != 0)
    {
      dist_left = sensorSonicRead(TRIG1,ECHO1);
      dist_right = sensorSonicRead(TRIG2,ECHO2);
      if((dist_left < 50) && (dist_right < 50))
      {
        motor_left.run(RELEASE);
        motor_right.run(RELEASE);
        search = 0;
        TCCR1B = 0;
      }
      delay(10);
    }
    
  }
  delay(100);
}

//Obsługa przerwania Timer1 steruje
//silnikami w gdy robot zgubi obiekt
//za którym podąża
ISR(TIMER1_OVF_vect)
{
  if(search == 1)
  {
    TCNT1 = timer1_load1;
    search = 2;
    //Turn Left
    motor_left.run(BACKWARD);
    motor_right.run(FORWARD);
  }
  else if(search == 2)
  {
    TCNT1 = timer1_load2;
    search = 3;
    //Stop
    motor_left.run(RELEASE);
    motor_right.run(RELEASE);
  }
  else if(search == 3)
  {
    TCNT1 = timer1_load2;
    search = 4;
    //Turn Right
    motor_left.run(FORWARD);
    motor_right.run(BACKWARD);
  }
  else if(search == 4)
  {
    TCNT1 = timer1_load2;
    search = 5;
    //Stop
    motor_left.run(RELEASE);
    motor_right.run(RELEASE);
  }
  else if(search == 5)
  {
    TCNT1 = timer1_load2;
    search = 2;
    //Turn Left
    motor_left.run(BACKWARD);
    motor_right.run(FORWARD);
  }
}
