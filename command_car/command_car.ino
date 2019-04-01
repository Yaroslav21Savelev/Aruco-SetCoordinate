#include <SoftwareSerial.h>
#define IN1 7
#define IN2 6
#define IN3 5
#define IN4 4
#define EN1 9
#define EN2 3

#define minSpeed 45


SoftwareSerial RasPi(A4, A5); // RX, TX
volatile unsigned long timer, stT;
String Stroke;
volatile int get_data[255];

int s1, s2;
bool d1, d2, stF;

void setup()
{
  
  pinMode (EN1, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (EN2, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (IN3, OUTPUT);
  Serial.begin(115200);
  RasPi.begin(57600);
  //timerInit();  // Инициализация таймера
  //RasPi.write("Hello, world?\n");
  //Serial.println(180);
  Get_Marker();
}

void loop()
{ 
  Get_Marker();
  if(millis() - timer <= 2000)
  {
    if(get_data[0] == 0)
      s1 = 0;
    else if(get_data[0] < 0)
    {
      digitalWrite (IN1, LOW);
      digitalWrite (IN2, HIGH); 
      s1 = map(get_data[0], -100, -1, 255, minSpeed);
    }
    else if(get_data[0] > 0)
    {
      digitalWrite (IN1, HIGH);
      digitalWrite (IN2, LOW); 
      s1 = map(get_data[0], 1, 100, minSpeed, 255);
    }
    if(get_data[1] == 0)
      s2 = 0;
    else if(get_data[1] < 0)
    {
      digitalWrite (IN3, HIGH);
      digitalWrite (IN4, LOW); 
      s2 = map(get_data[1], -100, -1, 255, minSpeed);
    }
    else if(get_data[1] > 0)
    {
      digitalWrite (IN3, LOW);
      digitalWrite (IN4, HIGH); 
      s2 = map(get_data[1], 1, 100, minSpeed, 255);
    }
    if(millis() - stT >= 40 && stF || millis() - stT >= 10 && !stF)
    {
      if(stF)
      {
        analogWrite(EN1, s1);
        analogWrite(EN2, s2);
      }
      else
      {
        analogWrite(EN1, 0);
        analogWrite(EN2, 0);
      }
      stT = millis();
      stF = !stF;
    }
  }
  else
  {
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
  }
}
void Get_Marker()
{
  while(RasPi.available())
  {
    int inChar = RasPi.read();
    Stroke += (char)inChar;
    if (inChar == '\n')
    {
      char input[255];
      byte sn = 0;
      for (int j = 0; j < Stroke.length(); j++)
      {
        input[j] = Stroke[j];
      }
      Stroke = "";
      int i = 0;
      while (input[i] && input[i] != '\n')
      {
        if (input[i] == ',')
        {
          i++;
          String oper_save;
          while (input[i] && input[i] != ',')
          {
            //Serial.print(input[i]);
            oper_save += input[i];
            i++;
          }
          get_data[sn] = atoi(oper_save.c_str());
          sn++;
        }
        i++;
      }
      for (int k = sn; k < 255; k++)
      {
        get_data[k] = -1;
      }
      //get_data[2] = rotate(get_data[2] - 90);
      //Вывод данных в монитор
      for (int k = 0; k < 20; k++)
      {
        Serial.print(get_data[k]);
        Serial.print(" ");
      }
      Serial.println();
      timer = millis();
    }
  }
}
