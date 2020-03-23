#include <ESP8266WiFi.h>  
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"


#define WIFI_SSID "agribot"
#define WIFI_PASS "agribot321"
#define WIFI_SSID "swaru"
#define WIFI_PASS "swaru65089"


#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "ssrvca"
#define MQTT_PASS "aio_YTan82kQOyHcHssvL5Ju8p0r2EsA"
#define ROBO_REPLY_MSG "I am AGRIBOT"


int forward1  = D0;    /////               Board1 IN2.
int reverse1  = D1;    /////               Board1 IN1.
int left1     = D2;    /////               Board1 IN4.
int right1    = D3;    /////               Board1 IN3.
int waterpump = D5;    /////               Board2 IN1.
int grasscut  = D6;    /////               Board2 IN3.
int alert     = D7;    /////               Buzzer positive.
  /////////////////////////       NOTE:  A0 is connected to the A0 of soil moisture sensor.
 
static int alert_state;
static int water_state;
static int grass_state;


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);


Adafruit_MQTT_Subscribe d_forward = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/forward");
Adafruit_MQTT_Subscribe d_reverse = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/reverse");
Adafruit_MQTT_Subscribe d_left = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/left");
Adafruit_MQTT_Subscribe d_right = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/right");
Adafruit_MQTT_Publish robotreplymessage = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/message");
Adafruit_MQTT_Subscribe alert_bird = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/alert");
Adafruit_MQTT_Subscribe water_moisture = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/moisture");
Adafruit_MQTT_Subscribe water_pump = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/water");
Adafruit_MQTT_Subscribe grass_cutter = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/grass");


void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
   
  Serial.print("WIFI Connecting..............\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(">\n");
    delay(50);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
  }
  Serial.print("WIFI Connected\n");
 
  //subscribe for the necessary feeds
  mqtt.subscribe(&d_forward);
  mqtt.subscribe(&d_reverse);
  mqtt.subscribe(&d_left);
  mqtt.subscribe(&d_right);
  mqtt.subscribe(&water_moisture);
  mqtt.subscribe(&alert_bird);
  mqtt.subscribe(&water_pump);
  mqtt.subscribe(&grass_cutter);
  
  robotreplymessage.publish(ROBO_REPLY_MSG);
  

  
  pinMode(forward1,OUTPUT);
  pinMode(reverse1,OUTPUT);
  pinMode(left1,OUTPUT);
  pinMode(right1,OUTPUT);
  pinMode(alert,OUTPUT);
  pinMode(waterpump,OUTPUT);
  pinMode(grasscut,OUTPUT);
  
  
}

void loop()
{
  //Connect/Reconnect to MQTT
  MQTT_connect();

  //Read from our subscription queue until we run out, or
  //wait up to 5 seconds for subscription to update
  Adafruit_MQTT_Subscribe * subscription;
  while ((subscription = mqtt.readSubscription(5000)))
  {
////////////////////////////////////////////////////////////////////  FORWARD  
    //digitalWrite(LED_BUILTIN,LOW);

    if (subscription == &d_forward)
    {
      
      //scan whether forward button is pressed
      if (strcmp((char*) d_forward.lastread, "0"))                         
      {
        digitalWrite(forward1, HIGH);
        digitalWrite(reverse1, LOW);
        digitalWrite(left1, HIGH);
        digitalWrite(right1, LOW); 
        Serial.print("moving forward\n");       
        robotreplymessage.publish("Moving Forward");
      }
      else
      {
        digitalWrite(forward1, LOW);
        digitalWrite(reverse1, LOW);
        digitalWrite(left1, LOW);
        digitalWrite(right1, LOW);   
        
      }
    }

//////////////////////////////////////////////////////////// REVERSE 
    if (subscription == &d_reverse)                                           
    {
      
      
      if (strcmp((char*) d_reverse.lastread, "0"))
      {
        digitalWrite(forward1, LOW);
        digitalWrite(reverse1, HIGH);
        digitalWrite(left1, LOW);
        digitalWrite(right1, HIGH);
        Serial.print("moving reverse\n"); 
        robotreplymessage.publish("Moving Reverse");
      }
      else 
      {
          digitalWrite(forward1, LOW);
          digitalWrite(reverse1, LOW);
          digitalWrite(left1, LOW);
          digitalWrite(right1, LOW);  
          
      }
    }

////////////////////////////////////////////////////////////// LEFT     
    if (subscription == &d_left)                                     
    {
      
      //scan whether reverse button is pressed
      if (strcmp((char*) d_left.lastread, "0"))
      {
        digitalWrite(forward1, LOW);
        digitalWrite(reverse1, HIGH);
        digitalWrite(left1, HIGH);
        digitalWrite(right1, LOW);
        Serial.print("took left\n");
        robotreplymessage.publish("Took Left"); 
      }
      else 
      {
        
          digitalWrite(forward1, LOW);
          digitalWrite(reverse1, LOW);
          digitalWrite(left1, LOW);
          digitalWrite(right1, LOW);
          //robotreplymessage.publish("Idle");  
      }
    }

    
///////////////////////////////////////////////////////////////////////////  RIGHT  
    if (subscription == &d_right)                                
    {
      
      //scan whether reverse button is pressed
      if (strcmp((char*) d_right.lastread, "0"))
      {
        digitalWrite(forward1, HIGH);
        digitalWrite(reverse1, LOW);
        digitalWrite(left1, LOW);
        digitalWrite(right1, HIGH);
        Serial.print("took right\n"); 
        robotreplymessage.publish("Took Right");
      }
      else 
      {
        
          digitalWrite(forward1, LOW);
          digitalWrite(reverse1, LOW);
          digitalWrite(left1, LOW);
          digitalWrite(right1, LOW); 
          //robotreplymessage.publish("Idle"); 
      }
    }
//////////////////////////////////////////////////////////           ALERT 
    if (subscription == &alert_bird)
    {
      if (strcmp((char*) alert_bird.lastread, "0"))
      {
        alert_state=~alert_state;

        if(alert_state)
        {
          digitalWrite(alert, HIGH);
        Serial.print("alert_on\n");
        robotreplymessage.publish("Alert On");
        }
        else
        {
          digitalWrite(alert, LOW);
        Serial.print("alert_off\n");
        robotreplymessage.publish("Alert Off");
          
        }        
      }      
    }

/////////////////////////////////////////////////////         MOISTURE SENSOR 
    
   
    if (subscription == &water_moisture)
    {
      
      //scan whether reverse button is pressed
      if (strcmp((char*) water_moisture.lastread, "0"))
      {
        int data=analogRead(A0);  
     robotreplymessage.publish(data); 
     //String data1=data; 
      Serial.print(data);
      }
      
    }
    
///////////////////////////////////////////////////          WATER PUMP
if (subscription == &water_pump)
    {
      if (strcmp((char*) water_pump.lastread, "0"))
      {
        water_state=~water_state;
        
        if(water_state)
        {
          digitalWrite(waterpump, HIGH);
        Serial.print("water_pump_on\n");
        robotreplymessage.publish("Water pump on");
        }
        else
        {
         digitalWrite(waterpump, LOW);
        Serial.print("water pump_off\n");
        robotreplymessage.publish("Water pump Off");
                  
        }        
      }      
    }
////////////////////////////////////////////////////    GRASS CUTTER
 if (subscription == &grass_cutter)
    {
      if (strcmp((char*) grass_cutter.lastread, "0"))
      {
        grass_state=~grass_state;
        
        if(grass_state)
        {
          digitalWrite(grasscut, HIGH);
        Serial.print("Grass cutter on\n");
        robotreplymessage.publish("Grass cutter on");
        }
        else
        {
         digitalWrite(waterpump, LOW);
        Serial.print("Grass cutter off\n");
        robotreplymessage.publish("Grass cutter off");
                  
        }        
      }      
    }
////////////////////////////////////////////////////
  }
}


void MQTT_connect()
{

  //  // Stop if already connected
  if (mqtt.connected() && mqtt.ping())
  {
    //    mqtt.disconnect();
    return;
  }

  int8_t ret;

  mqtt.disconnect();

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) // connect will return 0 for connected
  {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0)
    {
      ESP.reset();
    }
  }
  Serial.println("MQTT Connected!");
}
