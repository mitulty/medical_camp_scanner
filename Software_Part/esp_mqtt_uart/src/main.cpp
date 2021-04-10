#include <WiFi.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define RXD2 16
#define TXD2 17
#define LED_PIN 2
#define BUTTON_PIN 0

// Replace the next variables with your SSID/Password combination
const char *ssid = "Morack";
const char *password = "mitul45M";

// Add your MQTT Broker IP address, example:
// const char* mqtt_server = "192.168.1.144";
const char *mqtt_server = "192.168.43.46";
long lastMsg = 0;

WiFiClient espClient;
PubSubClient client(espClient);
char jsonStr[50];
StaticJsonDocument<50> doc_main;
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char recdJsonStr[100];
  int i;
  for (i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    recdJsonStr[i] = (char)message[i];
  }
  recdJsonStr[i]='\0';
  Serial.printf("\nRecevied JSON String: %s\n", recdJsonStr);
  StaticJsonDocument<100> doc;
  
  deserializeJson(doc, recdJsonStr);
  Serial2.write(recdJsonStr);

}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32", "AfDmr6rrnImHE3ZbrxtD", ""))
    {
      Serial.println("connected");
      // Subscribe
      // v1/devices/me/telemetry
      client.subscribe("data_recv");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  if (Serial2.available())
  {
    bool buttonState = digitalRead(BUTTON_PIN);
    doc_main = Serial2.readString();
    serializeJson(doc_main, jsonStr);
    Serial.printf("Serialized JSON: %s\n", jsonStr);
    client.publish("data_sent", jsonStr);
  }
}
