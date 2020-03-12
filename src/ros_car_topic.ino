/*
   Simple car on ESP8266 with rosserial
   roslaunch rosserial_server socket.launch
   The default port is 11411
*/

#include <ESP8266WiFi.h>
#include <ros.h>
// TODO: add custom msg
#include <std_msgs/ColorRGBA.h>

#define LEFT_FORWARD 13
#define LEFT_BACK 12
#define RIGHT_FORWARD 14
#define RIGHT_BACK 16

// Set Wi-Fi settings
const char* ssid     = "<your ssid>";
const char* password = "<your password>";
// Set the rosserial socket server IP address
IPAddress server(192, 168, 1, 1); // SET IP OF COMPUTER WITH ROSSERVER
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

void drive(const int linear, const int radian) {
  Serial.print("Linear speed: ");
  Serial.println(linear);
  Serial.print("Radian speed: ");
  Serial.println(radian);
  int left_weel = linear - radian;
  int right_weel = linear + radian;
  // Use pwm to slow down, else car move very fast
  int pwm = 240;
  analogWrite(LEFT_FORWARD, left_weel > 0 ? pwm : 0);
  analogWrite(LEFT_BACK, left_weel < 0 ? pwm : 0);
  analogWrite(RIGHT_FORWARD, right_weel > 0 ? pwm : 0);
  analogWrite(RIGHT_BACK, right_weel < 0 ? pwm : 0);
}

// Callback for processing msg from drive topic
void messageCb(const std_msgs::ColorRGBA& toggle_msg) {
  drive(toggle_msg.r, toggle_msg.g);
}

ros::Subscriber<std_msgs::ColorRGBA> sub("drive", messageCb );
ros::NodeHandle nh;

void setup()
{
  // Use ESP8266 serial to monitor the process
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.subscribe(sub);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  drive(0, 0);
}

bool connected = true;
void loop()
{
  nh.spinOnce();
  if (!nh.connected()) {
    connected = false;
    Serial.println("Not Connected to ROS!");
    delay(500);
  } else if (!connected) {
    connected = true;
    Serial.println("Connected to ROS!");
  }
  delay(1);
}
