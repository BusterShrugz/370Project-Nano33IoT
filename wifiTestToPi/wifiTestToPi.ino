#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>

char ssid[] = "NETGEAR18";
char pass[] = "jaggedstreet684";

IPAddress piAddress(192,168,0,23); // Your Pi’s IP address
unsigned int udpPort = 5005;          // UDP port to send to
WiFiUDP Udp;

int status = WL_IDLE_STATUS;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected!");
  Serial.print("Arduino IP: ");
  Serial.println(WiFi.localIP());

  // Start UDP on a local port
  Udp.begin(5005);
  Serial.println("UDP started on local port 5005");

  // Start IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Temperature sensor sample rate = ");
  Serial.print(IMU.temperatureSampleRate());
  Serial.println(" Hz");
  Serial.println("Temperature readings in degrees C:");
}

void loop() {
  float temp;

  if (IMU.temperatureAvailable()) {
    IMU.readTemperature(temp);
    Serial.print("Temp: ");
    Serial.println(temp);

    // Create message
    String message = "Temperature: " + String(temp, 1);

    // Send via UDP
    Serial.println("Sending UDP packet...");
    Udp.beginPacket(piAddress, udpPort);
    Udp.write((byte*)&temp, sizeof(temp));
    int result = Udp.endPacket(); 
    Serial.print("endPacket() result: ");
    Serial.println(result);
  }

  delay(1000);
}