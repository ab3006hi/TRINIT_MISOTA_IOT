#include <esp_now.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

#define inter_lat 495621 //Assume intersection lattitude we can get this data from the map data
#define inter_long 890326 //It is longitude similarly


void transmitInfo(){
  // Finds the time to reach the intersection
  if (gps.location.isValid()){
    float dist = distance_between(gps.location.lat(),gps.location.long(),inter_lat,inter_long);
    float time = dist /1;
    return time;
  }  
  else{
    Serial.print(F("INVALID"));
  }
}

float distance_between (float lat1, float long1, float lat2, float long2) {
  // returns distance in meters between two positions, both specified in lattitudes and longitudes 
  // In this case the distance between the car and intersection

  float del_long = radians(long2 - long1);
  float del_long = radians(lat2 - lat1);
  float delta = radians(lat1+lat2)/2;
  float x = del_long*cos(delta);
  float y = del_lat;
  float d = 6372795* sqrt((x*x)+(y*y));
  return d
}

bool new_data = 0; 

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; //Common Receiver MAC Address for all the edge modules

int n; // No. of cars arriving at the junction
int cars[n]=[]; // Priority Order of the Cars


int arrival_time = transmitInfo(); // Arrival Time of the host car

int cars_time; //Incoming arrival time of cars

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Arranging the Whole Cars time's in an array.
  int k=0;
  while(k<n)
  {
    memcpy(&cars_time, incomingData, sizeof(cars_time));
    Serial.print("Bytes received: ");
    Serial.println(len);
    CAR_time = cars_time;
    Serial.println(cars_time);
    cars[k]=CAR_time;
    k++;
  }
}

void setup() {
  Serial.begin(9600);// setting up serial port for gps sensor data
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  delay(3000);
}

int linearSearch(int a[], int n, int val) {  
  //Linear Search Algorithm
  for (int i = 0; i < n; i++)  
    {  
        if (a[i] == val)  
        return i+1;  
    }  
  return -1;  
}  

void process_data(int cars[],int n)
{
  int i, key, j;
  for (i = 1; i < n; i++)
  {
      key = cars[i];
      j = i - 1;
      while (j >= 0 && cars[j] > key)
      {
          cars[j + 1] = cars[j];
          j = j - 1;
      }
      cars[j + 1] = key;
    }
}
void loop()
{
  new_data=0; //Setting up to send next data.
  // Using GPS Info to calculate the arrival time of the host car
  while (Serial.available() > 0)
  {
  if (gps.encode(Serial.read())) // The data sent by the GPS is read and passed to transmitInfo function to get the time to reach the junction
  {   
      float time = transmitInfo();
      delay(1000);
  }
  if (millis() > 5000 && gps.charsProcessed() < 10){
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  
  esp_err_t result; 
  
  // Sending packet of data consisting info. on arrival time of host car
  if (new_data == 0)
  {
    result = esp_now_send(broadcastAddress, (uint8_t *) &arrival_time, sizeof(arrival_time));
    new_data = 1;
  }

  if (result == ESP_OK) 
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }

  //Data Reception Part
  process_data(); //Processing the Data received from all the cars in the junction.

  pos=linearsearch(int cars[],int n, int transmitInfo()); //Calculating Priority of the Host Car

  //If priority matches with existing available car time then decrease speed to let the previous car go.
  if(pos)
  {
    Serial.println("Slow Down");
  }

  else
  {
    Serial.println("Keep Moving at the same speed, Safe to Go");
  }
}
