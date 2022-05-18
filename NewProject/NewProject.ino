#include <WiFi.h>
#include <HTTPClient.h>

#define WIFI_NAME "Pixel_4334"
#define WIFI_PWD  "3ad3ad3ad"

//Your Domain name with URL path or IP address with path
String API = "http://portableauth.pythonanywhere.com/check-id/";


void setup() 
{
  //UART Setup
  Serial.begin(115200); 

  //WIFI Connect
  WiFi.begin(WIFI_NAME, WIFI_PWD);

  //Wait till WIFI connected
  while(WiFi.status() != WL_CONNECTED) 
    delay(500);
}

void loop() 
{

  String ID = ""; 
  
  while (Serial.available() > 0) 
  {
    char Buffer = Serial.read(); 
    ID += Buffer;     
  }
        
  //If ID was receieved, check if it exists
  if (ID != "") 
  {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED)
    {
      HTTPClient http;

      String API_Request = API + ID + '/'; 
      Serial.println (API_Request);
      
      // Your Domain name with URL path or IP address with path
      http.begin(API_Request.c_str());
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) 
      {
        //Response (1): User ID found 
        String Payload = http.getString(); 
        
        if (Payload == "True")
         Serial.println("1");

        //Response (0): User ID not found
        else 
         Serial.println("0");
      }
      else 
      {
         //Error code (-2): GET Request Failed
         Serial.println("-2");
      }
      // Free resources
      http.end();
    }
    else 
    {
      //Error code (-1): ID recieved, but WIFI not connected
      Serial.println("-1");
    }
  }
}
