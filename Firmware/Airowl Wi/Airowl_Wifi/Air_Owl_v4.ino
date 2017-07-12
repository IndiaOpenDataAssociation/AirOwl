#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFiMulti.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>

#define LED_Pin 13      // GPIO pin  <-- Pin 13 is controls the led colour
#define NUMPIXELS 2     // Number of leds to be controlled


byte data[24];
int code_version = 1;

String send_url = "";
String oz_deviceId = "";

WiFiClient wifiClient;
//PubSubClient tubClient(wifiClient);
ESP8266WebServer server(80);

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_Pin, NEO_GRB + NEO_KHZ800); //Initializing credentials for LED control

int status = WL_IDLE_STATUS;

unsigned int address = 7;
unsigned int total_ssid = 0, store_address = 0, timestart = 0, flag = 0, val = 0, ap_set = 0, s = 0, j = 0, k = 0;
unsigned int set_limit =2; //set the number of wifi to be remembered by the module
char wifissid[20], wifipswd[20];  //used to get  the stored ssid and password in EEPROM
String ssid1(wifissid), pswd2(wifipswd);//string obatined from reading data of eeprom
int cpy_srt = 0, cpy_end = 0, addr = 7; //variables used to delete previous stored ssid and password from eeprom


String ssid     = "";
String password = "";

bool send_data_flag = 0;

unsigned int PM25 = 0, PM10 = 0, PM1 = 0;
unsigned int count = 0;

boolean ap_flag = 0;
boolean init_flag = 1;
boolean retAp = 0;
boolean extra = 0;
boolean dataFlag = 1;
int dustPart = 0;
int inpin = 12;
char MACAddress[12];

// DNS server
const byte DNS_PORT = 53;
DNSServer serverDNS;
/* Soft Access Point network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

// hostname for mDNS. Try http://logger_one_wire.local
const char *myHostname = "airowl";

// cycle time in loop()
unsigned long cycleTime = 120000;  // ms
const unsigned long sec24h = 24 * 3600; // one day in seconds
unsigned long convTime = millis(); // conversion time about 1000 ms
boolean startSync = false;

unsigned long timeStart;
unsigned long timeRel;

// for NTP time server
unsigned int localPort = 2390;      // local port to listen for UDP packets
unsigned long epoch;
unsigned long timeNTPstart;
unsigned long timeUnix;

ESP8266WiFiMulti WiFiMulti;

//Function declaration
void Eeprom_store(void);
void Dust_sensor();
void initialize_sensors();
int Send_Payload();
void init_wifi();
void Get_data();
void SendSensorData(void);
void initDustSensor(void);
void getDustData(void);
//boolean reconnect();
void initApMode();
void getConfiguration();
void ledColor(byte, byte, byte);
void dataColor(unsigned int);
void getWinsenData(void);
void getNovaData(void);

// from handleHttp.ino
/* Handle root or redirect to captive portal */
void handleRoot();
void getNTPtime();
/* Redirect to captive portal if we got a request for another domain.
  Return true in that case so the page handler do not try to handle the request again. */
boolean captivePortal();

/* Wifi config page handler */
void handleWifi();

/* Handle the WLAN save form and redirect to WLAN config page again */
void handleWifiSave();

void handleNotFound();

// from tools.ino
/*Is this an IP? */
boolean isIp(String str);

/*IP to String? */
String toStringIp(IPAddress ip);

/*EEPROM*/
void wifi_check();
void delete_prevssid();
/*ISR*/
void call_ap();
//Setup begin
void setup() {
  pinMode(inpin, INPUT_PULLUP);
  attachInterrupt(inpin, call_ap, FALLING);
  Serial.begin(9600);
  EEPROM.begin(512);
  WiFi.disconnect();
  delay(10);

  pixels.begin();                                       //LED initialisation

  uint8_t MAC_array[6];

  WiFi.macAddress(MAC_array);                           //Accuring MAC id for ESP module
  for (int i = 0; i < sizeof(MAC_array); ++i) {
    sprintf(MACAddress, "%s%02x", MACAddress, MAC_array[i]);
  }

  String address = String(MACAddress);
  oz_deviceId = "AirOwl_" + address.substring(7, 12);  //Unique device ID

  // Setup MDNS responder
  if (!MDNS.begin("airowl")) {

  } else {
    MDNS.addService("http", "tcp", 80);
  }

  // Setup the DNS server redirecting all the domains to the apIP
  serverDNS.setErrorReplyCode(DNSReplyCode::NoError);
  serverDNS.start(DNS_PORT, "*", apIP);

  //Initial Led check
  // Green Color for 1 second
  ledColor(0, 255, 0); delay(1000);
  // Red Color for 1 second
  ledColor(255, 0, 0); delay(1000);
  // Blue Color for 1 second
  ledColor(0, 0, 255); delay(1000);

  /*delete_prevssid();
    while(1);*/
}


void loop() {

  if (init_flag)
  {
    initialize_sensors();  //Initializing sensors

    timeStart = millis();
    timeRel = millis() + cycleTime; // avoid unsigned long rollover after 40 days

    init_flag = 0;
  }

  boolean val = digitalRead(inpin);  //Manually getting into AP mode

  if (val == 0) {
    extra = 1;
    init_flag = 1;
  }

  if (ap_flag == 0)
  {
    
    if(dustPart ==12)
    {
    getNovaData();
    }
    else if(dustPart == 11)
    {
      getWinsenData();
    }
    if(PM10>0 && PM25>0)
    {
    Get_data();
    }

    timeUnix = (millis() - timeStart) / 1000 + timeNTPstart;

    // synch clock with NTP every 24h, must run every second
    if ((timeUnix % sec24h) == 0) {
      startSync = true;
    }
    delay(10);  // to avoid strange characters in buffer
  }

  if (WiFi.status() == WL_DISCONNECTED)
  {
    init_flag = 1;

  }


  // DNS
  serverDNS.processNextRequest();
  // HTTP
  server.handleClient();
}

void getDustData(void)
{
  if(dustPart == 11)
  {
    getWinsenData();
  }
  else if(dustPart == 12)
  {
    getNovaData();
  }
}

void getWinsenData(void)
{
  byte i = 0;
  int checksum = 0;
  Serial.flush();
  while (Serial.available()) {
    data[i] = Serial.read();
    if (i <= 21)
    {
      checksum += data[i];
    }
    if (i == 23) {
      if (checksum == ((256 * data[22]) + data[23]))
      {
        if (data[0] == 66)
        {
          if (data[1] == 77)
          {
            if (i == 23)
            {
              PM1 += ((data[4] * 256) + data[5]);
              PM25 += ((data[6] * 256) + data[7]);
              PM10 += ((data[8] * 256) + data[9]);
              count++;
              Serial.flush();
              break;
            }
          }
        }
      }
      else
      {
        break;
      }
    }
    i++;
    delay(10);
  }
}

void getNovaData(void)
{
  byte i = 0;
  int checksum = 0;
  Serial.flush();
  while (Serial.available()) {
    data[i] = Serial.read();
    
    if(i == 9)
    {
      if(data[0] == 170)
      {
        PM25 += ((data[3] * 256) + data[2]);
        PM10 += ((data[5] * 256) + data[4]);
        count++;
        //Serial.println(count);
        Serial.flush();
        break;
      }
      i = 0;
      
    }
    i++;
    delay(10);
  }
}

/******************************/
/* Getting data */
/******************************/
void Get_data(void)
{
  boolean val = digitalRead(inpin);
  if (val == 0) {
    extra = 1;
    init_flag = 1;
  }
  getDustData();
  delay(1000);
  
  if (dataFlag == 1)
  {
    //Serial.println("Here");
    convTime = millis();
    send_data_flag = 1;
    dataFlag = 0;
  }

  if ( (long)( millis() - timeRel ) >= 0) { // run every cycleTime
    timeRel = millis() + cycleTime;
    if (startSync) {
      getNTPtime();
      if (epoch > 0) {
        timeNTPstart = epoch;
        timeStart = millis();
        startSync = false;
      }
    }
    convTime = millis();
    send_data_flag = 1;
  }

  if (send_data_flag == 1 && millis() - convTime > 1000 )
  {
    
    //Serial.println("Sending data");
    SendSensorData();
  }
}


void SendSensorData(void)
{
  if (send_data_flag == 1)
  {
    PM25 = PM25 / count;
    PM10 = PM10 / count;
    PM1  = PM1  / count;
  }

  if (Send_Payload())
  {
    dataColor(PM10);
    PM25 = 0;
    PM10 = 0;
    PM1  = 0;
    count = 0;
    send_data_flag = 0;
  }
  else
  {
    count = 1;
  }
}

int Send_Payload()
{
  if (WiFi.status() != WL_CONNECTED) {
    init_wifi();
  }

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& payload = jsonBuffer.createObject();
  JsonObject& dataJson = payload.createNestedObject("d");

  dataJson["t"] = timeUnix;
  dataJson["p1"] = PM25;
  dataJson["p2"] = PM10;
  dataJson["p3"] = PM1;

  char buff[200];
  payload.printTo(buff, sizeof(buff));

  if ((WiFiMulti.run() == WL_CONNECTED)) {

    HTTPClient http;
    http.begin(send_url);
    http.addHeader("Content-Type", "application/json");
    //http.addHeader("Authorization", "Basic dXNlLXRva2VuLWF1dGg6aW5kaWFvcGVuZGF0YQ==");
    int httpCode = http.POST(String(buff));

    http.end();
    if (httpCode == 200)
    {

      return 1;
    }
    else
    {
      return 0;
    }
  }
}

void init_wifi(void)
{

  // We start by connecting to a WiFi network
  //WiFi.setAutoConnect(1);


Serial.println("MILI");
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect();

    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
    }

    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 7000) {
      delay(500);

    }

  }


  if (WiFi.status() == WL_CONNECTED)
  {
    if (flag == 1)
    {
      Eeprom_store();
      flag = 0;
    }
    //Serial.println("Wifi Connected");
    for (int j = 0 ; j <= 8 ; j++)
    {
      ledColor(0, 0, 0);            // LED colour blinks 8 times --> BLUE ..Indicating successfull connection with wifi
      delay(300);
      ledColor(0, 0, 255);
      delay(300);
    }
    getConfiguration();
  }
  else
  {
    ap_flag = 1;
    init_flag = 1;
    retAp = 0;
    ledColor(255, 0, 0);
  }

}

/******************************/
/* Initilize Wi-Fi to AP Mode */
/******************************/
void initApMode()
{
 
  ledColor(255, 0, 255);
  WiFi.disconnect();
  server.onNotFound ( handleNotFound );
  server.on("/", handleRoot);
  server.on("/wifi", handleWifi);
  server.on("/wifisave", handleWifiSave);
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.mode(WIFI_AP);  // Access Point & Station mode
  delay(10);
  WiFi.softAPConfig(apIP, netMsk, netMsk);
  WiFi.softAP(oz_deviceId.c_str());
  WiFi.begin();  // for test
  delay(500); // Without delay I've seen the IP address blank
  //delay(1000);

  server.begin(); // Web server start

  retAp = 0;
  delay(1000);
}

/******************************/
/* Initialize Sensors */
/******************************/
void initialize_sensors(void)
{
  delay(5000);
  Serial.println("rock");
  if (extra == 0 && ap_set == 0)
  {
    wifi_check();
    
  }
  // Check wifi status to start AP Mode
  if (!WiFi.isConnected() && retAp == 0)
  {
    ap_flag = 1;
    
  }
  else
  {
    ap_flag = 0;
   
  }

  if (extra)
  {
    ap_flag = 1;
    
  }
  

  if (ap_flag == 1)
  {
    extra = 0;
    initApMode();
    ap_set = 1;
  }
  else
  {
    init_wifi();
    initDustSensor();
    getNTPtime();
  }

  PM25 = 0;
  PM10 = 0;
  PM1 = 0;
  count = 0;
}

/******************************/
/* Get Configuration*/
/******************************/
void getConfiguration()
{
  for (int j = 0 ; j <= 3 ; j++)
  {
    ledColor(0, 0, 0);            // LED colour blinks 3 times --> WHITE ..Indicating successfull connection with wifi
    delay(300);
    ledColor(255, 255, 255);
    delay(300);
  }
  HTTPClient http;
  //Serial.println(MACAddress);
  String path = "http://api.airpollution.online/config/device/" + String(MACAddress);

  http.begin(path); //HTTP

  // start connection and send HTTP header
  int httpCode = http.GET();

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      // Json buffer to get response
      DynamicJsonBuffer jsonBuffer;
      JsonObject& root = jsonBuffer.parseObject(payload);
      //Checking if the JSON was parsed perfectly
      if (!root.success())
      {
        http.end();
        return;
      }
      else
      {
        http.end();
        //Organiztion to connect to for all communication
        //const char *test = root["org"];
        String oz_org = (const char*)root["org"];
        //Device ID and Device Type to register to send data
        String oz_deviceType = (const char*)(root["deviceType"]);
        //String oz_deviceId = (const char*)(root["deviceId"]);
        String oz_pass = (const char*)(root["pass"]);
        dustPart = int(root["dustPart"]);
        cycleTime = int(root["interval"]);
        code_version = int(root["codeVersion"]);
        //Serial.println(dustPart);
        send_url = "http://use-token-auth:" + oz_pass + "@" + oz_org + ".messaging.internetofthings.ibmcloud.com/api/v0002/device/types/" + oz_deviceType + "/devices/" + oz_deviceId + "/events/data";
        return;
      }
    }
  }

  http.end();
}


void ledColor(byte red, byte green, byte blue)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
  }
}

void dataColor(unsigned int value)
{
  if (value > 350)
  {
    ledColor(255, 0, 0);  //LED clour --> RED..Indicating VERY high dust value
  }
  else if (value > 100)
  {
    ledColor(255, 255, 0); //LED clour --> YELLOW..Indicating high dust value
  }
  else
  {
    ledColor(0, 255, 0);  //LED clour --> GREEN..Indicating GOOD air quality
  }
}


void initDustSensor()
{
  if(dustPart == 11){
  const unsigned char cmd_get_sensor[] =
  {
    0xff, 0x01, 0x78, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x47
  };

  // Send cmd to get data
  for (int i = 0; i < sizeof(cmd_get_sensor); i++)
  {
    Serial.write(cmd_get_sensor[i]);
  }
  delay(10);
  }
}

void getNTPtime() {
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    HTTPClient http;
    http.begin("http://api.airpollution.online/v1/airowl/timestamp");

    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled

      // file found at server
      if (httpCode == HTTP_CODE_OK) {                        //payload has timestamp value
        String payload = http.getString();
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(payload);
        //Checking if the JSON was parsed perfectly
        if (!root.success())
        {
          http.end();
          return;
        }
        else
        {
          http.end();
          //Organiztion to connect to for all communication
          epoch = root["time"].as<unsigned long>();
          timeNTPstart = epoch;
        }
      }
    }
  }
}

/* Handle root or redirect to captive portal */
void handleRoot() {
  if (captivePortal()) { // If caprive portal redirect instead of displaying the page.
    return;
  }

  server.sendContent(
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<title>\"Airowl Configuration \"</title>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "</head>"

    "<body>"
    "<div style=\"position: relative; width:auto\">"
    " <img src=\"data:image/png;base64,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\" style=\"width:25%; position:relative; margin-left: auto;"
    "margin-right: auto;display: block;\"></img>"
    "<div id = \"header\" style=\"  -webkit-box-shadow: 0 3px 6px 0 rgba(0,0,0,0.33);opacity:0.9; background-color: #fca915; height: 8%; padding-top: 0.1% \">"
    "<h1 style=\"color:#fff ;text-align:center; background-color: #fca915; font-size: 200% ; \"><b> Airowl Setup</b></h1>"
  );

  server.sendContent(
    "</div>"
    "<div id=\"content\">"
  );
  if (server.client().localIP() == apIP) {
    server.sendContent(String("<p style=\"text-align: center; font-size: medium\"> You are connected through soft AP: <p style=\"text-align: center; font-size: large\">") + oz_deviceId + "</p>");

  } else {
    server.sendContent(String("<p style=\"text-align: center; font-size:large\"> You are connected through soft AP:</p>"
                              "<div style=\"text-align: center\"><span style=\"font-size: x-small>") + ssid + "<span></div>");
  }

  server.sendContent(
    "</div>"
    "<br>"
    "<div style=\"width:75%; margin:0 auto; text-align: center;\">"
    "<a href ='/wifi' align=\"center\">"
    "<button type=\"submit\" style=\"position:relative;  background-color: #fca915; color: white;"
    "padding: 0.4em 1em; text-align:center; display: inline-block; cursor: pointer;font-size: 25px;\" value="">"
    "WiFi Config"
    "</button>"
    "</a>"
    "</div>"
    "</div>"

    "</body>"
    "</html>"
  );
  server.client().stop(); // Stop is needed because we sent no content length
}

/* Redirect to captive portal if we got a request for another domain.
  Return true in that case so the page handler do not try to handle the request again. */
boolean captivePortal() {
  if (!isIp(server.hostHeader()) && server.hostHeader() != (String(myHostname) + ".local")) {
    //Serial.println("Request redirected to captive portal");
    server.sendHeader("Location", String("http://") + toStringIp(server.client().localIP()), true);
    server.send ( 302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
    server.client().stop(); // Stop is needed because we sent no content length
    return true;
  }
  return false;
}

/* Wifi config page handler */
void handleWifi() {

  server.sendContent(
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<title>\" Airowl Configuration \"</title>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "</head>"

    "<body>"
    "<div style=\"position: relative; width:auto\">"
    " <img src=\"data:image/png;base64,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\" style=\"width:25%; position:relative; margin-left: auto;"
    "margin-right: auto;display: block;\"></img>"
    "<div id = \"header\" style=\"  -webkit-box-shadow: 0 3px 6px 0 rgba(0,0,0,0.33);opacity:0.9; background-color: #fca915; height: 8%; padding-top: 0.1% \">"
    "<h1 style=\"color:#fff ;text-align:center; background-color: #fca915; font-size: 200% ; \"><b> Airowl Setup</b></h1>"
  );
  server.sendContent(
    "</div>"
    "<div id=\"content\">"
  );
  if (server.client().localIP() == apIP) {
    //           server.sendContent(String("<p style=\"text-align: center; font-size:large\"> You are connected through soft AP :</p>"
    //            "<div style=\"text-align: center\"><span style=\"font-size: x-small>") + oz_deviceId + "<span></div>");
    server.sendContent(String("<p style=\"text-align: center; font-size: medium\"> You are connected through soft AP: <p style=\"text-align: center; font-size: large\">") + oz_deviceId + "</p>");
  } else {
    server.sendContent(String("<p style=\"text-align: center; font-size:large\"> You are connected through soft AP:</p>"
                              "<div style=\"text-align: center\"><span style=\"font-size: x-small>") + ssid + "<span></div>");
  }

  //Scanning number of wifi netwroks availble
  int n = WiFi.scanNetworks();


  server.sendContent(
    "<div>"
    "<form method='POST' action='wifisave'>"
    "<div id=\"list\" align=\"center\">"
    "<h2>Wifi Configuration List</h2>"
    "<div id=\"selectssid\" align =\"center\" style=\"overflow: hidden;height: 40%;width: 75%; margin:0 auto;padding:0.1% 0.1% 0.1% 0.1%; -moz-border-radius: 1% 1% 1% 1%;-webkit-border-radius: 1% 1% 1% 1%;"
    " border-radius: 5% 5% 5% 5%; box-shadow: 1px 1px 11px #fca915;\">"

    "<select name='n' align=\"center\" style=\"height: 100%; width:100%; padding:5px 10px; text-align:center; font-weight: bold; font-size:large\">"
    "<option >Select </option>"

  );
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      server.sendContent("<option >");
      server.sendContent(String() + "\r\n<tr><td>" + WiFi.SSID(i)  + "</td></tr>");
      server.sendContent("</option>");
    }
  } else {
    server.sendContent(String() + "<tr><td>No WLAN found</td></tr>");
  }
  server.sendContent(         "</select>"
                              "</div>"
                              "</div>"
                              "<div id=\"right\" align=\"center\">"
                              "<form action=\"#\" class=\"sign-form\">"
                              "<div class=\"row\"><p><h2> Connect to Network: </h2></p>"
                              "<input name='p' id=\"show\" style=\"display:inline-block;vertical-align:top; padding:4% 3% 4% 3%;\" type=\"password\" placeholder=\"Password\" required/><br><input type=\"checkbox\" value=\"show\" id=\"eye\" style=\"width:4%; zoom:1; cursor:pointer  \">Show Password"

                              "</div>"
                              "<br>"
                              "<div>"
                              "<input type = \"submit\"style=\"position:relative;  background-color: #fca915; color: white;"
                              " padding: 0.4em 1em; text-align:center; display: inline-block; cursor: pointer;font-size: 25px;\" value=\"Connect\">"
                              "</div>"
                              "</form>"

                              "</div>"
                              "</div>"
                              "<script>"

                              "document.getElementById(\"eye\").addEventListener(\"click\", function(e){"
                              "var show = document.getElementById(\"show\");"
                              "if(show.getAttribute(\"type\")==\"password\"){"
                              "show.setAttribute(\"type\",\"text\");"
                              " } "
                              "else {"
                              "show.setAttribute(\"type\",\"password\");"
                              "}"
                              "});"

                              "</script>"
                              "</body>"
                              "</html>"
                    );

  server.client().stop(); // Stop is needed because we sent no content length
}

/* Handle the WLAN save form and redirect to WLAN config page again */
void handleWifiSave() {
  server.sendContent(
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<title>\" Airowl Configuration \"</title>"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
    "</head>"

    "<body>"
    "<div style=\"position: relative; width:auto\">"
    " <img src=\"data:image/png;base64,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\" style=\"width:25%; position:relative; margin-left: auto;"
    "margin-right: auto;display: block;\"></img>"
    "<div id = \"header\" style=\"  -webkit-box-shadow: 0 3px 6px 0 rgba(0,0,0,0.33);opacity:0.9; background-color: #fca915; height: 8%; padding-top: 0.1% \">"
    "<h1 style=\"color:#fff ;text-align:center; background-color: #fca915; font-size: 200% ; \"><b> Airowl Setup</b></h1>"
    "<p style=\"text-align:center; color:#fff\"> Wait for lights to turn green... </p>"
  );
  ssid = server.arg("n");
  password = server.arg("p");
  flag = 1;
  server.sendHeader("Location", "wifi", true);
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  delay(5000);
  server.client().stop(); // Stop is needed because we sent no content length

  if (ssid.length() > 0)
  {
    retAp = 1;
    init_flag = 1;
    ledColor(0, 255, 255);
  }
}

void handleNotFound() {
  if (captivePortal()) { // If caprive portal redirect instead of displaying the error page.
    return;
  }
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }
  //  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  //  server.sendHeader("Pragma", "no-cache");
  //  server.sendHeader("Expires", "-1");
  server.send ( 404, "text/plain", message );
}

/*Is this an IP? */
boolean isIp(String str) {
  for (int i = 0; i < str.length(); i++) {
    int c = str.charAt(i);
    if (c != '.' && (c < '0' || c > '9')) {
      return false;
    }
  }
  return true;
}

/* IP to String? */
String toStringIp(IPAddress ip) {
  String res = "";
  for (int i = 0; i < 3; i++) {
    res += String((ip >> (8 * i)) & 0xFF) + ".";
  }
  res += String(((ip >> 8 * 3)) & 0xFF);
  return res;
}

void Eeprom_store(void)
{
  int f = 0, v = 0;
  total_ssid = EEPROM.read(0);
  if (total_ssid > (set_limit - 1))
  {

    delete_prevssid();

  }

  if (total_ssid == 0)
  {
    address = 7;
    EEPROM.write(2, 7);

  }
  else
  {
    address = EEPROM.read(1);
    val = total_ssid + 2;
    EEPROM.write(val, address);
  }
  while (ssid.charAt(f) != '\0')
  {
    EEPROM.write(address, ssid.charAt(f));
    address++;
    f++;
  }
  EEPROM.write(address, '\0');
  address++;
  f = 0;
  while (password.charAt(f) != '\0')
  {
    EEPROM.write(address, password.charAt(f));
    address++;
    f++;
  }
  EEPROM.write(address, '\0');
  address++;
  EEPROM.write(1, address);
  total_ssid++;
  EEPROM.write(0, total_ssid);
  EEPROM.commit();
}

void wifi_check()//checks availability of presence of wifi stored in eeprom
{
  
  s = EEPROM.read(0) + 1;
  Serial.println(s);
  while ((s >= 2) && (EEPROM.read(0)!=0))
  {
    store_address = EEPROM.read(s);
   
    j = 0;
    k = 0;
    while (EEPROM.read(store_address) != '\0')
    {
      wifissid[j] = EEPROM.read(store_address);
      store_address++;
      j++;
      
    }
    wifissid[j] = '\0';
    store_address++;
    while (EEPROM.read(store_address) != '\0')
    {
      wifipswd[k] = EEPROM.read(store_address);
      store_address++;
      k++;
    }
    wifipswd[k] = '\0';
    store_address++;
    s--;
    String ssid3(wifissid), pswd3(wifipswd);
    ssid1 = ssid3;
    pswd2 = pswd3;
    Serial.println(ssid1);
    Serial.println(pswd2);
    WiFi.begin(ssid1.c_str(), pswd2.c_str());
    timestart = millis();
    while ((WiFi.status() != WL_CONNECTED) && millis() - timestart < 10000)
    {
      delay(500);
      
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      
      ap_flag = 0;
      break;
    }
  }
}

void delete_prevssid()          //deletes previously used wifi when the number of wifi stored increases then set limit
{
  for ( int q = 0; q < (EEPROM.read(0) - 1); q++)
  {
    cpy_srt = EEPROM.read(q + 3);
    int counter1 = 0;
    counter1 = cpy_srt;                 //counte1 counts the number of characters to be deleted from i.e counts characters in previously used ssid
    while (EEPROM.read(counter1) != '\0')
    {
      counter1++;
    }
    counter1++;
    while (EEPROM.read(counter1) != '\0')
    {
      counter1++;
    }
    counter1++;
    cpy_end = counter1;
    for (int h = cpy_srt; h <= cpy_end; h++)
    {
      EEPROM.write(addr, EEPROM.read(h));
      addr++;
    }
    addr = addr - 1;
    
  }
  for (int v = addr; v <= (addr + 50); v++)
  {
    EEPROM.write(v, 0);
  }
  EEPROM.write(1, addr);
  total_ssid = total_ssid - 1;
  EEPROM.write(0, total_ssid);

  EEPROM.commit();
}

void call_ap()
{
  init_flag = 1;
  extra = 1;

}
