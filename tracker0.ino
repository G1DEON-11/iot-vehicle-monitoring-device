#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <LittleFS.h>
#include "driver/twai.h" // ESP32 Internal TWAI CAN Library

// === GPS Pins ===
#define GPS_RXD 9  
#define GPS_TXD 10   
#define GPS_BAUD 9600

// === 4G Modem Pins ===
#define MODEM_RXD 12 
#define MODEM_TXD 13 
#define MODEM_RST 11 
#define MODEM_BAUD 115200

// === TCAN334 CAN Pins ===
#define CAN_TX_PIN   GPIO_NUM_14  
#define CAN_RX_PIN   GPIO_NUM_48  
#define CAN_STB_PIN  21           
#define CAN_SHDN_PIN 47           

// === AWS INFLUXDB 2 CONFIGURATION ===
const String INFLUX_SERVER_IP = "98.92.64.181"; 
const String INFLUX_PORT      = "8086";
const String INFLUX_ORG       = "MUST";         
const String INFLUX_BUCKET    = "GPS_Data";     
const String INFLUX_TOKEN     = "IHpytYc9DeDZ4I_RAfhBZ_QN2joPzh6QRIPIJ-D352irYoGt0gWIDpwxsqeC5QCrDE-PmfwAIFhtQYEk7adbew=="; 

const String DEVICE_ID        = "tracker01";       
const char* BUFFER_FILE       = "/offline_gps.txt"; 

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
HardwareSerial modemSerial(2);

// FreeRTOS Synchronization Tools
SemaphoreHandle_t gpsMutex;      
SemaphoreHandle_t serialMutex;   
String globalLineProtocolPayload = "";  

// Global Vehicle Data Variables (OBD2)
int obd_rpm = 0;
int obd_speed = 0;
int obd_temp = 0;
int obd_throttle = 0;

byte pidList[] = {0x0C, 0x0D, 0x05, 0x11}; 
int listIndex = 0;
unsigned long lastPidTime = 0;
const unsigned long PID_INTERVAL = 1000; 

void safePrintln(String msg) {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    Serial.println(msg);
    xSemaphoreGive(serialMutex);
  }
}

String readModemResponse(int timeoutMs) {
  unsigned long start = millis();
  String response = "";
  while (millis() - start < timeoutMs) {
    while (modemSerial.available() > 0) {
      char c = modemSerial.read();
      response += c;
    }
  }
  return response;
}

bool sendATCommand(String cmd, String expectedResponse, int timeoutMs) {
  modemSerial.println(cmd);
  String resp = readModemResponse(timeoutMs);
  if (resp.indexOf(expectedResponse) != -1) {
    return true;
  }
  return false;
}

// Function to send OBD2 PIDs over CAN Bus
void sendObdRequest(byte pid) {
  twai_message_t tx_msg;
  tx_msg.identifier = 0x7DF;       
  tx_msg.extd = 0;                 
  tx_msg.rtr = 0;                  
  tx_msg.data_length_code = 8;     
  
  tx_msg.data[0] = 0x02;           
  tx_msg.data[1] = 0x01;           
  tx_msg.data[2] = pid;            
  for(int i=3; i<8; i++) tx_msg.data[i] = 0x00;

  twai_transmit(&tx_msg, pdMS_TO_TICKS(50));
}

void saveToOfflineBuffer(String data) {
  File file = LittleFS.open(BUFFER_FILE, FILE_APPEND);
  if (!file) return;
  file.println(data); 
  file.close();
  safePrintln("[💾 LittleFS] Connection lost. Data appended to offline buffer file.");
}

bool postToInfluxDB(String lineProtocol) {
  String openCmd = "AT+CIPSTART=\"TCP\",\"" + INFLUX_SERVER_IP + "\"," + INFLUX_PORT;
  modemSerial.println(openCmd);
  String connectResp = readModemResponse(3000);

  if (connectResp.indexOf("CONNECT OK") != -1) {
    String httpRequest = "POST /api/v2/write?org=" + INFLUX_ORG + "&bucket=" + INFLUX_BUCKET + "&precision=s HTTP/1.1\r\n"
                       + "Host: " + INFLUX_SERVER_IP + ":" + INFLUX_PORT + "\r\n"
                       + "Authorization: Token " + INFLUX_TOKEN + "\r\n"
                       + "Content-Type: text/plain; charset=utf-8\r\n"
                       + "Content-Length: " + String(lineProtocol.length()) + "\r\n"
                       + "Connection: close\r\n\r\n" 
                       + lineProtocol;
                       
    modemSerial.print("AT+CIPSEND=");
    modemSerial.println(httpRequest.length());
    vTaskDelay(pdMS_TO_TICKS(200));
    
    modemSerial.print(httpRequest);
    String httpResp = readModemResponse(3000); 
    sendATCommand("AT+CIPCLOSE", "OK", 200);

    if (httpResp.indexOf("204 No Content") != -1 || httpResp.indexOf("200 OK") != -1) {
      return true;
    }
  }
  sendATCommand("AT+CIPCLOSE", "OK", 100);
  return false;
}

void checkAndUploadOfflineData() {
  if (!LittleFS.exists(BUFFER_FILE)) return;
  File file = LittleFS.open(BUFFER_FILE, FILE_READ);
  if (!file) return;

  safePrintln("\n[🔄 LittleFS] Found offline buffer. Recovering and uploading data to InfluxDB...");
  
  modemSerial.println("AT+CIPSTART=\"TCP\",\"" + INFLUX_SERVER_IP + "\"," + INFLUX_PORT);
  String testResp = readModemResponse(2000);
  sendATCommand("AT+CIPCLOSE", "OK", 100);

  if (testResp.indexOf("CONNECT OK") == -1) {
    safePrintln("[❌ LittleFS] Remote server is unreachable. Offline dataset retained.");
    file.close();
    return;
  }

  bool uploadSuccess = true;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 10) {
      safePrintln("[🚀 Flush Record]: " + line);
      if (!postToInfluxDB(line)) {
        uploadSuccess = false;
        break; 
      }
      vTaskDelay(pdMS_TO_TICKS(400)); 
    }
  }
  file.close();

  if (uploadSuccess) {
    LittleFS.remove(BUFFER_FILE);
    safePrintln("[🧹 LittleFS] All cached data synchronised successfully. Buffer cleared!\n");
  }
}

void init_4g_modem() {
  safePrintln("[4G] Initialising and waking up LTE module...");
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH); vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(MODEM_RST, LOW); vTaskDelay(pdMS_TO_TICKS(1500));
  pinMode(MODEM_RST, INPUT);     
  vTaskDelay(pdMS_TO_TICKS(4000)); 

  sendATCommand("AT", "OK", 500);
  sendATCommand("AT+CSQ", "OK", 1000); 

  safePrintln("\n[SYSTEM] Checking SIM card status and scanning cellular network...");
  bool hasNetwork = false;
  String currentAPN = "internet"; 
  int attempts = 0;

  while (!hasNetwork && attempts < 20) {
    attempts++;
    modemSerial.println("AT+COPS?"); 
    vTaskDelay(pdMS_TO_TICKS(1500));
    String copsResp = readModemResponse(2000); 

    if (copsResp.indexOf("42801") != -1 || copsResp.indexOf("MobiCom") != -1 || copsResp.indexOf("42899") != -1) {
      currentAPN = "internet"; hasNetwork = true;
    } else if (copsResp.indexOf("42802") != -1 || copsResp.indexOf("Unitel") != -1) {
      currentAPN = "net"; hasNetwork = true;
    } else if (copsResp.indexOf("42800") != -1 || copsResp.indexOf("Skytel") != -1) {
      currentAPN = "internet"; hasNetwork = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (hasNetwork) {
    safePrintln("[AUTO-APN] Matching network provider. Configuring APN to: '" + currentAPN + "'");
    sendATCommand("AT+CGDCONT=1,\"IP\",\"" + currentAPN + "\"", "OK", 1000);
    sendATCommand("AT+CGACT=1,1", "OK", 3000); 
    sendATCommand("AT+CGPADDR=1", "OK", 1000);
    safePrintln("[SUCCESS] Cellular GPRS data context activated ready for HTTP!\n");
  } else {
    safePrintln("\n[FATAL ERROR] Cell network connection timeout! Device running offline.");
  }
}

// === Core 0 Task: Process GNSS NMEA Stream & Read OBD2 Parameters ===
void vGPSTask(void *pvParameters) {
  (void) pvParameters;
  unsigned long lastLogTime = 0;

  for (;;) {
    // 1. Process GPS UART Stream
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // 2. Transmit OBD2 PID Requests at 1-second Interval
    if (millis() - lastPidTime >= PID_INTERVAL) {
      lastPidTime = millis();
      sendObdRequest(pidList[listIndex]);
      listIndex++;
      if (listIndex >= 4) listIndex = 0;
    }

    // 3. Receive Vehicle CAN Bus Frames via TWAI Controller
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(1)) == ESP_OK) {
      if (rx_msg.identifier == 0x7E8 && rx_msg.data[1] == 0x41) { 
        byte pid = rx_msg.data[2];
        switch(pid) {
          case 0x0C: obd_rpm = ((rx_msg.data[3] * 256) + rx_msg.data[4]) / 4; break;
          case 0x0D: obd_speed = rx_msg.data[3]; break;
          case 0x05: obd_temp = rx_msg.data[3] - 40; break;
          case 0x11: obd_throttle = (rx_msg.data[3] * 100) / 255; break;
        }
      }
    }

    // Diagnostics Telemetry Log output every 3 seconds
    if (millis() - lastLogTime > 3000) {
      lastLogTime = millis();
      String logMsg = "[🛰️ GPS] Satellites: " + String(gps.satellites.value());
      if (gps.location.isValid()) {
        logMsg += " | Lat: " + String(gps.location.lat(), 6) + " Lng: " + String(gps.location.lng(), 6);
      }
      logMsg += " | 🚗 [CAN] RPM: " + String(obd_rpm) + " Speed: " + String(obd_speed);
      safePrintln(logMsg);
    }

    // Formulate Time-Series InfluxDB Line Protocol Dataset
    if (gps.location.isUpdated() && gps.location.isValid()) {
      if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        globalLineProtocolPayload = "location,device_id=" + DEVICE_ID 
                                  + " lat=" + String(gps.location.lat(), 6) 
                                  + ",lng=" + String(gps.location.lng(), 6) 
                                  + ",gps_speed=" + String(gps.speed.kmph(), 1) 
                                  + ",satellites=" + String(gps.satellites.value())
                                  + ",obd_rpm=" + String(obd_rpm)
                                  + ",obd_speed=" + String(obd_speed)
                                  + ",engine_temp=" + String(obd_temp)
                                  + ",throttle=" + String(obd_throttle);
        xSemaphoreGive(gpsMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// === Core 1 Task: Handle Periodic Cloud Communications via HTTP POST ===
void vUploadTask(void *pvParameters) {
  (void) pvParameters;
  init_4g_modem();

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(10000)); // Trigger every 10 seconds

    String localData = "";

    if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      localData = globalLineProtocolPayload;
      globalLineProtocolPayload = ""; 
      xSemaphoreGive(gpsMutex);
    }

    if (localData.length() > 10) {
      safePrintln("\n🚀 [Core1 HTTP] Initiating connection to AWS InfluxDB Endpoint...");
      safePrintln("[➔ Payload Transmitted]: " + localData); 
      
      checkAndUploadOfflineData();

      if (postToInfluxDB(localData)) {
        safePrintln("[SUCCESS] Telemetry pipeline update verified by AWS Cloud Server.\n");
      } else {
        safePrintln("[❌ ERROR] Network socket failed. Redirecting packet payload to LittleFS cache.");
        saveToOfflineBuffer(localData); 
      }
    } else {
      safePrintln("\n[ℹ️ Core1 HTTP] Pipeline idle. New sensor metrics not acquired yet.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Awake and set TCAN334 Transceiver Pins to Normal Operating Mode
  pinMode(CAN_STB_PIN, OUTPUT);
  digitalWrite(CAN_STB_PIN, LOW);   
  pinMode(CAN_SHDN_PIN, OUTPUT);
  digitalWrite(CAN_SHDN_PIN, LOW);  

  gpsMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  if (!LittleFS.begin(true)) {
    Serial.println("[FATAL] LittleFS initialization aborted due to disk layer panic!");
    while (1);
  }
  Serial.println("[SUCCESS] LittleFS flash filesystem mounted successfully.");

  // Hardware Driver Configuration for ESP32 Internal TWAI Controller
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK) {
    Serial.println("[SUCCESS] Internal TWAI CAN Controller pipeline linked to TCAN334.");
  } else {
    Serial.println("[FATAL] TWAI Driver setup crashed!");
    while(1);
  }

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RXD, GPS_TXD);
  modemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RXD, MODEM_TXD);

  xTaskCreatePinnedToCore(vGPSTask, "GPSTask", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vUploadTask, "UploadTask", 8192, NULL, 1, NULL, 1);

  Serial.println("[SYSTEM] Multi-threaded GPS + OBD2 Integrated Tracker System Online.");
}

void loop() {
  vTaskDelete(NULL); 
}