#include "OneButton.h"
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TM1637Display.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <SoftwareSerial.h>
#include "driver/rtc_io.h"

TimerHandle_t sleepTimer;

uint8_t macAddr[6];
int bedengan;
int node;
int channel;
int doubleClickCount = 0; // Variabel untuk menghitung double clicks

// button
#define PIN_BUTTON1 7
#define PIN_BUTTON2 11
OneButton btn1(PIN_BUTTON1, true);
OneButton btn2(PIN_BUTTON2, true);

//indikator
#define led 15
#define led1 GPIO_NUM_1
#define led2 GPIO_NUM_2
#define led3 GPIO_NUM_3
#define ind 35
// kontrol on off daya
#define control_1 13 //VDDA
#define control_2 16 //VDD
#define control_3 14 //3v3

//pin sensor
#define pHSensor 10   
#define soilSensor 5
#define suhuSensor 3
#define battSensor 4
// mult
#define mult 8

//NPK Sensor
#define RE 38
#define DE 36
const byte code[] = {0x01, 0x03, 0x00, 0x1e, 0x00, 0x03, 0x65, 0xCD};
byte valuesBytes[11];
SoftwareSerial mod(40, 34);

//tm1637 seven segment
#define CLK 17 // Pin untuk clock
#define DIO 21  // Pin untuk data I/O
TM1637Display display(CLK, DIO);

//state global
bool peerAdded;
bool readySend = false;
bool pairing = false;
bool editingMode = false; // Menandakan apakah dalam mode pengeditan
bool readyToSave = false; // Siap untuk menyimpan jika true
int editingX = -1;

// perulangan sample
const int numSample = 100;
int adcValue[numSample];

//timer mode tidur
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
#define minutes 2

//RTOS
TaskHandle_t sendDataTaskHandle = NULL;
TaskHandle_t handleBacaSuhuTask = NULL;
const uint32_t SENSOR_BATTERY_READY = 0x01; // Battery ready flag
const uint32_t SENSOR_TEMPERATURE_READY = 0x02; // Temperature ready flag
const uint32_t SENSOR_SOILPH_READY = 0x04; // soil and pH ready flag
const uint32_t ALL_SENSORS_READY = SENSOR_BATTERY_READY | SENSOR_TEMPERATURE_READY | SENSOR_SOILPH_READY; // All sensors ready

// const uint32_t SENSOR_SOIL_READY = 0x04; // Soil ready flag
OneWire oneWire(suhuSensor);
DallasTemperature sensors(&oneWire);

// message struct
typedef struct struct_message {
  uint8_t msgType;
  char id[6];
  int humidity;
  float temperature;
  float pH;
  int N;
  int P;
  int K;
  float baterai;
} struct_message;
struct_message myData;
typedef struct struct_pairing {       // new structure for pairing
  uint8_t msgType;
  char id[6];
  uint8_t macAddr[6];
  uint8_t channel;
} struct_pairing;
struct_pairing pairingData;

enum MessageType {PAIRING, DATA,};
void deleteMAC(){
  for (int j = 0; j < 6; ++j) {
    EEPROM.write(j, 0);
  }
  EEPROM.commit();
}

/** scan WIFI **/
void ScanWifi() {
  Serial.println("Memindai jaringan WiFi...");
  int n = WiFi.scanNetworks();
  Serial.println("Pemindaian selesai");
  if (n == 0) {
    Serial.println("Tidak ada jaringan ditemukan");
  } else {
    for (int i = 0; i < n; ++i) {
      // Cek SSID untuk menemukan Gateway
      if (WiFi.SSID(i) == "Master_GATEWAY") { // Ganti "NamaSSIDGateway" dengan SSID Gateway Anda
        Serial.print("Gateway ditemukan: ");
        Serial.print(WiFi.SSID(i));
        Serial.print(" dengan BSSID: ");

        // Dapatkan alamat MAC dari Gateway
        const uint8_t* BSSID = WiFi.BSSID(i);
        // Mencetak alamat MAC
        for (int j = 0; j < 6; ++j) {
          if (BSSID[j] < 16) {
            Serial.print("0");
          }
          Serial.print(BSSID[j], HEX);
          if (j < 5) Serial.print(":");
        }

        // Simpan alamat MAC dan channel ke EEPROM
        for (int j = 0; j < 6; ++j) {
          EEPROM.write(j, BSSID[j]);
        }
        EEPROM.commit();
        EEPROM.write(7, WiFi.channel(i));
        EEPROM.commit();

        // Mencetak channel WiFi
        Serial.print(", Channel: ");
        Serial.println(WiFi.channel(i));

        Serial.println("Alamat MAC Gateway dan Channel disimpan ke EEPROM");
        channel = EEPROM.read(7);
        pairingRequest();
        break;
      }
      delay(10);
    }
  }
}

/** cek Penyimpanan MAC adress target **/
bool isMACStored() {
  uint8_t storedMAC[6];
  for (int i = 0; i < 6; i++) {
    storedMAC[i] = EEPROM.read(i);
    if (storedMAC[i] != 0xFF && storedMAC[i] != 0x00) { // Ganti 0xFF dan 0x00 dengan nilai default EEPROM Anda jika berbeda
      return true; // Ada setidaknya satu byte yang tidak 0xFF atau 0x00, anggap MAC tersimpan
    }
  }
  return false; // Semua byte 0xFF atau 0x00, anggap tidak ada MAC yang tersimpan
}
void loadGatewayMAC(uint8_t *macAddr) {
  for (int i = 0; i < 6; i++) {
    macAddr[i] = EEPROM.read(i);
  }
}

/** Save Pair ESPNOW **/
void addPeer(const uint8_t * mac_addr, uint8_t chan){
  esp_now_peer_info_t peer;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

/** Delete Pair ESPNOW **/
void deletePeer(const uint8_t * addr){
  esp_err_t delStatus = esp_now_del_peer(addr);
  if (delStatus == ESP_OK) {
    peerAdded = false;
    digitalWrite(ind, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(ind, LOW);    
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } 
}

/** Menyimpan ID Node **/
void save_id(const char* id) {
    int addr = 10; // Mulai dari alamat 10 di EEPROM
    for (int i = 0; i < strlen(id); i++) {
        EEPROM.write(addr++, id[i]); // Menulis karakter per karakter
    }
    EEPROM.write(addr, '\0'); // Menambahkan null terminator untuk menandai akhir string
    EEPROM.commit(); // Pastikan untuk melakukan commit agar perubahan disimpan
}
void read_id(char* id, size_t max_len) {
    if (id == NULL || max_len == 0) return; // Pastikan pointer valid dan ada ruang

    int addr = 10; // Mulai membaca dari alamat 10 di EEPROM
    char ch = EEPROM.read(addr++);
    if (ch == 0xFF) {
        // EEPROM kosong, isi dengan "0.0"
        snprintf(id, max_len, "0.0");
        return;
    }

    // Jika EEPROM tidak kosong, baca nilai yang ada
    size_t i = 0;
    while (ch != '\0' && i < max_len - 1) { // Baca sampai null terminator atau batas buffer
        id[i++] = ch;
        ch = EEPROM.read(addr++);
    }
    id[i] = '\0'; // Pastikan string diakhiri dengan null terminator
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

/** Callback Send Data ESPNOW **/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Kirim Data ke: ");
  printMAC(mac_addr);
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // mengulang hubungan pairing
  if(status != 0 && pairingData.msgType == PAIRING){
    pairingRequest(); 
  }

}

/** Callback Send Data ESPNOW **/
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len){
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type){
  case PAIRING:
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (strcmp(pairingData.id, "0") == 0){
      printMAC(mac_addr);
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      pairing = true;
      peerAdded = true;  
      digitalWrite(ind, HIGH); // Asumsi 'led' telah didefinisikan sebagai pin output
      vTaskDelay(pdMS_TO_TICKS(500));
      digitalWrite(ind, LOW);
      //reset timer
      xTimerReset(sleepTimer, 0);
    } else{
      pairFailed();
    }
    break;

  case DATA:
    goToSleep();
  }
}

/** ESP masuk ke mode tidur **/
void goToSleep() {
  //memastikan daya dari semua sensor benar benar mati
    digitalWrite(control_1, LOW);
    digitalWrite(control_2, LOW);
    digitalWrite(control_3, LOW);
  //mengatur powerDomain agar tetap mati selama mode tidur
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF); // Mematikan RTC peripherals
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF); // Mematikan RTC slow memory
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF); // Mematikan RTC fast memory
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    esp_sleep_enable_timer_wakeup(minutes * TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds"); 
    Serial.println("Going to sleep now");
    Serial.flush(); 
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
}

/** Menambahkan hubungan pairing ke Gateway **/
void pairingRequest(){
  // setting channel dan target
  channel = EEPROM.read(7);
  loadGatewayMAC(macAddr);
  channel = EEPROM.read(7);
  Serial.print("Pairing request on channel "  );
  Serial.println(channel);

  // set WiFi channel   
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_500K);

  // set callback routines
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // set pairing data to send to the server
  pairingData.msgType = PAIRING;
  strncpy(pairingData.id, myData.id, sizeof(pairingData.id) - 1);
  WiFi.macAddress(pairingData.macAddr);
  pairingData.id[sizeof(pairingData.id) - 1] = '\0'; // Pastikan ada null-terminator
  pairingData.channel = channel;
  // add peer and send request
  addPeer(macAddr, channel);
  esp_now_send(macAddr, (uint8_t *) &pairingData, sizeof(pairingData));
}

void pairFailed(){
  Serial.println("silahkan ganti ID");
  for(int i=0; i<100 ; ++i){
    digitalWrite(ind, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(ind, LOW);
    vTaskDelay(pdMS_TO_TICKS(100));   
  }
}

/** fungsi untuk menghitung tegangan baterai **/
float calculateVoltage(int adc, float i_min, float i_max, float o_min, float o_max) {
    float m = (o_max - o_min) / (i_max - i_min);
    float c = o_min - (m * i_min);
    return m * adc + c;
}

/** handle mode tidur **/
void sleepTimeout(TimerHandle_t xTimer) {
    Serial.println("Timeout: Tidak ada data yang dikirim dalam 30 detik.");
    goToSleep();
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  mod.begin(9600);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(ind, OUTPUT);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP); 
  pinMode(control_1, OUTPUT);
  pinMode(control_2, OUTPUT);
  pinMode(control_3, OUTPUT);
  pinMode(mult, OUTPUT);
  digitalWrite(control_1, HIGH);
  // ADC resolusi 10 bit
  analogReadResolution(10);
  //inisialisasi sensor suhu
  sensors.begin();
  // Inisialisasi ESP-NOW
  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //inisialisasi EEPROM
  EEPROM.begin(512);

  // Setup OneButton
  btn1.attachClick(singleClick1);
  btn1.attachDoubleClick(doubleClick1);
  btn1.attachLongPressStart(longPressStart1);
  // setup btn 2 untuk proses espnow
  btn2.attachClick(singleClick2);
  btn2.attachDoubleClick(doubleClick2);
  btn2.attachLongPressStart(longPressStart2);

  //inisialisasi freeRTOS
  xTaskCreate(
    buttonTask,     /* Task function */
    "Button Task",  /* Name of the task */
    2048,          /* Stack size */
    NULL,           /* Task input parameter */
    2,              /* Priority of the task */
    NULL            /* Task handle */
  );
  xTaskCreate(
    bacaSuhu,     /* Task function */
    "sensor suhu",  /* Name of the task */
    15000,          /* Stack size */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &handleBacaSuhuTask           /* Task handle */
  );
  xTaskCreate(
    bacaSoilpH,     /* Task function */
    "sensor soil dan pH",  /* Name of the task */
    15000,          /* Stack size */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    NULL            /* Task handle */
  );  
  xTaskCreate(
    sensorBatt,     /* Task function */
    "sensor baterai 2",  /* Name of the task */
    4096,          /* Stack size */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    NULL            /* Task handle */
  ); 
  xTaskCreate(
    segment,     /* Task function */
    "Display Seven Segment",  /* Name of the task */
    1024,          /* Stack size */
    NULL,           /* Task input parameter */
    2,              /* Priority of the task */
    NULL            /* Task handle */
  );       
  xTaskCreate(
    sendData,     /* Task function */
    "kirim Data",  /* Name of the task */
    50000,          /* Stack size */
    NULL,           /* Task input parameter */
    2,              /* Priority of the task */
    &sendDataTaskHandle            /* Task handle */
  ); 

  //Cek ID
  size_t max_len = sizeof(myData.id); 
  read_id(myData.id, max_len); 
  sscanf(myData.id, "%d.%d", &bedengan, &node);
  Serial.print("ID disimpan: ");
  Serial.println(myData.id);
  if (myData.id != NULL && isMACStored()){
    Serial.println("Memulai Pairing");
    pairing = true;
    peerAdded = true;
  }

  // timer mode aktif
  sleepTimer = xTimerCreate("SleepTimer", pdMS_TO_TICKS(30000), pdFALSE, (void *)0, sleepTimeout);
  if (sleepTimer != NULL) {
      xTimerStart(sleepTimer, 0);
  }  
}

void loop() {

}

/** Task untuk button **/
void buttonTask(void *parameter) {
  for(;;) { // Loop tak terbatas
    btn1.tick();
    btn2.tick();
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay untuk mengurangi pemakaian CPU
  }
}

void sensorBatt (void *parameter){
  for(;;){
    int sample = 20;
    int battValue[sample];
    int rataBatt = 0;
    for (int i = 0; i < sample; i++) {
        battValue[i] = analogRead(battSensor);
        rataBatt += battValue[i];
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    rataBatt /= sample;
    //float val = (rataBatt * 4.05)/778;
    float val = calculateVoltage(rataBatt, 695, 817, 3.51, 4.12);
    myData.baterai = val;
    if (val > 3.5){
      analogWrite(led1, 0);
      analogWrite(led2, 150);
    }else{
      analogWrite(led1, 150);
      analogWrite(led2, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void bacaNPK(){
  for(int i=0; i<10; i++){
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));
    if (mod.write(code, sizeof(code)) == 8)
    {
      Serial.println("Bytes sent");
      digitalWrite(DE, LOW);
      digitalWrite(RE, LOW);
      if (mod.available() > 0) {
        // Read the response into an array
        int bytesAvailable = mod.available();
        mod.readBytes(valuesBytes, bytesAvailable);

        // Print received bytes
        Serial.print("Received: ");
        for (int i = 0; i < bytesAvailable; i++) {
          Serial.print("0x");
          Serial.print(valuesBytes[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      } else {
        Serial.println("No response received");
      }
      Serial.println();
    }
    myData.N = valuesBytes[4];
    myData.P = valuesBytes[6];
    myData.K = valuesBytes[8];

    Serial.printf("Nitrogen : %d\n", myData.N);
    Serial.printf("Phos : %d\n", myData.P);
    Serial.printf("Potassium : %d\n", myData.K);
    vTaskDelay(pdMS_TO_TICKS(500));
  }  
}

/** Task Sensor Pemantauan **/
void bacaSoilpH (void *parameter) {
  int sample = 100;
  int adcValue[sample];
  vTaskDelay(pdMS_TO_TICKS(2000));
  for(;;){
    if(pairing){
      digitalWrite(mult, HIGH);
      vTaskDelay(pdMS_TO_TICKS(5000));
      float ratapH = 0, pH;
      for (int i = 0; i < sample; i++) {
          adcValue[i] = analogRead(pHSensor);
          ratapH += adcValue[i];
          vTaskDelay(pdMS_TO_TICKS(50));
      }
      ratapH /= sample;

      /** adjust for pH Calibration **/
      // pH = (-0.0092*ratapH + 9.3533); // adjust based on sensor calibration
      // pH = (-0.0233 * ratapH) + 12.698; //ganti berdasarkan validitas sensor
      pH = (-0.0092*ratapH + 9.3533); //1
      // pH = (-0.0066*ratapH + 8.9712);  //2
      // pH = (-0.0155*ratapH + 10.526);
      // pH = (-0.0139*ratapH + 8.8253); //3
      myData.pH = pH < 0 ? 0 : (pH > 14 ? 14 : pH);
      sample = 20;
      float rataSoil = 0;         // Declared before use
      float soilHumidity;         // Ensure no syntax errors above this line
      for (int i = 0; i < sample; i++) {
        adcValue[i] = analogRead(soilSensor);
        rataSoil += adcValue[i];
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      rataSoil /= sample;
      soilHumidity = map(rataSoil, 1010, 85, 0, 100);
      //myData.humidity = rataSoil;
      myData.humidity = soilHumidity < 0 ? 0 : (soilHumidity > 100 ? 100 : soilHumidity);
      digitalWrite(mult, LOW);
      // Notify bacaSuhu task to start
      xTaskNotify(handleBacaSuhuTask, 0x01, eSetBits);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void bacaSuhu(void *parameter) {
  uint32_t ulNotificationValue = 0;
  vTaskDelay(pdMS_TO_TICKS(2000));
  for (;;) {
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);
    if (ulNotificationValue & 0x01) { // Check if notification received
      digitalWrite(control_3, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50));
      sensors.requestTemperatures();
      myData.temperature = sensors.getTempCByIndex(0);
      bacaNPK();
      // Notify sendData task to process the collected data
      xTaskNotify(sendDataTaskHandle, ALL_SENSORS_READY, eSetBits);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

/** Kirim Data Pemantauan Ke GATEWAY **/
void sendData(void *parameter) {
  vTaskDelay(pdMS_TO_TICKS(2000));
    uint32_t ulNotificationValue = 0;  // Initialize outside the loop to persist state across iterations
    for (;;) {
      // Wait for any notification without clearing them automatically, so we manually control when they are cleared
      xTaskNotifyWait(0x00, 0xFFFFFFFF, &ulNotificationValue, portMAX_DELAY);

      // Log current notification state
      Serial.print("Received notification value: ");
      Serial.println(ulNotificationValue, BIN);

      // Check if both notifications have been received
      if ((ulNotificationValue & ALL_SENSORS_READY) == ALL_SENSORS_READY) {
          myData.msgType = DATA;
          digitalWrite(ind, HIGH);
          vTaskDelay(pdMS_TO_TICKS(500));
          digitalWrite(ind, LOW);   
          esp_now_send(macAddr, (uint8_t *)&myData, sizeof(myData));
          readySend = false;  // Reset readySend flag

          // Clear all notifications since we are ready to process them
          ulNotificationValue = 0;  // Reset the notification value manually
      } else {
          // If not all sensors are ready, we wait more
          Serial.println("Waiting for all sensors to be ready...");
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
}
void segment(void *parameter) {
  for (;;) {
    // Hanya memperbarui display jika ada perubahan
    if (editingMode) {
      uint8_t data[] = { 0xFF, 0xFF, 0xFF, 0xFF }; // Default semua segmen dimatikan
      display.setBrightness(0x0f);
      switch (doubleClickCount) {
        case 0:
          display.clear();
          vTaskDelay(pdMS_TO_TICKS(100));
          data[0] = display.encodeDigit(bedengan / 10);
          data[1] = display.encodeDigit(bedengan % 10);
          data[2] = display.encodeDigit(node / 10);
          data[3] = display.encodeDigit(node % 10);
          break;
        case 1: // Mengedit Bedengan
          data[0] = display.encodeDigit(bedengan / 10); // Digit pertama bedengan
          data[1] = display.encodeDigit(bedengan % 10); // Digit kedua bedengan
          data[2] = 0x00; // Matikan digit 3
          data[3] = 0x00;
          break;
        case 2: // Mengedit Node
          data[0] = 0x00;
          data[1] = 0x00;
          data[2] = display.encodeDigit(node / 10); // Digit pertama node
          data[3] = display.encodeDigit(node % 10); // Digit kedua node
          break;
        case 3: // Tampilkan semua
          data[0] = display.encodeDigit(bedengan / 10);
          data[1] = display.encodeDigit(bedengan % 10);
          data[2] = display.encodeDigit(node / 10);
          data[3] = display.encodeDigit(node % 10);
          break;
        case 4: // Semua segmen dimatikan, tidak perlu melakukan apa-apa karena data[] sudah disiapkan
          data[0] = 0x00;
          data[1] = 0X00;
          data[2] = 0x00;
          data[3] = 0x00;
          break;
      }

      // Hanya memperbarui display jika diperlukan untuk mengurangi flickering
       // Atur kecerahan maksimum
      display.setSegments(data);

      // Tambahkan delay di sini jika Anda tidak membutuhkan pembaruan display yang sangat cepat
      // Hal ini untuk menghindari pembaruan display yang terlalu sering dan menggunakan CPU secara berlebihan
      vTaskDelay(pdMS_TO_TICKS(500)); // Delay 500 ms
    } else {
      // Jika tidak dalam editing mode, tidak perlu terus memperbarui display
      // Ini bisa diadaptasi berdasarkan kebutuhan Anda
      vTaskDelay(pdMS_TO_TICKS(1000)); // Delay lebih lama jika tidak ada apa-apa untuk ditampilkan
    }
  }
}


/** fungsi untuk button 1 1x klik**/
void singleClick1() {
  if (editingMode) {
    // Logika untuk menaikkan nilai bedengan atau node
    if (editingX == 1) {
      bedengan++;
      if (bedengan > 10){
        bedengan = 0;
      }
      Serial.print("Bedengan: ");
      Serial.println(bedengan);
    } else if(editingX == 0) {
      node++;
      if (node > 10){
        node = 0;
      }
      Serial.print("Node: ");
      Serial.println(node);
    }
    readyToSave = false;
    Serial.print("ID: ");
    Serial.print(bedengan);
    Serial.print(".");
    Serial.println(node);
  }
}

/** fungsi untuk button 1 2x klik**/
void doubleClick1() {
  if (editingMode) {
    doubleClickCount++; // Tambahkan jumlah double click
    switch (doubleClickCount) {
      case 1: // Double click pertama, mulai pengeditan Node
        editingX = 1;
        Serial.println("Mengedit Bedengan");
        break;
      case 2: // Double click kedua, beralih ke pengeditan Bedengan
        editingX = 0;
        Serial.println("Mengedit Node");
        break;
      case 3: // Double click ketiga, LED berkedip sebagai indikasi ready to save
        // Implementasikan logika kedip LED di sini
        editingX=-1;
        Serial.println("Led kedip - Siap menyimpan");
        readyToSave = true;
        break;
      case 4: // Double click keempat, simpan konfigurasi
        if (readyToSave) {
          snprintf(myData.id, sizeof(myData.id), "%d.%d", bedengan, node);// simpan ke struct massage id
          save_id(myData.id);
          Serial.print("ID disimpan: ");
          Serial.println(myData.id);
          editingX=-1;
          editingMode = false; // Keluar dari mode pengeditan
          doubleClickCount = 0; // Reset double click count
          digitalWrite(ind, HIGH); // Misal, nyalakan LED sebagai indikasi keluar mode pengeditan
          vTaskDelay(pdMS_TO_TICKS(500));
          digitalWrite(ind, LOW);
          digitalWrite(control_2, LOW);
        }
        break;
      default:
        // Jika ada double click lebih dari yang diharapkan, tidak melakukan apa-apa
        break;
    }
  }
}

/** fungsi untuk button 1 hold **/
void longPressStart1() {
    editingMode = !editingMode; // Toggle editing mode
    doubleClickCount = 0;
    readyToSave = false; // Reset readyToSave
    if (editingMode) {
      digitalWrite(control_2, HIGH);
      Serial.println("Masuk mode pengeditan");
    } else if (doubleClickCount != 4) {
      digitalWrite(control_2, LOW);
      Serial.println("Keluar mode pengeditan");
    }
}

/** fungsi untuk button 2 1x klik**/
void singleClick2(){
  ScanWifi();
}

/** fungsi untuk button 2 2x klik**/
void doubleClick2(){
  if (!peerAdded){
    pairingRequest();
  }
}
/** fungsi untuk button 2 hold**/
void longPressStart2(){
  strcpy(myData.id, "0.0"); // Set ID ke nilai default atau "0.0"

  // Memutus Hubungan Komunikasi
  if(peerAdded) {
    deletePeer(macAddr); // Hanya jika sudah ada peer yang ditambahkan
  }

    // Menghapus EEPROM
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFF); // Menghapus semua data di EEPROM dengan menulis 0xFF
  }
  EEPROM.commit(); // Pastikan untuk melakukan commit agar perubahan disimpan
  digitalWrite(control_2, HIGH);
  editingMode = true;
  doubleClickCount = 0;
  size_t max_len = sizeof(myData.id);
  read_id(myData.id, max_len); 
  sscanf(myData.id, "%d.%d", &bedengan, &node);
  digitalWrite(led, LOW);
}
