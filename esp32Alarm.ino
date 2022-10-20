#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN 4
const int motionSensor = 16; // PIR Motion Sensor

const char* ssid = "SSID";
const char* password = "PASSWORD";

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

// Initialize Telegram BOT
String BOTtoken = "XXXXXXXXXX:XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";  //Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
String CHAT_ID = "XXXXXXXXXX";

bool sendPhoto = false;
bool showStats = false;
bool flashState = LOW;
bool motionDetected = false;
bool disableNotifications = false;
bool restart = false;

int disableNotificationsTimer = 0;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

//Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;

void IRAM_ATTR detectsMovement(void * arg) {
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

void configInitCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 1;  //10 0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 1;  //12 0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  //  s->set_brightness(s, 0);     // -2 to 2
  //  s->set_contrast(s, 0);       // -2 to 2
  //  s->set_saturation(s, 0);     // -2 to 2
  //  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  //  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  //  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  //  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  //  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  //  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  //  s->set_ae_level(s, 0);       // -2 to 2
  //  s->set_aec_value(s, 300);    // 0 to 1200
  //  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  //  s->set_agc_gain(s, 0);       // 0 to 30
  //  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  //  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  //  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  //  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  //  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  //  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  //  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  //  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  //  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

}

void handleNewMessages(int numNewMessages) {
  //Serial.print("Handle New Messages: ");
  //Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    //Serial.println(text);


    String from_name = bot.messages[i].from_name;
    if (text == "/start" || text == "/help") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      welcome += "/stats : shows flash, and notifications state \n";
      welcome += "/notifications : toggles notifications. Add _X to disable for X minutes \n";
      welcome += "/restart : restarts the device \n";
      welcome += "/reset : sets everything back to default \n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    else if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      //Serial.println("Changed flash LED state");
      if (flashState) {
        bot.sendMessage(CHAT_ID, "Flash enabled", "");
      }
      else {
        bot.sendMessage(CHAT_ID, "Flash disabled", "");
      }
    }
    else if (text == "/photo") {
      //takes a photo
      sendPhoto = true;
      //Serial.println("New photo request");
    }
    else if (text == "/stats") {
      //camera status(on/off), flash status(on/off)
      showStats = true;
      //Serial.println("Show stats");
    }
    else if (text == "/reset") {
      sendPhoto = false;
      showStats = false;
      flashState = LOW;
      digitalWrite(FLASH_LED_PIN, flashState);
      motionDetected = false;
      disableNotifications = false;
      restart = false;
      disableNotificationsTimer = 0;
      bot.sendMessage(CHAT_ID, "Reset success", "");
    }
    else if (text == "/restart") {
      restart = true;
    }
    else if (text.indexOf("/notifications") >= 0) {
      //disables/enables notifications for an amount of time or until enabled again
      if (disableNotifications) {
        //notifications are disabled
        int length = text.length();
        String time = text.substring(14, length);
        disableNotificationsTimer = time.toInt() * 60;
        if (disableNotificationsTimer == 0) {
          //no time was given so i want to enable notifications
          disableNotifications = false;
          disableNotificationsTimer = 0;
          bot.sendMessage(CHAT_ID, "Enabled notifications", "");
          //Serial.println("Enabled notifications");
        }
        else {
          //time was given so i want to disable the notifications for the new time
          disableNotifications = true;
          bot.sendMessage(CHAT_ID, "New time given. Disabling notifications for aproximatelly " + String(disableNotificationsTimer / 60) + " minutes", "");
          //Serial.println("New time given. Disabling notifications for aproximatelly " + String(disableNotificationsTimer/60) + " minutes");
        }
      }
      else {
        //notifications are enabled so we want to disable them and we will also check if time was given
        int length = text.length();
        String time = text.substring(14, length);
        disableNotificationsTimer = time.toInt() * 60;
        if (disableNotificationsTimer == 0) {
          bot.sendMessage(CHAT_ID, "No time given. Disabling for 10 minutes", "");
          //Serial.println("No time given. Disabling for 10 minutes");
          disableNotificationsTimer = 10 * 60;
          disableNotifications = true;
        }
        else {
          disableNotifications = true;
          bot.sendMessage(CHAT_ID, "Disabling notifications for aproximatelly " + String(disableNotificationsTimer / 60) + " minutes", "");
          //Serial.println("Disabling notifications for aproximatelly" + String(disableNotificationsTimer/60) + " minutes");
        }
      }
    }
    else {
      bot.sendMessage(CHAT_ID, "Message not recognized. Send '/help' to get available commands", "");
    }
  }
}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    bot.sendMessage(CHAT_ID, "Camera capture failed. Restarting device", "");
    //Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  //Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    //Serial.println("Connection successful");

    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      //Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    //Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    //Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

String printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    bot.sendMessage(CHAT_ID, "Failed to obtain time", "");
    return "ERROR";
  }
  char timeHour[3];
  char timeMinute[3];
  char timeSecond[3];

  char timeWeekDay[10];
  char timeDay[3];
  char timeMonth[10];
  char timeYear[5];

  strftime(timeHour, 3, "%H", &timeinfo);
  strftime(timeMinute, 3, "%M", &timeinfo);
  strftime(timeSecond, 3, "%S", &timeinfo);

  strftime(timeWeekDay, 10, "%A", &timeinfo);
  strftime(timeDay, 3, "%d", &timeinfo);
  strftime(timeMonth, 10, "%B", &timeinfo);
  strftime(timeYear, 5, "%Y", &timeinfo);

  return String(timeWeekDay) + " " + String(timeDay) + " " + String(timeMonth) + " " + String(timeYear) + " - " + String(timeHour) + ":" + String(timeMinute) + ":" + String(timeSecond);
}

void setup() {
  Serial.begin(115200);

  esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);
  if (err != ESP_OK) {
    Serial.printf("fail with error 0x%x \r\n", err);
  }

  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);
  if (err != ESP_OK) {
    Serial.printf("handler add failed with error 0x%x \r\n", err);
  }

  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);

  // Config and init the camera
  configInitCamera();

  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  //Serial.println();
  //Serial.print("Connecting to ");
  //Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.print(".");
    delay(500);
  }
  //Serial.println();
  //Serial.print("ESP32-CAM IP Address: ");
  //Serial.println(WiFi.localIP());

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  bot.sendMessage(CHAT_ID, "Bot started up", "");
}

void loop() {

  if (motionDetected) {
    if (!disableNotifications) {
      while (!timeClient.update()) {
        timeClient.forceUpdate();
      }
      bot.sendMessage(CHAT_ID, "Motion at: " + printLocalTime(), "");
      sendPhotoTelegram();
      //Serial.println("Motion Detected");
      motionDetected = false;
    }
    else {
      //Serial.println("Motion Detected but ignoring");
      motionDetected = false;
    }
  }

  if (sendPhoto) {
    //Serial.println("Preparing photo");
    sendPhotoTelegram();
    sendPhoto = false;
  }
  if (showStats) {
    showStats = false;
    String stats;
    if (String(flashState) == "0")
    {
      stats += "Flash: disabled\n";
    }
    else
    {
      stats += "Flash:  enabled\n";
    }
    if (String(disableNotifications) == "0")
    {
      stats += "Notifications: enabled\n";
    }
    else
    {
      stats += "Notifications: disabled for ~ " + String(disableNotificationsTimer / 60) + " minutes \n";
    }
    bot.sendMessage(CHAT_ID, stats, "");
  }
  if (restart) {
    restart = false;
    bot.sendMessage(CHAT_ID, "Restarting ESP", "");
    //Serial.println("Restart ESP");
    ESP.restart();
  }
  if (millis() > lastTimeBotRan + botRequestDelay)  {
    if (disableNotifications) {
      disableNotificationsTimer = disableNotificationsTimer - 4;
      //Serial.println("Notifications will be of for " + String(disableNotificationsTimer) + " more seconds");
      if (disableNotificationsTimer <= 0) {
        disableNotifications = false;
        Serial.println("Notifications enabled again");
      }
    }
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      //Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}
