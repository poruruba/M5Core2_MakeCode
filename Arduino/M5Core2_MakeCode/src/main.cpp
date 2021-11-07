#include <M5Core2.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsServer.h>
#include "quickjs_esp32.h"

const char *wifi_ssid = "【WiFiアクセスポイントのSSID】";
const char *wifi_password = "【WiFiアクセスポイントのパスワード】";

extern const char jscode_main[] asm("_binary_rom_main_js_start");

//const char *jscode_main = "console.log('Hello Default');";
const char *JS_NAME_DEFAULT = "/main.js";

ESP32QuickJS qjs;

#define WEBSOCKET_PORT 81 // WebSocketのポート番号
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

#define CODE_BUFFER_SIZE (1 * 1024)
uint8_t recv_buffer[CODE_BUFFER_SIZE];
char js_buffer[CODE_BUFFER_SIZE];
bool switch_jscode = false;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:{
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      break;
    }
    case WStype_TEXT:{
      long ret = file_write(JS_NAME_DEFAULT, payload, length);
      if (ret < 0){
        Serial.println("file_write error");
        return;
      }

      switch_jscode = true;
      break;
    }
    case WStype_ERROR:
      Serial.printf("WStype_ERROR : [%u]\n", num);
      break;
    case WStype_BIN:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    case WStype_FRAGMENT_TEXT_START:
    default:
      break;
  }
}

void setup()
{
  M5.begin(true, true, true, true);
  Serial.begin(115200);
  Serial.println("setup");

  M5.Axp.SetSpkEnable(true);
  M5.Axp.SetLDOEnable(3, false);
  M5.Axp.SetLed(false);
  M5.IMU.Init();

  long ret;

  ret = ctrl_m5core2_initialize(wifi_ssid, wifi_password);
  if( ret < 0 ){
    Serial.println("ctrl_m5core2_initialize error");
    while(1);
  }

  uint32_t length = CODE_BUFFER_SIZE;
  ret = file_read(JS_NAME_DEFAULT, (uint8_t *)js_buffer, &length);
  if( ret < 0 ){
    length = strlen(jscode_main);
    memmove(js_buffer, jscode_main, length);
    js_buffer[length] = '\0';
    Serial.println("QuickJS starting(default)");
  }else{
    js_buffer[length] = '\0';
    Serial.println("QuickJS starting");
  }
  Serial.println(js_buffer);

  qjs.begin();
  qjs.exec(js_buffer);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
  M5.update();
  ctrl_m5core2_loop();
  qjs.loop(); // For timer, async, etc.

  if( switch_jscode ){
//     webSocket.close();
//     Serial.println("Rebooting");
//     delay(1000);
//     esp_restart();
//     return;

    qjs.end();
    Serial.println("QuickJS stopped");
    switch_jscode = false;

    uint32_t length = CODE_BUFFER_SIZE;
    long ret = file_read(JS_NAME_DEFAULT, (uint8_t *)js_buffer, &length);
    if( ret < 0 )
      return;
    
    js_buffer[length] = '\0';
    Serial.println(js_buffer);

    Serial.println("QuickJS starting");
    qjs.begin();
    qjs.exec(js_buffer);
  }
}

