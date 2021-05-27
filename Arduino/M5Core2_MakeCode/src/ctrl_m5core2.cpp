#define LGFX_AUTODETECT
//#define ENABLE_AUDIO

#include <M5Core2.h>
#include <WiFi.h>
#include <LovyanGFX.hpp>
#include <FS.h>
#include <SD.h>
#include "ctrl_m5core2.h"

#ifdef ENABLE_AUDIO
#include "Audio.h"
Audio audio;
#endif

static LGFX lcd;
static LGFX_Sprite sprite(&lcd);

static const char *COSTUME_BASE = "/costume/";
static const char *BACKGROUND_BASE = "/background/";
#ifdef ENABLE_AUDIO
static const char *SOUND_BASE = "/sound/";
#endif

extern const uint8_t default_costume_start[] asm("_binary_rom_costume1_png_start");
extern const uint8_t default_costume_end[] asm("_binary_rom_costume1_png_end");
extern const uint8_t default_background_start[] asm("_binary_rom_background1_jpg_start");
extern const uint8_t default_background_end[] asm("_binary_rom_background1_jpg_end");

uint8_t background_buffer[BG_BUFFER_SIZE];
uint32_t background_size;

int16_t player_x = 0;
int16_t player_y = 0;
int16_t player_degree = 0;
bool player_onoff = true;

long file_read(const char *path, uint8_t *p_buffer, uint32_t *p_length)
{
  File file = SD.open(path, FILE_READ);
  if (!file)
    return -1;
  uint32_t ptr = 0;
  while (file.available())
  {
    if (ptr > *p_length)
      return -1;
    p_buffer[ptr++] = file.read();
  }
  file.close();
  *p_length = ptr;

  return *p_length;
}

long file_write(const char *path, const uint8_t *p_buffer, uint32_t length)
{
  File file = SD.open(path, FILE_WRITE);
  if (!file)
    return -1;

  for (uint32_t ptr = 0; ptr < length ; ptr++ )
    file.write(p_buffer[ptr]);
  file.close();

  return length;
}

char tohex(uint8_t c){
  if( c < 10 )
    return '0' + c;
  if( c < 16 )
    return 'a' + (c - 10);
  return '0';
}

long bin2hex(const char* p_str, char* p_hex){
  uint16_t index = 0;
  while(p_str[index] != '\0'){
    p_hex[index * 2] = tohex((p_str[index] >> 4) & 0x0f);
    p_hex[index * 2 + 1] = tohex(p_str[index] & 0x0f);
    index++;
  }
  p_hex[index * 2] = '\0';

  return index;
}

String make_path(const char *p_base, const char *p_name, const char *p_ext)
{
  return String(p_base) + p_name + p_ext;
}

long ctrl_m5core2_initialize(const char *ssid, const char *password)
{
  long ret;

#ifdef ENABLE_AUDIO
  audio.setPinout(12, 0, 2);
  audio.setVolume(SND_VOLUME);
#endif

  lcd.init();
//  lcd.setFont(&fonts::lgfxJapanGothic_16);
  lcd.setBrightness(128);
  lcd.setCursor(0, 50);
  lcd.printf("Now WiFi connecting");

  wifi_connect(ssid, password);

  player_x = DISP_WIDTH / 2;
  player_y = DISP_HEIGHT / 2;
  player_degree = 0;
  player_onoff = true;

  background_size = default_background_end - default_background_start;
  memmove(background_buffer, default_background_start, background_size);

  sprite.createSprite(SPRITE_SIZE, SPRITE_SIZE);
  sprite.drawPng(default_costume_start, default_costume_end - default_costume_start);

  ret = updateSprite();
  if( ret < 0 )
    return ret;

  return 0;
}

void ctrl_m5core2_loop(void){
#ifdef ENABLE_AUDIO
  if (audio.isRunning())
    audio.loop();
#endif
}

long sound_play(const char* name){
#ifdef ENABLE_AUDIO
  String sound_path = make_path(SOUND_BASE, name, ".mp3");
  Serial.println(sound_path);
  if (audio.isRunning())
    audio.stopSong();
  Serial.printf("sound_play : %s\n", sound_path.c_str());
  audio.connecttoFS(SD, sound_path.c_str());
#endif

  return 0;
}

long updateSprite(void)
{
  lcd.drawJpg(background_buffer, background_size);
  if (player_onoff)
    sprite.pushRotateZoom(player_x, player_y, player_degree, 1.0f, 1.0f, 0);

  return 0;
}

long costume_change(const char *name)
{
  sprite.clear();
  String costume_path = make_path(COSTUME_BASE, name, ".png");
  Serial.printf("costume_change : %s\n", costume_path.c_str());
  bool ret = sprite.drawPngFile(SD, costume_path);
  if( !ret ){
    Serial.println("drawPngFile error");
    return -1;
  }
  
  return 0;
}

long player_change_onoff(bool onoff){
  player_onoff = onoff;
  return 0;
}

long led_change_onoff(uint8_t onoff)
{
  M5.Axp.SetLed(onoff);
  return 0;
}

long player_change_step(int16_t x, int16_t y){
  player_x += x;
  player_y += y;

  return 0;
}

long player_change_goto(int16_t x, int16_t y){
  player_x = x;
  player_y = y;

  return 0;
}

long player_change_rotation(int16_t deg)
{
  player_degree += deg;
  player_degree %= 360;

  return 0;
}

long player_change_angle(int16_t deg)
{
  player_degree = deg;
  player_degree %= 360;

  return 0;
}

long player_get_position(int16_t *p_x, int16_t *p_y)
{
  *p_x = player_x;
  *p_y = player_y;

  return 0;
}

long player_get_direction(int16_t *p_degree)
{
  *p_degree = player_degree;

  return 0;
}

long vibration_play(uint32_t duration){
  M5.Axp.SetLDOEnable(3, true);
  delay(duration);
  M5.Axp.SetLDOEnable(3, false);

  return 0;
}

long direction_get(float *p_x, float *p_y, float *p_z)
{
  M5.IMU.getGyroData(p_x, p_y, p_z);
  return 0;
}

long acceleration_get(float *p_x, float *p_y, float *p_z)
{
  M5.IMU.getAccelData(p_x, p_y, p_z);
  return 0;
}

long rolling_get(float *p_x, float *p_y, float *p_z)
{
  M5.IMU.getAhrsData(p_x, p_y, p_z);
  return 0;
}

long background_change(const char *name)
{
  String background_path = make_path(BACKGROUND_BASE, name, ".jpg");
  Serial.printf("background_change : %s\n", background_path.c_str());

  background_size = sizeof(background_buffer);
  long ret = file_read(background_path.c_str(), background_buffer, &background_size);
  if( ret < 0 ){
    Serial.println("file_read error");
    return ret;
  }
  
  return 0;
}

long player_distance(int16_t x, int16_t y){
  if( !player_onoff )
    return -1;

  int16_t d_x, d_y;
  d_x = player_x - x;
  d_y = player_y - y;
  return d_x * d_x + d_y * d_y;
}

void wifi_connect(const char *ssid, const char *password)
{
  Serial.println("");
  Serial.print("WiFi Connenting");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected : ");
  Serial.print(WiFi.localIP());
  Serial.println("");
}