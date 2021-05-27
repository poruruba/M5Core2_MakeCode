#ifndef _CTRL_M5CORE2_H_
#define _CTRL_M5CORE2_H_

#define SPRITE_SIZE 100
#define DISP_WIDTH 320
#define DISP_HEIGHT 240
#define SND_VOLUME 7 // 0...21
#define BG_BUFFER_SIZE (8 * 1024)

long ctrl_m5core2_initialize(const char *ssid, const char *password);
void wifi_connect(const char *ssid, const char *password);
void ctrl_m5core2_loop(void);
long updateSprite(void);
long sound_play(const char* name);
long costume_change(const char *name);
long player_change_onoff(bool onoff);
long led_change_onoff(uint8_t onoff);
long player_change_step(int16_t x, int16_t y);
long player_change_goto(int16_t x, int16_t y);
long player_change_rotation(int16_t deg);
long player_change_angle(int16_t deg);
long player_get_position(int16_t *p_x, int16_t *p_y);
long player_get_direction(int16_t *p_degree);
long vibration_play(uint32_t duration);
long direction_get(float *p_x, float *p_y, float *p_z);
long acceleration_get(float *p_x, float *p_y, float *p_z);
long rolling_get(float *p_x, float *p_y, float *p_z);
long background_change(const char *name);
long file_read(const char *path, uint8_t *p_buffer, uint32_t *p_length);
long file_write(const char *path, const uint8_t *p_buffer, uint32_t length);
long player_distance(int16_t x, int16_t y);

#endif