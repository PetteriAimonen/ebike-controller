#pragma once

#include <stdbool.h>

void ui_start();
bool ui_task_settings_menu_accel_bias_is_open();
int ui_get_assist_level();
int ui_get_ok_button_clicks();
void ui_show_msg(const char *msg);
