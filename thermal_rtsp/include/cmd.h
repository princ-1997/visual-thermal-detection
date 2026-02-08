#ifndef _CMD_H_
#define _CMD_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined (_WIN32)
#include "windows.h"
#include "io.h"
#include "tchar.h"
#endif

#include "data.h"
#include "libiruart.h"

#define PATH_INVALID    0
#define PATH_VALID      1

typedef struct{
	IrcmdHandle_t* handle;
	float percentage;
}user_cfg_t;

typedef enum {
    TEST_NORMAL_FILE = 0,
    TEST_SCATTERED_FILE = 1,
}test_single_file_type_e;

//int preview_stop(Iruvc_v4l2Handle_t *iruvc_handle);

//int preview_start(Iruvc_v4l2Handle_t *iruvc_handle,StreamFrameInfo_t* handle);

void judge_path_valid(char* path, int* valid_flag, char* basename);

//select the command
void command_sel(int cmd_type, StreamFrameInfo_t* handle);
int search_com(IrControlHandle_t* ir_control_handle, IrcmdHandle_t** cmd_handle, int com_number);

//command thread, get the input and select the command
void* cmd_function(void* threadarg);

//judge whether the device data is ready
int judge_device_status(IrcmdHandle_t* ircmd_handle);

#endif


