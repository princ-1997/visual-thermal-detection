#define _CRT_SECURE_NO_WARNINGS
#include "cmd.h"

void* get_percentage_of_data_update(void* callback_data, void* priv_data)
{
    printf("download percentage = %.2f %%\n", *((float*)callback_data) * 100);
    return NULL;
}


unsigned short do_crc(unsigned char* ptr, int len)
{
    unsigned int i;
    unsigned short crc = 0x0000;

    while (len--)
    {
        crc ^= (unsigned short)(*ptr++) << 8;
        for (i = 0; i < 8; ++i)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

void get_device_info_demo(IrcmdHandle_t* ircmd_handle)
{
    if (ircmd_handle == NULL)
    {
        printf("ircmd_handle is NULL\n");
        return;
    }

    int param;
    int ret;
    char data[100];
    printf("enter get device info\n");
    while (1)
    {
        printf("----------please select the type of device information to obtain----------\n");
        printf("1:device name\n");
        printf("2:release version of firmware\n");
        printf("3:customer_id\n");
        printf("4:vendor_id\n");
        printf("5:product_id\n");
        printf("6:product number\n");
        printf("7:serial number\n");
        printf("8:data_version\n");
        printf("9:whole_package_version\n");
        printf("--------------------------------------------------------------------------\n");
        scanf("%d", &param);
        if (param > 9)
        {
            printf("you are exit get device info\n");
            break;
        }
        memset(data, 0, 100);
        switch (param)
        {
        case 1:
            ret = basic_device_info_get(ircmd_handle, 1, data);
            printf("ret = %d\n", ret);
            printf("get device name: %s\n", data);
            printf("\n");
            break;
        case 2:
            ret = basic_device_info_get(ircmd_handle, 2, data);
            printf("ret = %d\n", ret);
            printf("release version of firmware: %s\n", data);
            break;
        case 3:
            ret = basic_device_info_get(ircmd_handle, 3, data);
            printf("ret = %d\n", ret);
            printf("customer_id: %s\n", data);
            break;
        case 4:
            ret = basic_device_info_get(ircmd_handle, 4, data);
            printf("ret = %d\n", ret);
            printf("vendor_id:");
            for (int i = 0; i < 2; i++)
            {
                printf("0x%.2x ", data[i]);
            }
            printf("\n");
            break;
        case 5:
            ret = basic_device_info_get(ircmd_handle, 5, data);
            printf("ret = %d\n", ret);
            printf("product_id:");
            for (int i = 0; i < 2; i++)
            {
                printf("0x%.2x ", data[i]);
            }
            printf("\n");
            break;
        case 6:
            ret = basic_device_info_get(ircmd_handle, 6, data);
            printf("ret = %d\n", ret);
            printf("PN is: %s\n", data);
            break;
        case 7:
            ret = basic_device_info_get(ircmd_handle, 7, data);
            printf("ret = %d\n", ret);
            printf("SN is:");
            for (int i = 0; i < 14; i++)
            {
                printf("%c", data[i]);
            }
            printf("\n");
            break;
        case 8:
            ret = basic_device_info_get(ircmd_handle, 12, data);
            printf("ret = %d\n", ret);
            printf("data version is:%s\n", data);
            break;
        case 9:
            ret = basic_device_info_get(ircmd_handle, BASIC_DEV_WHOLE_PACKAGE_VERSION, data);
            printf("ret = %d\n", ret);
            printf("whole package version is:%s\n", data);
            break;
        default:
            printf("param is invalid!\n");
            break;
        }
    }

    return;
}

void palette_operate_demo(IrcmdHandle_t* ircmd_handle)
{
    if (ircmd_handle == NULL)
    {
        printf("ircmd_handle is NULL\n");
        return;
    }
    printf("enter palette operate\n");
    int color_number;
    int color_index;
    int param;
    int ret;

    while (1)
    {
        printf("-----------please select the type of palette operate-----------\n");
        printf("1:get_palette_num\n");
        printf("2:get_palette_idx\n");
        printf("3:set_palette_idx\n");
        printf("---------------------------------------------------------------\n");
        scanf("%d", &param);
        if (param > 3)
        {
            printf("you are exit palette operation\n");
            break;
        }
        switch (param)
        {
        case 1:
            ret = basic_palette_num_get(ircmd_handle, &color_number);
            printf("ret = %d\n", ret);
            printf("color_number = %d\n", color_number);
            break;
        case 2:
            ret = basic_palette_idx_get(ircmd_handle, &color_index);
            printf("ret = %d\n", ret);
            printf("color index is %d\n", color_index);
            break;
        case 3:
            printf("color_index is\n");
            scanf("%d", &color_index);
            ret = basic_palette_idx_set(ircmd_handle, color_index);
            printf("ret = %d\n", ret);
            break;
        default:
            printf("param is invalid!\n");
            break;
        }
    }

    return;
}

void temp_measure_demo(IrcmdHandle_t* ircmd_handle)
{
    if (ircmd_handle == NULL)
    {
        printf("ircmd_handle is NULL\n");
        return;
    }

    printf("enter temp measure\n");

    int param;
    int ret;
    IrcmdPoint_t point_pos;
    float point_temp_value;

    IrcmdLine_t line_pos;
    LineRectTempInfo_t line_temp_info;
    memset(&line_temp_info, 0, sizeof(line_temp_info));

    IrcmdRect_t rect_pos;
    LineRectTempInfo_t rect_temp_info;
    memset(&rect_temp_info, 0, sizeof(rect_temp_info));

    while (1)
    {
        printf("---------------please select the type of temp measure----------------\n");
        printf("1:get point temp\n");
        printf("2:get line temp\n");
        printf("3:get rect temp\n");
        printf("---------------------------------------------------------------------\n");
        scanf("%d", &param);
        if (param > 3)
        {
            printf("you are exit temp measure\n");
            break;
        }
        switch (param)
        {
        case 1:
            printf("Please enter point coordinate\n");
            scanf("%hd %hd", &(point_pos.x), &(point_pos.y));
            ret = basic_point_temp_info_get(ircmd_handle, point_pos, &point_temp_value);
            printf("ret = %d\n", ret);
            printf("point_temp_value is %f\n", point_temp_value);
            break;
        case 2:
            printf("Please enter line coordinate\n");
            printf("Please enter start point coordinate\n");
            scanf("%hd %hd", &(line_pos.start_point.x), &(line_pos.start_point.y));
            printf("Please enter end point coordinate\n");
            scanf("%hd %hd", &(line_pos.end_point.x), &(line_pos.end_point.y));

            ret = basic_line_temp_info_get(ircmd_handle, line_pos, &line_temp_info);
            printf("ret = %d\n", ret);

            printf("ave_temp is %f\n", line_temp_info.ave_temp);
            printf("max_temp is %f\n", line_temp_info.max_min_temp_info.max_temp);
            printf("max_temp_point_coordinate is ( %d , %d )\n", line_temp_info.max_min_temp_info.max_temp_point.x, \
                line_temp_info.max_min_temp_info.max_temp_point.y);
            printf("min_temp is %f\n", line_temp_info.max_min_temp_info.min_temp);
            printf("min_temp_point_coordinate is ( %d , %d )\n", line_temp_info.max_min_temp_info.min_temp_point.x, \
                line_temp_info.max_min_temp_info.min_temp_point.y);

            break;
        case 3:
            printf("Please enter rect coordinate\n");
            printf("Please enter start point coordinate\n");
            scanf("%hd %hd", &(rect_pos.start_point.x), &(rect_pos.start_point.y));
            printf("Please enter end point coordinate\n");
            scanf("%hd %hd", &(rect_pos.end_point.x), &(rect_pos.end_point.y));

            ret = basic_rect_temp_info_get(ircmd_handle, rect_pos, &rect_temp_info);
            printf("ret = %d\n", ret);

            printf("ave_temp is %f\n", rect_temp_info.ave_temp);
            printf("max_temp is %f\n", rect_temp_info.max_min_temp_info.max_temp);
            printf("max_temp_point_coordinate is ( %d , %d )\n", rect_temp_info.max_min_temp_info.max_temp_point.x, \
                rect_temp_info.max_min_temp_info.max_temp_point.y);
            printf("min_temp is %f\n", rect_temp_info.max_min_temp_info.min_temp);
            printf("min_temp_point_coordinate is ( %d , %d )\n", rect_temp_info.max_min_temp_info.min_temp_point.x, \
                rect_temp_info.max_min_temp_info.min_temp_point.y);

            break;
        default:
            printf("param is invalid!\n");
            break;
        }
    }

    return;
}

void image_effection_demo(IrcmdHandle_t* ircmd_handle)
{
    if (ircmd_handle == NULL)
    {
        printf("ircmd_handle is NULL\n");
        return;
    }

    printf("enter image effection\n");

    int param;
    int level;
    int ret;

    while (1)
    {
        printf("----------------please select the type of image effect----------------\n");
        printf("1:set brightness level\n");
        printf("2:get brightness level\n");
        printf("3:set contrast level\n");
        printf("4:get contrast level\n");
        printf("----------------------------------------------------------------------\n");
        scanf("%d", &param);
        if (param > 4)
        {
            printf("you are exit image effection\n");
            break;
        }
        switch (param)
        {
        case 1:
            printf("level is\n");
            scanf("%d", &level);
            ret = basic_image_brightness_level_set(ircmd_handle, level);
            printf("ret = %d\n", ret);
            break;
        case 2:
            ret = basic_current_brightness_level_get(ircmd_handle, &level);
            printf("ret = %d\n", ret);
            printf("level = %d\n", level);
            break;
        case 3:
            printf("level is\n");
            scanf("%d", &level);
            ret = basic_image_contrast_level_set(ircmd_handle, level);
            printf("ret = %d\n", ret);
            break;
        case 4:
            ret = basic_current_contrast_level_get(ircmd_handle, &level);
            printf("ret = %d\n", ret);
            printf("level = %d\n", level);
            break;
        default:
            printf("param is invalid!\n");
            break;
        }
    }

    return;
}

void switch_stream_source_demo(IrcmdHandle_t* ircmd_handle)
{
    if (ircmd_handle == NULL)
    {
        printf("ircmd_handle is NULL\n");
        return;
    }

    printf("switch stream source\n");

    int source;
    int ret;

    while (1)
    {
        printf("----------------please select the stream source----------------\n");
        printf("0:ADV_IR_SOURCE_MODE\n");
        printf("1:ADV_KBC_SOURCE_MODE\n");
        printf("2:ADV_TNR_SOURCE_MODE\n");
        printf("3:ADV_HBC_DPC_SOURCE_MODE\n");
        printf("4:ADV_VBC_SOURCE_MODE\n");
        printf("5:ADV_SNR_SOURCE_MODE\n");
        printf("6:ADV_DDE_SOURCE_MODE\n");
        printf("7:ADV_AGC_SOURCE_MODE\n");
        printf("8:ADV_GAMMA_SOURCE_MODE\n");
        printf("9:ADV_TPD_SOURCE_MODE\n");
        printf("10:ADV_MIRROR_SOURCE_MODE\n");
        printf("11:ADV_PTCTURE_SOURCE_MODE\n");
        printf("----------------------------------------------------------------------\n");
        scanf("%d", &source);
        if (source > 11)
        {
            printf("you are exit image effection\n");
            break;
        }
        ret = adv_stream_source_mode_set(ircmd_handle, source);
        printf("ret =%d\n", ret);
    }
    return;
}

//command selection
void command_sel(int cmd_type, StreamFrameInfo_t* handle)
{
    int ret = 0;
    int param;
    int mode;
    VideoOutputInfo_t output_info;
    IrcmdHandle_t* ircmd_handle = handle->ircmd_handle;
    struct timeval current_time;
    struct tm *ptm;
    Rtc_Time_t rtc_time;
    switch (cmd_type)
    {
    case 1:
        get_device_info_demo(ircmd_handle);
        break;
    case 2:
        ret = basic_ffc_update(ircmd_handle);
        printf("ret = %d\n", ret);
        break;
    case 3:
        palette_operate_demo(ircmd_handle);
        break;
    case 4:
        temp_measure_demo(ircmd_handle);
        break;
    case 5:
        printf("mode is 0:default 1:rescue 2:patrol 3:urban\n");
        scanf("%d", &mode);
        ret = basic_image_scene_mode_set(ircmd_handle, mode);
        printf("ret = %d", ret);
        break;
    case 6:
        image_effection_demo(ircmd_handle);
        break;
    case 7:
        printf("switch stream source mode\n");
        switch_stream_source_demo(ircmd_handle);
        break;
    if (handle->product_config.camera.open_temp_measure)
    {
    case 8:
#if defined(_WIN32)
        ReleaseSemaphore(temp_sem, 1, NULL);
        WaitForSingleObject(cmd_sem, INFINITE);
#elif defined(linux) || defined(unix)
        sem_post(&temp_sem);
        sem_wait(&cmd_sem);
#endif
        break;
    }
    case 9:
        output_info.video_output_status = BASIC_ENABLE;
        printf("0:ADV_USB_FORMAT\n");
        printf("1:ADV_DVP_FORMAT\n");
        printf("2:ADV_BT656_FORMAT\n");
        printf("3:ADV_MIPI_FORMAT\n");
        printf("4:ADV_BT656_INTERLACED_FORMAT\n");
        scanf("%d", &output_info.video_output_format);
        printf("please input fps\n");
        scanf("%d", &output_info.video_output_fps);
        ret = adv_digital_video_output_set(ircmd_handle, output_info);
        printf("ret = %d\n", ret);
        break;
    case 10:
        output_info.video_output_status = BASIC_DISABLE;
        output_info.video_output_format = ADV_MIPI_FORMAT;
        output_info.video_output_fps = 30;
        ret = adv_digital_video_output_set(ircmd_handle, output_info);
        printf("ret = %d\n", ret);
        break;
    #if defined(linux) || defined(unix)
    case 11:
        //触发GPIO
        system("echo 80 > /sys/class/gpio/export");
        system("echo out > /sys/class/gpio/gpio80/direction");
        system("echo 1 > /sys/class/gpio/gpio80/value");
        //获取系统时间
        gettimeofday(&current_time, NULL);
        ptm = localtime(&current_time.tv_sec);
        rtc_time.day = ptm->tm_mday;
        rtc_time.month = ptm->tm_mon + 1;
        rtc_time.year = ptm->tm_year + 1900;
        rtc_time.hour = ptm->tm_hour;
        rtc_time.minute = ptm->tm_min;
        rtc_time.second = ptm->tm_sec;
        rtc_time.millisecond = current_time.tv_usec / 1000;
        rtc_time.microsecond = current_time.tv_usec % 1000;
        //设置RTC
        ret = basic_rtc_current_time_set(ircmd_handle, &rtc_time);
        printf("ret = %d\n", ret);
        break;
    #endif
    default:
        printf("invalid cmd!\n");
        break;
    }
}


//command thread function
void* cmd_function(void* threadarg)
{
    int isRUNNING = 1;
    int cmd;
    while (isRUNNING)
    {
        printf("--------------------------please select cmd type--------------------------\n");
        printf("1 :get_device_info\n");
        printf("2 :basic_ffc_update\n");
        printf("3 :palette_operate\n");
        printf("4 :temp_measure\n");
        printf("5 :set_image_scene_mode\n");
        printf("6 :image_effection\n");
        printf("7 :switch_stream_source_mode\n");
        if (((StreamFrameInfo_t*)threadarg)->product_config.camera.open_temp_measure)
        {
            printf("8 :temp_correct\n");
        }
        printf("9:digital_video_output_start\n");
        printf("10:digital_video_output_stop\n");
        #if defined(linux) || defined(unix)
        printf("11:basic_rtc_current_time_set\n");
        #endif
        printf("--------------------------------------------------------------------------\n");
        scanf("%d", &cmd);
        command_sel(cmd, (StreamFrameInfo_t*)threadarg);
    }
    printf("cmd thread exit!!\n");
    return NULL;
}

void* get_percentage_of_cfg_update(void* callback_param,void* data)
{
    printf("download percentage = %.2f %%\n", *((float*)callback_param) * 100);
    return NULL;
}


void judge_path_valid(char* path, int* valid_flag, char* basename)
{
    if (path == NULL || valid_flag == NULL || basename == NULL)
    {
        printf("path or valid_flag or basename is NULL\n");
        return;
    }

    char str_dirname[256] = { '\0' };

    int dir_exist_flag = 0;
    int i;
    int length = strlen(path);
    for (i = length - 1; i >= 0; i--)
    {
        if (path[i] == '\\' || path[i] == '/')
            break;
    }
    if (i != -1)
    {
        strncpy(str_dirname, path, i + 1);
        str_dirname[i + 1] = '\0';

#if defined(_WIN32)
        dir_exist_flag = _access(str_dirname, 0);
#elif defined (unix) || (linux)
        dir_exist_flag = access(str_dirname, 0);
#endif

        if (dir_exist_flag < 0)
        {
            printf("fail to access %s\n", path);
            *valid_flag = PATH_INVALID;
            return;
        }
        strncpy(basename, path + i + 1, length - i - 1);
        basename[length - i - 1] = '\0';
    }
    else
    {
        strcpy(basename, path);
    }
    if (strstr(basename, ".bin") == NULL || strlen(basename) <= 4)
    {
        printf("input path is invalid\n");
        *valid_flag = PATH_INVALID;
    }
    else
    {
        *valid_flag = PATH_VALID;
    }

    return;
}


#if defined (_WIN32)
int search_com(IrControlHandle_t* ir_control_handle, IrcmdHandle_t** cmd_handle, int com_number)
{
    if (ir_control_handle == NULL)
    {
        printf("handle is NULL\n");
        return -1;
    }

    TCHAR dev_node[30];
    TCHAR prefix[20] = _TEXT("\\\\.\\COM");
    TCHAR temp[10];
    char data[100];
    int ret;
    if (com_number == 0)
    {
        iruart_log_register(IRUART_LOG_NO_PRINT,NULL,NULL);
        ircmd_log_register(IRCMD_LOG_NO_PRINT, NULL,NULL);
        printf("Searching available serial port......\n");

        int i = 0;
        for (i = 0; i <= 100; i++)
        {
            memset(dev_node, 0, 30 * sizeof(TCHAR));
            _tcscat(dev_node, prefix);
            _tcscat(dev_node, _itow(i, temp, 10));
            ret = ir_control_handle->ir_control_open(ir_control_handle->ir_control_handle, (void*)(dev_node));
            if (ret == 0)
            {
                int status;
                // judge in rom
                // ret = ir_control_handle->ir_control_detect_device_status(ir_control_handle->ir_control_handle, NULL, &status);
                // if (ret != IRLIB_SUCCESS || status != 1)
                // {
                    // if it's in rom baudrate will be set 115200, else use "ir_control_init" adapt baudrate
                ret = ir_control_handle->ir_control_init(ir_control_handle->ir_control_handle, NULL);
                if (ret != 0)
                {
                    printf("COM%d set baudrate failed\n", i);
                }
                // }
                *cmd_handle = ircmd_create_handle(ir_control_handle);
                ret = basic_device_info_get(*cmd_handle, 1, data);
                if (ret == 0)
                {
                    printf("success to open COM%d\n", i);
                    return 0;
                }
                else
                {
                    printf("COM%d exist, but not uart0\n", i);
                    ir_control_handle->ir_control_close(ir_control_handle->ir_control_handle);
                }
            }
        }
        if (i > 31)
        {
            printf("fail to search available com\n");
            return -1;
        }
    }
    else
    {
        printf("attempt to connect the specified serial port......\n");
        memset(dev_node, 0, 30 * sizeof(TCHAR));
        _tcscat(dev_node, prefix);
        _tcscat(dev_node, _itow(com_number, temp, 10));
        ret = ir_control_handle->ir_control_open(ir_control_handle->ir_control_handle, (void*)(dev_node));
        int status;
        // judge in rom
        // ret = ir_control_handle->ir_control_detect_device_status(ir_control_handle->ir_control_handle, NULL, &status);
        // if (ret != IRLIB_SUCCESS || status != 1)
        // {
            // if it's in rom baudrate will be set 115200, else use "ir_control_init" adapt baudrate
        ret = ir_control_handle->ir_control_init(ir_control_handle->ir_control_handle, NULL);
        if (ret != 0)
        {
            printf("COM%d set baudrate failed\n", com_number);
        }
        // }
        *cmd_handle = ircmd_create_handle(ir_control_handle);
        if (ret == 0)
        {
            printf("success to open COM%d\n", com_number);
            return 0;
        }
        else
        {
            printf("fail to open COM%d\n", com_number);
            return -1;
        }
    }

    return 0;
}
#elif defined (linux) || (unix)
int search_com(IrControlHandle_t* ir_control_handle, IrcmdHandle_t** cmd_handle, int com_number)
{
    if (ir_control_handle == NULL)
    {
        printf("handle is NULL\n");
        return -1;
    }

    FILE* fp;
    char data[100];
    char buffer[100] = { 0 };
    char dev_node[30] = { 0 };
    char prefix[20] = "/dev/";
    char temp[10] = { 0 };
    char find_str[10] = "ttyUSB";
    char* tmp = NULL;
    char* current_addr = NULL;

    int ret;

    if (com_number == 0)
    {
        iruart_log_register(IRUART_LOG_NO_PRINT,NULL,NULL);
        ircmd_log_register(IRCMD_LOG_NO_PRINT,NULL,NULL);
        printf("Searching available serial port......\n");
        fp = popen("ls /dev | grep \"ttyUSB*\" ", "r");
        fread(buffer, 1, sizeof(buffer), fp);
        pclose(fp);
        fp = NULL;
        printf("content of ls /dev | grep \"ttyUSB*\" is %s\n", buffer);

        int i = 0;
        current_addr = buffer;

        while ((tmp = strstr(current_addr, find_str)) != NULL)
        {
            memset(dev_node, 0, 30 * sizeof(char));
            strcat(dev_node, prefix);

            for (i = 0; i <= (int)strlen(tmp); i++)
            {
                if (tmp[i] == '\n' || tmp[i] == '\0' || tmp[i] == ' ')
                {
                    break;
                }
            }
            strncpy(dev_node + strlen(dev_node), tmp, i);
            current_addr += i;

            ret = ir_control_handle->ir_control_open(ir_control_handle->ir_control_handle, (void*)(dev_node));
            if (ret == 0)
            {
                int status;
                // judge in rom
                // ret = ir_control_handle->ir_control_detect_device_status(ir_control_handle->ir_control_handle, NULL, &status);
                // if (ret != IRLIB_SUCCESS || status != 1)
                // {
                    // if it's in rom baudrate will be set 115200, else use "ir_control_init" adapt baudrate
                ret = ir_control_handle->ir_control_init(ir_control_handle->ir_control_handle, NULL);
                if (ret != 0)
                {
                    printf("COM%d set baudrate failed\n", i);
                }
                // }
                *cmd_handle = ircmd_create_handle(ir_control_handle);
                ret = basic_device_info_get(*cmd_handle, 1, data);
                if (ret == 0)
                {
                    printf("success to open %s\n", dev_node);
                    return 0;
                }
                else
                {
                    printf("%s exist, but not uart0\n", dev_node);
                    ir_control_handle->ir_control_close(ir_control_handle->ir_control_handle);
                }
            }
        }

    }
    else
    {
        printf("attempt to connect the specified serial port......\n");
        strcat(dev_node, prefix);
        strcat(dev_node, "ttyUSB");
        sprintf(temp, "%d", com_number);
        strcat(dev_node, temp);
        ret = ir_control_handle->ir_control_open(ir_control_handle->ir_control_handle, (void*)(dev_node));
        int status;
        // judge in rom
        // ret = ir_control_handle->ir_control_detect_device_status(ir_control_handle->ir_control_handle, NULL, &status);
        // if (ret != IRLIB_SUCCESS || status != 1)
        // {
            // if it's in rom baudrate will be set 115200, else use "ir_control_init" adapt baudrate
        ret = ir_control_handle->ir_control_init(ir_control_handle->ir_control_handle, NULL);
        if (ret != 0)
        {
            printf("COM%d set baudrate failed\n", com_number);
        }
        // }
        *cmd_handle = ircmd_create_handle(ir_control_handle);
        if (ret == 0)
        {
            printf("success to open COM%d\n", com_number);
            return 0;
        }
        else
        {
            printf("fail to open COM%d\n", com_number);
            return -1;
        }
    }

    return 0;
}
#endif

int judge_device_status(IrcmdHandle_t* ircmd_handle)
{
    if(ircmd_handle == NULL)
    {
        printf("ircmd_handle is null\n");
        return -1;
    }
    int ret = 0;
    int execution_times = 0;
    char package_version[50];
    while(1)
    {
        if(execution_times > 10)
        {
            return -1;
        }
        ret = basic_device_info_get(ircmd_handle, BASIC_DEV_WHOLE_PACKAGE_VERSION, package_version);
        if(ret == IRLIB_SUCCESS)
        {
            break;
        }
        #if defined(_WIN32)
        Sleep(1);
        #elif defined(linux) || defined(unix)
        usleep(1000);
        #endif
        execution_times++;
    }
    printf("device status is ready\n");
    return 0;
}