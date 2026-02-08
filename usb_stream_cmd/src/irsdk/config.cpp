#include "config.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include "cJSON.h"
#include "data.h"

using std::ifstream;
using std::stringstream;
using std::istreambuf_iterator;
using std::cin;
using std::cout;
using std::endl;

#define PARSE_STRING_VALUE(json, obj, value) \
do {\
    cJSON* object_item = cJSON_GetObjectItem(json, obj);\
    if (object_item != nullptr) { \
        value = cJSON_GetStringValue(object_item); \
    } else {\
        cout << "not found " << obj << " object item" << endl; \
        return -1; \
    } \
} while(0)

#define PARSE_STRING_VALUE_WITHOUT_RETURN(json, obj, value) \
do {\
    cJSON* object_item = cJSON_GetObjectItem(json, obj);\
    if (object_item != nullptr) { \
        value = cJSON_GetStringValue(object_item); \
    } \
} while(0)

#define PARSE_NUMBER_VALUE(json, obj, value) \
do {\
    cJSON* object_item = cJSON_GetObjectItem(json, obj);\
    if (object_item != nullptr) { \
        value = cJSON_GetNumberValue(object_item); \
    } else {\
        cout << "not found " << obj << " object item" << endl; \
        return -1; \
    } \
} while(0)

#define PARSE_BOOL_VALUE_WITHOUT_RETURN(json, obj, value) \
do {\
    cJSON* object_item = cJSON_GetObjectItem(json, obj);\
    if (object_item != nullptr) { \
        value = cJSON_IsTrue(object_item); \
    } \
} while(0)

static const string format_map[5] = { "YUYV_IMAGE", "NV12_IMAGE", "NV12_AND_TEMP", "YUYV_AND_TEMP", "UYVY_IMAGE" };

config::config()
    : frame_output_format_dict{},
    product_config{}
{
    init_frame_output_format_dict();
}

int config::parse_config(const char* config_file_path)
{

    ifstream fin(config_file_path, ifstream::binary);
    string str((istreambuf_iterator<char>(fin)), istreambuf_iterator<char>());
    fin.close();
    // parse json
    parse_config_json(str.data(), str.length());
    return 0;
}

bool config::get_config(single_config& sc, const string& product)
{
    if (product_config.find(product) != product_config.end()) {
        sc = product_config[product];
        return true;
    }
    return false;
}

bool config::get_config(single_config& sc)
{
    cout << "********** select product **********" << endl;
    cout << "0: quit process" << endl;
    int i = 1;
    vector<string> products;
    products.push_back("");
    for (const auto& config : product_config)
    {
        cout << i++ << ": " << config.first << endl;
        products.push_back(config.first);
    }
    printf("*********** input number ***********\n");

    size_t select_product;
    while (1)
    {
        cin >> select_product;
        if (select_product > 0 && select_product < products.size())
        {
            sc = product_config[products[select_product]];
            cout << sc.to_string();
            return true;
        }
        if (select_product == 0)
        {
            return false;
        }
        else if (select_product > products.size())
        {
            cout << "input an illicit number" << endl;
        }
    }

    return false;
}

void config::init_frame_output_format_dict()
{
    frame_output_format_dict["yuyv_image"] = YUYV_IMAGE;
    frame_output_format_dict["nv12_image"] = NV12_IMAGE;
    frame_output_format_dict["nv12_and_temp"] = NV12_AND_TEMP;
    frame_output_format_dict["yuyv_and_temp"] = YUYV_AND_TEMP;
    frame_output_format_dict["uyvy_image"] = UYVY_IMAGE;
}

int config::parse_config_json(const char* data, size_t data_length)
{
    cJSON* json = cJSON_ParseWithLength(data, data_length);
    if (json == nullptr)
    {
        cout << "failed parse json" << endl;
        return -1;
    }

    int config_size = cJSON_GetArraySize(json);
    for (int config_idex = 0; config_idex < config_size; ++config_idex)
    {
        cJSON* config_obj = cJSON_GetArrayItem(json, config_idex);
        if (parse_single_config(config_obj) != 0)
        {
            cout << "failed parse config object[" << config_idex << "]" << endl;
            cJSON_Delete(json);
            return -1;
        }
    }
    cJSON_Delete(json);
    return 0;
}

int config::parse_single_config(const cJSON* json)
{
    single_config sc;

    string product_name;
    PARSE_STRING_VALUE(json, "product", product_name);
    cJSON* data_item = cJSON_GetObjectItem(json, "camera");
    if (parse_camera_config(data_item, sc.camera) != 0)
    {
        cout << "parse data config failed" << endl;
        return -1;
    }
    cJSON* control_item = cJSON_GetObjectItem(json, "control");
    if (parse_control_config(control_item, sc.control) != 0)
    {
        cout << "parse control_config failed" << endl;
        return -1;
    }

    product_config[product_name] = sc;
    return 0;
}

int config::parse_control_config(const cJSON* json, control_config& control)
{
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "is_uart_control", control.is_uart_control);
    if (control.is_uart_control)
    {
        cJSON* uart_param_item = cJSON_GetObjectItem(json, "uart_param");
        if (uart_param_item == nullptr)
        {
            cout << "use uart control but not found uart_param in json" << endl;
            return -1;
        }
        PARSE_NUMBER_VALUE(uart_param_item, "com_index", control.uart_param.com_index);

        return 0;
    }
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "is_i2c_control", control.is_i2c_control);
    if (control.is_i2c_control)
    {
        cJSON* i2c_param_item = cJSON_GetObjectItem(json, "i2c_param");
        if (i2c_param_item == nullptr)
        {
            cout << "use uart control but not found i2c_param in json" << endl;
            return -1;
        }
        PARSE_STRING_VALUE(i2c_param_item, "device_name", control.i2c_param.dev_name);
        return 0;
    }
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "is_i2c_usb_control", control.is_i2c_usb_control);
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "is_usb_control", control.is_usb_control);
    if (control.is_i2c_usb_control || control.is_usb_control)
    {
        cJSON* usb_param_item = cJSON_GetObjectItem(json, "usb_param");
        if (usb_param_item == nullptr)
        {
            cout << "use uart control but not found usb_param in json" << endl;
            return -1;
        }
        return parse_uvc_dev_info(usb_param_item, control.usb_param);
    }
    return 0;
}

int config::parse_camera_config(const cJSON* json, camera_config& data)
{
    string format;
    PARSE_STRING_VALUE(json, "format", format);
    if (frame_output_format_dict.find(format) == frame_output_format_dict.end())
    {
        cout << "set an illicit format" << endl;
        return -1;
    }
    data.format = frame_output_format_dict[format];
    PARSE_NUMBER_VALUE(json, "width", data.width);
    PARSE_NUMBER_VALUE(json, "height", data.height);
    PARSE_NUMBER_VALUE(json, "image_info_height", data.image_info_height);
    PARSE_NUMBER_VALUE(json, "info_line_height", data.info_line_height);
    PARSE_NUMBER_VALUE(json, "temp_info_height", data.temp_info_height);
    PARSE_NUMBER_VALUE(json, "dummy_info_height", data.dummy_info_height);
    PARSE_NUMBER_VALUE(json, "image_info_ratio", data.image_info_ratio);
    PARSE_NUMBER_VALUE(json, "info_line_ratio", data.info_line_ratio);
    PARSE_NUMBER_VALUE(json, "temp_line_ratio", data.temp_line_ratio);
    PARSE_NUMBER_VALUE(json, "dummy_info_ratio", data.dummy_info_ratio);
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "is_auto_image", data.is_auto_image);
    PARSE_BOOL_VALUE_WITHOUT_RETURN(json, "open_temp_measure", data.open_temp_measure);
    PARSE_STRING_VALUE_WITHOUT_RETURN(json, "image_channel_type", data.image_channel_type);
    cJSON* v4l2_stream_item = cJSON_GetObjectItem(json, "v4l2_stream");
    if (v4l2_stream_item != nullptr && parse_v4l2_stream_config(v4l2_stream_item, data.v4l2_config) != 0)
    {
        cout << "parse v4l2 stream failed" << endl;
        return -1;
    }
    cJSON* uvc_stream_item = cJSON_GetObjectItem(json, "uvc_stream");
    if (uvc_stream_item != nullptr && parse_uvc_stream_config(uvc_stream_item, data.uvc_stream_conf) != 0)
    {
        cout << "parse uvc stream failed" << endl;
        return -1;
    }
    return 0;
}

int config::parse_v4l2_stream_config(const cJSON* json, v4l2_streams& v4l2_stream_config)
{
    int v4l2_size = cJSON_GetArraySize(json);
    for (int v4l2_idx = 0; v4l2_idx < v4l2_size; ++v4l2_idx)
    {
        cJSON* v4l2_obj = cJSON_GetArrayItem(json, v4l2_idx);
        v4l2_stream vs;
        PARSE_STRING_VALUE(v4l2_obj, "device_name", vs.device_name);
        PARSE_STRING_VALUE(v4l2_obj, "type", vs.type);
        PARSE_NUMBER_VALUE(v4l2_obj, "fps", vs.fps);
        PARSE_NUMBER_VALUE(v4l2_obj, "dev_width", vs.dev_width);
        PARSE_NUMBER_VALUE(v4l2_obj, "dev_height", vs.dev_height);
        if (vs.type == "image")
        {
            v4l2_stream_config.image_stream = vs;
            v4l2_stream_config.has_image = true;
        }
        else if (vs.type == "temp")
        {
            v4l2_stream_config.temp_stream = vs;
            v4l2_stream_config.has_temp = true;
        }
        else
        {
            cout << "set stream not image or temp" << endl;
        }
    }

    return 0;
}

int config::parse_uvc_stream_config(const cJSON* json, uvc_stream& uvc_stream_config)
{
    cJSON* dev_info_obj = cJSON_GetObjectItem(json, "device_info");
    if (dev_info_obj == nullptr)
    {
        cout << "not found dev_info_obj" << endl;
        return -1;
    }
    if (parse_uvc_dev_info(dev_info_obj, uvc_stream_config.dev_info) != 0)
    {
        cout << "parse uvc dev info failed" << endl;
        return -1;
    }
    PARSE_NUMBER_VALUE(json, "width", uvc_stream_config.width);
    PARSE_NUMBER_VALUE(json, "height", uvc_stream_config.height);
    PARSE_NUMBER_VALUE(json, "fps", uvc_stream_config.fps);
    PARSE_NUMBER_VALUE(json, "frame_size_ratio", uvc_stream_config.frame_size_ratio);
    return 0;
}

int string_to_int_0x(string number)
{
    int ret = 0;
    if (number.length() < 6 || number.substr(0, 2) != "0x")
    {
        return -1;
    }
    for (int i = 2; i < 6; ++i)
    {
        if (number[i] >= 'a' && number[i] <= 'f')
        {
            number[i] -= 32;
        }
    }
    ret = ret + (number[2] >= 'A' && number[2] <= 'F' ? (number[2] - 'A' + 10) * 16 * 16 * 16 : (number[2] - '0') * 16 * 16 * 16);
    ret = ret + (number[3] >= 'A' && number[3] <= 'F' ? (number[3] - 'A' + 10) * 16 * 16 : (number[3] - '0') * 16 * 16);
    ret = ret + (number[4] >= 'A' && number[4] <= 'F' ? (number[4] - 'A' + 10) * 16 : (number[4] - '0') * 16);
    ret = ret + (number[5] >= 'A' && number[5] <= 'F' ? (number[5] - 'A' + 10) : (number[5] - '0'));
    return ret;
}

int config::parse_uvc_dev_info(const cJSON* json, usb_dev_info& dev_info)
{
    string pid, vid;
    int pid_int, vid_int;
    PARSE_STRING_VALUE(json, "pid", pid);
    if ((pid_int = string_to_int_0x(pid)) < 0)
    {
        cout << "pid set error" << endl;
        return -1;
    }
    dev_info.pid = pid_int;
    PARSE_STRING_VALUE(json, "vid", vid);
    if ((vid_int = string_to_int_0x(vid)) < 0)
    {
        cout << "vid set error" << endl;
        return -1;
    }
    dev_info.vid = vid_int;
    PARSE_NUMBER_VALUE(json, "same_id", dev_info.same_id);
    return 0;
}

string v4l2_stream::to_string()
{
    stringstream ss;
    ss << "device_name: " << device_name << ", type:" << type << endl;
    ss << "fps: " << fps << ", dev_width: " << dev_width << ", dev_height: " << dev_height << endl;
    return ss.str();
}

string usb_dev_info::to_string()
{
    stringstream ss;
    ss << "pid: 0x" << std::hex << pid << ", vid: 0x" << vid << ", same_id: " << same_id << endl;
    return ss.str();
}

string uvc_stream::to_string()
{
    stringstream ss;
    ss << dev_info.to_string();
    ss << "width: " << width << ", height: " << height << ", fps: " << fps << endl;
    ss << "frame_size_ratio: " << frame_size_ratio << endl;
    return ss.str();
}

control_config::control_config()
    : is_uart_control(false),
      is_i2c_control(false),
      is_usb_control(false),
      is_i2c_usb_control(false)
{

}

control_config& control_config::operator=(const control_config& rhs)
{
    this->is_i2c_control = rhs.is_i2c_control;
    this->is_uart_control = rhs.is_uart_control;
    this->is_usb_control = rhs.is_usb_control;
    this->is_i2c_usb_control = rhs.is_i2c_usb_control;
    this->i2c_param = rhs.i2c_param;
    this->uart_param = rhs.uart_param;
    this->usb_param = rhs.usb_param;
    return *this;
}

string control_config::to_string()
{
    stringstream ss;
    if (is_uart_control)
    {
        ss << "control use uart, com index: " << uart_param.com_index << endl;
    }
    else if (is_i2c_control)
    {
        ss << "control use i2c, dev name: " << i2c_param.dev_name << endl;
    }
    else if (is_i2c_usb_control)
    {
        ss << "control use i2c usb, " << usb_param.to_string();
    }
    else
    {
        ss << "control use usb, " << usb_param.to_string();
    }
    return ss.str();
}

string camera_config::to_string()
{
    stringstream ss;
    ss << "format: " << format_map[static_cast<int>(format)] << ", width: " << width << ", height: " << height << endl;
    ss << "image_info_height: " << image_info_height << ", info_line_height: " << info_line_height << endl;
    ss << "temp_info_height: " << temp_info_height << ", dummy_info_height: " << dummy_info_height << endl;
    ss << "image_info_ratio: " << image_info_ratio << ", info_line_ratio: " << info_line_ratio << endl;
    ss << "temp_line_ratio: " << temp_line_ratio << ", dummy_info_ratio: " << dummy_info_ratio << endl;
    ss << "is_auto_image: " << is_auto_image << ", open_temp_measure: " << open_temp_measure << endl;
    ss << "image_channel_type: " << image_channel_type << endl;
    if (v4l2_config.has_image)
    {
        ss << "v4l2_config::image_stream: " << v4l2_config.image_stream.to_string();
    }
    if (v4l2_config.has_temp)
    {
        ss << "v4l2_config::temp_stream: " << v4l2_config.temp_stream.to_string();
    }
    if (image_channel_type == "usb")
    {
        ss << "uvc_stream: " << uvc_stream_conf.to_string() << endl;
    }

    return ss.str();
}

camera_config::camera_config()
    : is_auto_image(true),
      open_temp_measure(false),
      image_channel_type("usb")
{

}

v4l2_streams::v4l2_streams()
    : has_image(false),
      has_temp(false)
{

}

camera_config& camera_config::operator= (const camera_config& rhs)
{
    this->format = rhs.format;
    this->width = rhs.width;
    this->height = rhs.height;
    this->image_info_height = rhs.image_info_height;
    this->info_line_height = rhs.info_line_height;
    this->temp_info_height = rhs.temp_info_height;
    this->dummy_info_height = rhs.dummy_info_height;
    this->image_info_ratio = rhs.image_info_ratio;
    this->info_line_ratio = rhs.info_line_ratio;
    this->temp_line_ratio = rhs.temp_line_ratio;
    this->dummy_info_ratio = rhs.dummy_info_ratio;
    this->is_auto_image = rhs.is_auto_image;
    this->open_temp_measure = rhs.open_temp_measure;
    this->image_channel_type = rhs.image_channel_type;
    this->v4l2_config = rhs.v4l2_config;
    this->uvc_stream_conf = rhs.uvc_stream_conf;
    return *this;
}

string single_config::to_string()
{
    stringstream ss;
    ss << control.to_string();
    ss << camera.to_string();
    return ss.str();
}

single_config& single_config::operator= (const single_config& rhs)
{
    this->camera = rhs.camera;
    this->control = rhs.control;
    return *this;
}
