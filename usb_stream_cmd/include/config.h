#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <map>
#include <vector>

using std::string;
using std::map;
using std::vector;

typedef struct cJSON cJSON;

struct v4l2_stream {
    string to_string();

    string device_name;  // /dev/videox
    string type;         // image or temp
    int fps;
    int dev_width;
    int dev_height;
};

struct v4l2_streams {
    v4l2_streams();
    v4l2_stream image_stream;
    v4l2_stream temp_stream;
    bool        has_image;
    bool        has_temp;
};

struct usb_dev_info {
    string to_string();
    unsigned int pid;
    unsigned int vid;
    unsigned int same_id;
};

struct uvc_stream {
    string to_string();

    usb_dev_info dev_info;
    int width;
    int height;
    int fps;
    float frame_size_ratio;
};

struct control_config {
    control_config();
    control_config& operator=(const control_config& rhs);
    string to_string();
    struct uart_param {
        unsigned int com_index;
    };
    struct i2c_param {
        string dev_name;
    };

    bool is_uart_control;
    bool is_i2c_control;
    bool is_i2c_usb_control;
    bool is_usb_control;
    usb_dev_info  usb_param;
    i2c_param     i2c_param;
    uart_param    uart_param;
};

struct camera_config {
    string to_string();
    camera_config();
    camera_config& operator= (const camera_config& rhs);
    int format;
    int width;
    int height;
    int image_info_height;
    int info_line_height;
    int temp_info_height;
    int dummy_info_height;
    float image_info_ratio;
    float info_line_ratio;
    float temp_line_ratio;
    float dummy_info_ratio;
    bool is_auto_image;
    bool open_temp_measure;

    string image_channel_type;
    v4l2_streams v4l2_config;
    uvc_stream uvc_stream_conf;
};

struct single_config {
    string  to_string();
    single_config& operator= (const single_config& rhs);
    control_config control;
    camera_config  camera;
};

class config {
public:
    config();
    int parse_config(const char* config_file_path);
    bool get_config(single_config& sc);
    bool get_config(single_config& sc, const string& product);
private:
    void init_frame_output_format_dict();
    int  parse_config_json(const char* data, size_t data_length);
    int  parse_single_config(const cJSON* json);
    int  parse_control_config(const cJSON* json, control_config& control);
    int  parse_camera_config(const cJSON* json, camera_config& data);
    int  parse_v4l2_stream_config(const cJSON* json, v4l2_streams& v4l2_stream_config);
    int  parse_uvc_stream_config(const cJSON* json, uvc_stream& uvc_stream_config);
    int  parse_uvc_dev_info(const cJSON* json, usb_dev_info& dev_info);
private:
    map<string, int> frame_output_format_dict;
    map<string, single_config> product_config;
};
#endif // __CONFIG_H__