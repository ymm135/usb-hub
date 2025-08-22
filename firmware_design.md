# USB3.0 HUB 固件设计文档

## 1. 固件架构概述

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                   应用层 (Application Layer)              │
├─────────────────────────────────────────────────────────┤
│  设备管理  │  授权控制  │  状态监控  │  通信协议  │  日志记录  │
├─────────────────────────────────────────────────────────┤
│                   中间件层 (Middleware Layer)             │
├─────────────────────────────────────────────────────────┤
│  USB协议栈 │  串口驱动  │  GPIO控制  │  定时器管理 │  存储管理  │
├─────────────────────────────────────────────────────────┤
│                   硬件抽象层 (HAL Layer)                  │
├─────────────────────────────────────────────────────────┤
│  STM32 HAL │  USB OTG  │  UART     │  GPIO      │  Timer   │
└─────────────────────────────────────────────────────────┘
```

### 1.2 主要功能模块

1. **USB设备检测模块**：检测插入的USB设备类型和信息
2. **授权控制模块**：与上位机通信，获取设备授权
3. **端口切换模块**：控制USB信号在MCU和PC之间切换
4. **通信协议模块**：处理与上位机的串口通信
5. **状态管理模块**：管理系统和端口状态
6. **安全检测模块**：识别违禁或未知设备

## 2. 开发环境配置

### 2.1 开发工具

- **IDE**: STM32CubeIDE 1.13.0+
- **HAL库**: STM32Cube HAL V1.17.0+
- **编译器**: ARM GCC
- **调试器**: ST-Link V2/V3
- **版本控制**: Git

### 2.2 项目配置

```c
// main.h - 主要配置
#define FIRMWARE_VERSION    "V1.0.0"
#define USB_PORT_COUNT      4
#define MAX_DEVICE_COUNT    4
#define UART_BAUDRATE       115200
#define SYSTEM_CLOCK_FREQ   216000000  // 216MHz

// 端口定义
#define PORT1_SWITCH_OE     GPIO_PIN_0   // PD0
#define PORT1_SWITCH_SEL    GPIO_PIN_1   // PD1
#define PORT2_SWITCH_OE     GPIO_PIN_2   // PD2
#define PORT2_SWITCH_SEL    GPIO_PIN_3   // PD3
#define PORT3_SWITCH_OE     GPIO_PIN_4   // PD4
#define PORT3_SWITCH_SEL    GPIO_PIN_5   // PD5
#define PORT4_SWITCH_OE     GPIO_PIN_6   // PD6
#define PORT4_SWITCH_SEL    GPIO_PIN_7   // PD7

// 电源控制
#define PORT1_POWER_EN      GPIO_PIN_0   // PE0
#define PORT2_POWER_EN      GPIO_PIN_1   // PE1
#define PORT3_POWER_EN      GPIO_PIN_2   // PE2
#define PORT4_POWER_EN      GPIO_PIN_3   // PE3

// 状态LED
#define PORT1_LED           GPIO_PIN_0   // PF0
#define PORT2_LED           GPIO_PIN_1   // PF1
#define PORT3_LED           GPIO_PIN_2   // PF2
#define PORT4_LED           GPIO_PIN_3   // PF3
#define COMM_LED            GPIO_PIN_4   // PF4
```

## 3. USB设备检测模块

### 3.1 设备检测流程

```c
// usb_device_detect.h
typedef enum {
    DEVICE_TYPE_UNKNOWN = 0,
    DEVICE_TYPE_KEYBOARD,
    DEVICE_TYPE_MOUSE,
    DEVICE_TYPE_STORAGE,
    DEVICE_TYPE_HID_OTHER,
    DEVICE_TYPE_AUDIO,
    DEVICE_TYPE_VIDEO,
    DEVICE_TYPE_PROHIBITED
} DeviceType_t;

typedef struct {
    uint16_t vid;           // 厂商ID
    uint16_t pid;           // 产品ID
    uint16_t device_class;  // 设备类
    uint16_t device_subclass; // 设备子类
    uint8_t  interface_class; // 接口类
    char     manufacturer[64]; // 制造商字符串
    char     product[64];      // 产品字符串
    char     serial[64];       // 序列号
    DeviceType_t type;         // 设备类型
    uint8_t  port_num;         // 端口号
    uint32_t connect_time;     // 连接时间戳
} USBDeviceInfo_t;

// 设备检测函数
HAL_StatusTypeDef USB_DetectDevice(uint8_t port_num, USBDeviceInfo_t* device_info);
DeviceType_t USB_ClassifyDevice(USBDeviceInfo_t* device_info);
bool USB_IsProhibitedDevice(USBDeviceInfo_t* device_info);
```

### 3.2 设备分类算法

```c
// usb_device_detect.c
DeviceType_t USB_ClassifyDevice(USBDeviceInfo_t* device_info) {
    // 基于设备类进行初步分类
    switch(device_info->device_class) {
        case 0x03: // HID设备
            if(device_info->interface_class == 0x01) {
                return DEVICE_TYPE_KEYBOARD;
            } else if(device_info->interface_class == 0x02) {
                return DEVICE_TYPE_MOUSE;
            } else {
                return DEVICE_TYPE_HID_OTHER;
            }
            break;
            
        case 0x08: // 大容量存储设备
            return DEVICE_TYPE_STORAGE;
            break;
            
        case 0x01: // 音频设备
            return DEVICE_TYPE_AUDIO;
            break;
            
        case 0x0E: // 视频设备
            return DEVICE_TYPE_VIDEO;
            break;
            
        case 0x02: // 通信设备类（包含串口转换器）
            if(device_info->is_system_device = true;
               device_info->pid ==
               0x7523) {
                device_info->is_system_device = true;
                return DEVICE_TYPE_SERIAL_CONVERTER;
            }
        }
        
        // 基于设备类进行初步分类
        switch(device_info->device_class) {
            case 0x03: // HID设备
                if(device_info->interface_class == 0x01) {
                    return DEVICE_TYPE_KEYBOARD;
                } else if(device_info->interface_class == 0x02) {
                    return DEVICE_TYPE_MOUSE;
                } else {
                    return DEVICE_TYPE_HID_OTHER;
                }
                break;
                
            case 0x08: // 大容量存储设备
                return DEVICE_TYPE_STORAGE;
                break;
                
            case 0x01: // 音频设备
                return DEVICE_TYPE_AUDIO;
                break;
                
            case 0x0E: // 视频设备
                return DEVICE_TYPE_VIDEO;
                break;
                
            case 0x02: // 通信设备类（包含串口转换器）
                return DEVICE_TYPE_SERIAL_CONVERTER;
                break;
                
            default:
                // 检查是否为违禁设备
                if(USB_IsProhibitedDevice(device_info)) {
                    return DEVICE_TYPE_PROHIBITED;
                }
                return DEVICE_TYPE_UNKNOWN;
        }
    }
}

// 系统设备检测
bool USB_IsSystemDevice(uint8_t port_num) {
    return (port_num == 5); // 端口5为系统保留端口
}

// 违禁设备检测
bool USB_IsProhibitedDevice(USBDeviceInfo_t* device_info) {
    // 系统设备不进行违禁检测
    if(device_info->is_system_device) {
        return false;
    }
    
    // 违禁设备VID/PID黑名单
    const uint32_t prohibited_list[] = {
        0x04B31234, // 示例：某个违禁设备的VID:PID
        0x05AC5678, // 示例：另一个违禁设备
        // 可根据需要添加更多
    };
    
    uint32_t device_id = (device_info->vid << 16) | device_info->pid;
    
    for(int i = 0; i < sizeof(prohibited_list)/sizeof(uint32_t); i++) {
        if(device_id == prohibited_list[i]) {
            return true;
        }
    }
    
    // 检查制造商字符串中的关键词
    const char* prohibited_keywords[] = {
        "BadUSB", "Malware", "Hacker", NULL
    };
    
    for(int i = 0; prohibited_keywords[i] != NULL; i++) {
        if(strstr(device_info->manufacturer, prohibited_keywords[i]) != NULL) {
            return true;
        }
    }
    
    return false;
}
```

## 4. 授权控制模块

### 4.1 授权状态机

```c
// auth_control.h
typedef enum {
    AUTH_STATE_IDLE = 0,
    AUTH_STATE_DEVICE_DETECTED,
    AUTH_STATE_REQUEST_SENT,
    AUTH_STATE_WAITING_RESPONSE,
    AUTH_STATE_AUTHORIZED,
    AUTH_STATE_DENIED,
    AUTH_STATE_TIMEOUT,
    AUTH_STATE_SYSTEM_AUTO_APPROVED  // 新增：系统设备自动批准
} AuthState_t;

typedef struct {
    uint8_t port_num;
    USBDeviceInfo_t device_info;
    AuthState_t state;
    uint32_t request_time;
    uint32_t timeout_ms;
    bool auto_approve_keyboard;
    bool auto_approve_mouse;
    bool is_system_port;             // 新增：系统端口标志
} AuthContext_t;

// 授权控制函数
HAL_StatusTypeDef Auth_RequestPermission(uint8_t port_num, USBDeviceInfo_t* device_info);
HAL_StatusTypeDef Auth_ProcessResponse(uint8_t port_num, bool approved);
void Auth_StateMachine(void);
void Auth_InitSystemPort(void);      // 新增：初始化系统端口
```

### 4.2 授权流程实现

```c
// auth_control.c
static AuthContext_t auth_contexts[USB_PORT_COUNT];

// 初始化系统端口
void Auth_InitSystemPort(void) {
    AuthContext_t* ctx = &auth_contexts[4]; // 端口5（索引4）
    ctx->port_num = 5;
    ctx->is_system_port = true;
    ctx->state = AUTH_STATE_SYSTEM_AUTO_APPROVED;
    
    // 端口5始终保持授权状态
    Port_SwitchToPC(5); // 实际上端口5没有切换功能，直接连接
    Port_SetLED(5, LED_STATE_ON);
}

HAL_StatusTypeDef Auth_RequestPermission(uint8_t port_num, USBDeviceInfo_t* device_info) {
    if(port_num > USB_PORT_COUNT) return HAL_ERROR;
    
    AuthContext_t* ctx = &auth_contexts[port_num - 1];
    
    // 保存设备信息
    memcpy(&ctx->device_info, device_info, sizeof(USBDeviceInfo_t));
    ctx->port_num = port_num;
    ctx->request_time = HAL_GetTick();
    ctx->timeout_ms = 30000; // 30秒超时
    
    // 端口5系统设备自动批准
    if(port_num == 5 || device_info->is_system_device) {
        ctx->state = AUTH_STATE_SYSTEM_AUTO_APPROVED;
        return HAL_OK; // 端口5无需切换，直接返回
    }
    
    // 检查自动批准规则（仅适用于用户端口1-4）
    if((device_info->type == DEVICE_TYPE_KEYBOARD && ctx->auto_approve_keyboard) ||
       (device_info->type == DEVICE_TYPE_MOUSE && ctx->auto_approve_mouse)) {
        ctx->state = AUTH_STATE_AUTHORIZED;
        return Port_SwitchToPC(port_num);
    }
    
    // 发送授权请求到上位机
    ctx->state = AUTH_STATE_REQUEST_SENT;
    return Comm_SendAuthRequest(port_num, device_info);
}

void Auth_StateMachine(void) {
    for(int i = 0; i < USB_PORT_COUNT; i++) {
        AuthContext_t* ctx = &auth_contexts[i];
        uint8_t port_num = i + 1;
        
        switch(ctx->state) {
            case AUTH_STATE_REQUEST_SENT:
                ctx->state = AUTH_STATE_WAITING_RESPONSE;
                break;
                
            case AUTH_STATE_WAITING_RESPONSE:
                if(HAL_GetTick() - ctx->request_time > ctx->timeout_ms) {
                    ctx->state = AUTH_STATE_TIMEOUT;
                    if(port_num <= USER_PORT_COUNT) { // 仅用户端口需要切换
                        Port_SwitchToMCU(port_num);
                    }
                }
                break;
                
            case AUTH_STATE_AUTHORIZED:
                if(port_num <= USER_PORT_COUNT) { // 仅用户端口需要切换
                    Port_SwitchToPC(port_num);
                }
                Port_SetLED(port_num, LED_STATE_ON);
                ctx->state = AUTH_STATE_IDLE;
                break;
                
            case AUTH_STATE_DENIED:
                if(port_num <= USER_PORT_COUNT) { // 仅用户端口需要切换
                    Port_SwitchToMCU(port_num);
                }
                Port_SetLED(port_num, LED_STATE_BLINK_FAST);
                ctx->state = AUTH_STATE_IDLE;
                break;
                
            case AUTH_STATE_SYSTEM_AUTO_APPROVED:
                // 系统端口保持授权状态
                Port_SetLED(port_num, LED_STATE_ON);
                break;
                
            default:
                break;
        }
    }
}
```

## 5. 端口切换控制模块

### 5.1 端口控制接口

```c
// port_control.h
typedef enum {
    PORT_MODE_MCU = 0,  // 连接到MCU
    PORT_MODE_PC = 1    // 连接到PC
} PortMode_t;

typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_ON,
    LED_STATE_BLINK_SLOW,
    LED_STATE_BLINK_FAST
} LEDState_t;

typedef struct {
    uint8_t port_num;
    PortMode_t mode;
    bool power_enabled;
    bool has_switch_control;         // 新增：是否有切换控制
    bool is_system_port;             // 新增：是否为系统端口
    LEDState_t led_state;
    uint32_t led_last_toggle;
} PortStatus_t;

// 端口控制函数
HAL_StatusTypeDef Port_SwitchToMCU(uint8_t port_num);
HAL_StatusTypeDef Port_SwitchToPC(uint8_t port_num);
HAL_StatusTypeDef Port_SetPower(uint8_t port_num, bool enable);
HAL_StatusTypeDef Port_SetLED(uint8_t port_num, LEDState_t state);
PortStatus_t* Port_GetStatus(uint8_t port_num);
void Port_UpdateLEDs(void);
```

### 5.2 端口控制实现

```c
// port_control.c
static PortStatus_t port_status[USB_PORT_COUNT];

HAL_StatusTypeDef Port_SwitchToMCU(uint8_t port_num) {
    if(port_num >= USB_PORT_COUNT) return HAL_ERROR;
    
    // 设置切换器：OE=1, SEL=0 (连接到MCU)
    HAL_GPIO_WritePin(port_gpio_map[port_num].switch_oe_port, 
                      port_gpio_map[port_num].switch_oe_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port_gpio_map[port_num].switch_sel_port, 
                      port_gpio_map[port_num].switch_sel_pin, GPIO_PIN_RESET);
    
    port_status[port_num].mode = PORT_MODE_MCU;
    port_status[port_num].last_activity = HAL_GetTick();
    
    return HAL_OK;
}

HAL_StatusTypeDef Port_SwitchToPC(uint8_t port_num) {
    if(port_num >= USB_PORT_COUNT) return HAL_ERROR;
    
    // 设置切换器：OE=1, SEL=1 (连接到PC)
    HAL_GPIO_WritePin(port_gpio_map[port_num].switch_oe_port, 
                      port_gpio_map[port_num].switch_oe_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port_gpio_map[port_num].switch_sel_port, 
                      port_gpio_map[port_num].switch_sel_pin, GPIO_PIN_SET);
    
    port_status[port_num].mode = PORT_MODE_PC;
    port_status[port_num].last_activity = HAL_GetTick();
    
    return HAL_OK;
}

HAL_StatusTypeDef Port_SetPower(uint8_t port_num, bool enable) {
    if(port_num >= USB_PORT_COUNT) return HAL_ERROR;
    
    HAL_GPIO_WritePin(port_gpio_map[port_num].power_port, 
                      port_gpio_map[port_num].power_pin, 
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    port_status[port_num].power_enabled = enable;
    
    return HAL_OK;
}

void Port_UpdateLEDs(void) {
    static uint32_t last_blink_time = 0;
    static bool blink_state = false;
    uint32_t current_time = HAL_GetTick();
    
    // 500ms闪烁周期
    if(current_time - last_blink_time > 500) {
        blink_state = !blink_state;
        last_blink_time = current_time;
    }
    
    for(int i = 0; i < USB_PORT_COUNT; i++) {
        GPIO_PinState pin_state = GPIO_PIN_RESET;
        
        switch(port_status[i].led_state) {
            case LED_STATE_ON:
                pin_state = GPIO_PIN_SET;
                break;
            case LED_STATE_BLINK_SLOW:
                pin_state = blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET;
                break;
            case LED_STATE_BLINK_FAST:
                pin_state = (current_time % 200 < 100) ? GPIO_PIN_SET : GPIO_PIN_RESET;
                break;
            default:
                pin_state = GPIO_PIN_RESET;
                break;
        }
        
        HAL_GPIO_WritePin(port_gpio_map[i].led_port, 
                          port_gpio_map[i].led_pin, pin_state);
    }
}
```

## 6. 通信协议模块

### 6.1 协议格式定义

```c
// comm_protocol.h
#define COMM_FRAME_HEADER    0xAA55
#define COMM_FRAME_TAIL      0x55AA
#define COMM_MAX_DATA_LEN    256

typedef enum {
    CMD_DEVICE_INFO = 0x01,      // 设备信息上报
    CMD_AUTH_REQUEST = 0x02,     // 授权请求
    CMD_AUTH_RESPONSE = 0x03,    // 授权响应
    CMD_PORT_STATUS = 0x04,      // 端口状态查询
    CMD_PORT_CONTROL = 0x05,     // 端口控制
    CMD_SYSTEM_INFO = 0x06,      // 系统信息查询
    CMD_CONFIG_SET = 0x07,       // 配置设置
    CMD_LOG_DATA = 0x08,         // 日志数据
    CMD_HEARTBEAT = 0x09         // 心跳包
} CommandType_t;

typedef struct __packed {
    uint16_t header;             // 帧头 0xAA55
    uint8_t  cmd;                // 命令类型
    uint8_t  seq;                // 序列号
    uint16_t data_len;           // 数据长度
    uint8_t  data[COMM_MAX_DATA_LEN]; // 数据内容
    uint16_t checksum;           // 校验和
    uint16_t tail;               // 帧尾 0x55AA
} CommFrame_t;

// 通信函数
HAL_StatusTypeDef Comm_SendFrame(CommFrame_t* frame);
HAL_StatusTypeDef Comm_SendDeviceInfo(uint8_t port_num, USBDeviceInfo_t* device_info);
HAL_StatusTypeDef Comm_SendAuthRequest(uint8_t port_num, USBDeviceInfo_t* device_info);
void Comm_ProcessReceivedData(uint8_t* data, uint16_t len);
```

### 6.2 协议处理实现

```c
// comm_protocol.c
static uint8_t tx_buffer[512];
static uint8_t rx_buffer[512];
static uint16_t rx_index = 0;
static uint8_t frame_seq = 0;

HAL_StatusTypeDef Comm_SendDeviceInfo(uint8_t port_num, USBDeviceInfo_t* device_info) {
    CommFrame_t frame;
    
    frame.header = COMM_FRAME_HEADER;
    frame.cmd = CMD_DEVICE_INFO;
    frame.seq = ++frame_seq;
    
    // 构造设备信息数据
    uint8_t* data_ptr = frame.data;
    *data_ptr++ = port_num;
    memcpy(data_ptr, &device_info->vid, 2); data_ptr += 2;
    memcpy(data_ptr, &device_info->pid, 2); data_ptr += 2;
    *data_ptr++ = device_info->type;
    
    // 字符串信息
    uint8_t str_len = strlen(device_info->manufacturer);
    *data_ptr++ = str_len;
    memcpy(data_ptr, device_info->manufacturer, str_len);
    data_ptr += str_len;
    
    str_len = strlen(device_info->product);
    *data_ptr++ = str_len;
    memcpy(data_ptr, device_info->product, str_len);
    data_ptr += str_len;
    
    frame.data_len = data_ptr - frame.data;
    frame.checksum = Comm_CalculateChecksum(&frame);
    frame.tail = COMM_FRAME_TAIL;
    
    return Comm_SendFrame(&frame);
}

HAL_StatusTypeDef Comm_SendAuthRequest(uint8_t port_num, USBDeviceInfo_t* device_info) {
    CommFrame_t frame;
    
    frame.header = COMM_FRAME_HEADER;
    frame.cmd = CMD_AUTH_REQUEST;
    frame.seq = ++frame_seq;
    
    // 构造授权请求数据
    uint8_t* data_ptr = frame.data;
    *data_ptr++ = port_num;
    memcpy(data_ptr, &device_info->vid, 2); data_ptr += 2;
    memcpy(data_ptr, &device_info->pid, 2); data_ptr += 2;
    *data_ptr++ = device_info->type;
    
    frame.data_len = data_ptr - frame.data;
    frame.checksum = Comm_CalculateChecksum(&frame);
    frame.tail = COMM_FRAME_TAIL;
    
    return Comm_SendFrame(&frame);
}

void Comm_ProcessReceivedData(uint8_t* data, uint16_t len) {
    for(uint16_t i = 0; i < len; i++) {
        rx_buffer[rx_index++] = data[i];
        
        // 防止缓冲区溢出
        if(rx_index >= sizeof(rx_buffer)) {
            rx_index = 0;
        }
        
        // 检查是否接收到完整帧
        if(rx_index >= sizeof(CommFrame_t)) {
            CommFrame_t* frame = (CommFrame_t*)rx_buffer;
            
            if(frame->header == COMM_FRAME_HEADER && 
               frame->tail == COMM_FRAME_TAIL &&
               Comm_VerifyChecksum(frame)) {
                
                Comm_ProcessCommand(frame);
                rx_index = 0;
            }
        }
    }
}

void Comm_ProcessCommand(CommFrame_t* frame) {
    switch(frame->cmd) {
        case CMD_AUTH_RESPONSE:
            {
                uint8_t port_num = frame->data[0];
                bool approved = frame->data[1];
                Auth_ProcessResponse(port_num, approved);
            }
            break;
            
        case CMD_PORT_CONTROL:
            {
                uint8_t port_num = frame->data[0];
                uint8_t action = frame->data[1];
                
                switch(action) {
                    case 0: // 禁用端口
                        Port_SetPower(port_num, false);
                        Port_SwitchToMCU(port_num);
                        break;
                    case 1: // 启用端口
                        Port_SetPower(port_num, true);
                        break;
                    case 2: // 强制切换到PC
                        Port_SwitchToPC(port_num);
                        break;
                }
            }
            break;
            
        case CMD_CONFIG_SET:
            {
                // 处理配置设置
                Config_ProcessSettings(frame->data, frame->data_len);
            }
            break;
            
        default:
            break;
    }
}
```

## 7. 主程序流程

### 7.1 主函数实现

```c
// main.c
int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // 外设初始化
    GPIO_Init();
    UART_Init();
    USB_Init();
    Timer_Init();
    
    // 模块初始化
    Port_Init();
    Auth_Init();
    Comm_Init();
    
    // 系统自检
    System_SelfTest();
    
    // 主循环
    while(1) {
        // USB设备检测
        USB_DeviceDetectTask();
        
        // 授权状态机
        Auth_StateMachine();
        
        // 通信处理
        Comm_ProcessTask();
        
        // 端口状态更新
        Port_UpdateTask();
        
        // LED状态更新
        Port_UpdateLEDs();
        
        // 看门狗喂狗
        HAL_IWDG_Refresh(&hiwdg);
        
        // 系统延时
        HAL_Delay(10);
    }
}

void USB_DeviceDetectTask(void) {
    static uint32_t last_check_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每100ms检测一次
    if(current_time - last_check_time > 100) {
        for(int i = 0; i < USB_PORT_COUNT; i++) {
            USBDeviceInfo_t device_info;
            
            if(USB_DetectDevice(i, &device_info) == HAL_OK) {
                // 检测到新设备
                if(!port_status[i].device_connected) {
                    port_status[i].device_connected = true;
                    
                    // 发送设备信息
                    Comm_SendDeviceInfo(i, &device_info);
                    
                    // 请求授权
                    Auth_RequestPermission(i, &device_info);
                }
            } else {
                // 设备断开
                if(port_status[i].device_connected) {
                    port_status[i].device_connected = false;
                    Port_SwitchToMCU(i);
                    Port_SetLED(i, LED_STATE_OFF);
                }
            }
        }
        
        last_check_time = current_time;
    }
}
```

## 8. 配置管理模块

### 8.1 配置参数定义

```c
// config.h
typedef struct {
    uint32_t magic;                    // 配置魔数
    uint16_t version;                  // 配置版本
    
    // 授权设置
    bool auto_approve_keyboard;        // 自动批准键盘
    bool auto_approve_mouse;           // 自动批准鼠标
    uint32_t auth_timeout_ms;          // 授权超时时间
    
    // 通信设置
    uint32_t uart_baudrate;            // 串口波特率
    uint8_t heartbeat_interval;        // 心跳间隔(秒)
    
    // 安全设置
    bool enable_device_whitelist;      // 启用设备白名单
    bool enable_manufacturer_check;    // 启用制造商检查
    bool log_all_devices;              // 记录所有设备
    
    // 端口设置
    bool port_power_control[USB_PORT_COUNT]; // 端口电源控制
    uint32_t port_timeout_ms[USB_PORT_COUNT]; // 端口超时时间
    
    uint16_t checksum;                 // 配置校验和
} SystemConfig_t;

// 配置函数
HAL_StatusTypeDef Config_Load(void);
HAL_StatusTypeDef Config_Save(void);
HAL_StatusTypeDef Config_Reset(void);
SystemConfig_t* Config_Get(void);
```

### 8.2 Flash存储管理

```c
// config.c
#define CONFIG_FLASH_ADDR    0x08060000  // Flash扇区15
#define CONFIG_MAGIC         0x55AA55AA

static SystemConfig_t system_config;

HAL_StatusTypeDef Config_Load(void) {
    SystemConfig_t* flash_config = (SystemConfig_t*)CONFIG_FLASH_ADDR;
    
    // 检查配置有效性
    if(flash_config->magic == CONFIG_MAGIC && 
       Config_VerifyChecksum(flash_config)) {
        memcpy(&system_config, flash_config, sizeof(SystemConfig_t));
        return HAL_OK;
    } else {
        // 加载默认配置
        return Config_Reset();
    }
}

HAL_StatusTypeDef Config_Save(void) {
    HAL_StatusTypeDef status;
    
    // 计算校验和
    system_config.checksum = Config_CalculateChecksum(&system_config);
    
    // 解锁Flash
    HAL_FLASH_Unlock();
    
    // 擦除扇区
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = FLASH_SECTOR_15;
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    uint32_t sector_error;
    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    
    if(status == HAL_OK) {
        // 写入配置数据
        uint32_t* data = (uint32_t*)&system_config;
        uint32_t addr = CONFIG_FLASH_ADDR;
        
        for(int i = 0; i < sizeof(SystemConfig_t)/4; i++) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data[i]);
            if(status != HAL_OK) break;
            addr += 4;
        }
    }
    
    // 锁定Flash
    HAL_FLASH_Lock();
    
    return status;
}
```

## 9. 日志记录模块

### 9.1 日志系统设计

```c
// logger.h
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} LogLevel_t;

typedef struct {
    uint32_t timestamp;
    LogLevel_t level;
    uint8_t port_num;
    uint16_t event_code;
    char message[64];
} LogEntry_t;

// 日志函数
void Log_Init(void);
void Log_Write(LogLevel_t level, uint8_t port_num, uint16_t event_code, const char* format, ...);
void Log_SendToHost(void);
LogEntry_t* Log_GetEntries(uint16_t* count);
```

### 9.2 事件代码定义

```c
// 事件代码定义
#define EVENT_SYSTEM_START       0x0001
#define EVENT_DEVICE_CONNECTED   0x0101
#define EVENT_DEVICE_DISCONNECTED 0x0102
#define EVENT_DEVICE_AUTHORIZED  0x0103
#define EVENT_DEVICE_DENIED      0x0104
#define EVENT_DEVICE_PROHIBITED  0x0105
#define EVENT_PORT_SWITCHED      0x0201
#define EVENT_AUTH_TIMEOUT       0x0301
#define EVENT_COMM_ERROR         0x0401
#define EVENT_CONFIG_CHANGED     0x0501
```

## 10. 编译和部署

### 10.1 Makefile配置

```makefile
# Makefile
TARGET = usb_hub_firmware

# 源文件
C_SOURCES = \
src/main.c \
src/usb_device_detect.c \
src/auth_control.c \
src/port_control.c \
src/comm_protocol.c \
src/config.c \
src/logger.c \
Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
# ... 其他HAL库文件

# 编译选项
CFLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F767xx
CFLAGS += -Wall -Wextra -Og -g3

# 链接选项
LDFLAGS = -T STM32F767VITx_FLASH.ld
LDFLAGS += --specs=nano.specs -lc -lm -lnosys

# 编译规则
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin

$(TARGET).elf: $(C_SOURCES:.c=.o)
	$(CC) $(LDFLAGS) $^ -o $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: $(TARGET).hex
	st-flash write $< 0x8000000

.PHONY: all clean flash
```

### 10.2 调试配置

```json
// .vscode/launch.json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug STM32",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "stlink",
            "cwd": "${workspaceRoot}",
            "executable": "./build/usb_hub_firmware.elf",
            "device": "STM32F767VI",
            "interface": "swd",
            "runToMain": true,
            "svdFile": "./STM32F767.svd"
        }
    ]
}
```

---

**总结**：

本固件设计文档提供了完整的USB3.0 HUB控制器固件实现方案，包括：

1. **模块化设计**：清晰的模块划分和接口定义
2. **USB设备检测**：完整的设备识别和分类算法
3. **授权控制**：灵活的授权机制和状态管理
4. **通信协议**：可靠的串口通信协议
5. **配置管理**：持久化配置存储
6. **日志系统**：完整的事件记录和追踪
7. **安全机制**：设备白名单和违禁设备检测

该固件设计具有良好的可扩展性和可维护性，能够满足产品的功能需求。