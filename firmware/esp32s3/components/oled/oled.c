/**
 * @file oled.c
 * @brief SSD1306 OLED显示屏控制模块
 * 
 * 本模块实现了基于I2C总线的SSD1306 OLED显示屏驱动，主要功能包括：
 * 
 * 核心功能：
 * - 128x64像素单色OLED显示控制
 * - ASCII字符集显示支持
 * - 多行文本自动滚动显示
 * - 坐标定位和字符绘制
 * - 实时状态信息显示
 * 
 * 显示内容：
 * - 机器人基本信息（标题）
 * - WiFi连接状态和IP地址
 * - MPU6050陀螺仪实时数据
 * - 电机PID控制参数
 * - 左右电机转速反馈和目标值
 * - 机器人线速度和角速度
 * 
 * 技术特点：
 * - 基于I2C通信协议（地址0x3C）
 * - 支持6x8和8x16两种ASCII字体
 * - FreeRTOS任务驱动的周期性更新
 * - 内置ASCII字符点阵数据
 * 
 * @version 1.0
 * @date 2024
 */

#include "oled.h"

// ========================= 模块配置和全局变量 =========================

static const char *TAG = "OLED";

#define OLED_MAX_LINES 10        ///< OLED最大显示行数
#define OLED_START_LINE 1        ///< 自动滚动显示起始行

static uint8_t last_line_ = 2;   ///< 自动行显示的当前行位置

// ========================= ASCII字符点阵数据 =========================

/**
 * @brief ASCII字符点阵数据表 (6x8像素)
 * 
 * 包含95个可打印ASCII字符的点阵数据，字符范围从空格(32)到波浪号(126)：
 * - 每个字符占用6个字节，代表6列像素数据
 * - 每个字节的8个位代表该列的8行像素
 * - 位值1表示点亮，0表示熄灭
 * 
 * 字符映射表：
 * 空格(32) !(33) "(34) #(35) $(36) %(37) &(38) '(39) ((40) )(41) *(42) +(43) ,(44) -(45) .(46)
 * /(47) 0(48) 1(49) 2(50) 3(51) 4(52) 5(53) 6(54) 7(55) 8(56) 9(57) :(58) ;(59)
 * <(60) =(61) >(62) ?(63)
 * @(64) A(65) B(66) C(67) D(68) E(69) F(70) G(71) H(72) I(73) J(74) K(75) L(76)
 * M(77) N(78) O(79) P(80) Q(81) R(82) S(83) T(84) U(85) V(86) W(87) X(88) Y(89)
 * Z(90) [(91) \(92) ](93) ^(94) _(95)
 * `(96) a(97) b(98) c(99) d(100) e(101) f(102) g(103) h(104) i(105) j(106) k(107) l(108)
 * m(109) n(110) o(111) p(112) q(113) r(114) s(115) t(116) u(117) v(118) w(119) x(120) y(121)
 * z(122) {(123) |(124) }(125) ~(126)
 */
unsigned char ascii[95][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // sp
    {0x00, 0x00, 0x00, 0x2f, 0x00, 0x00},  // !
    {0x00, 0x00, 0x07, 0x00, 0x07, 0x00},  // "
    {0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14},  // #
    {0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12},  // $
    {0x00, 0x62, 0x64, 0x08, 0x13, 0x23},  // %
    {0x00, 0x36, 0x49, 0x55, 0x22, 0x50},  // &
    {0x00, 0x00, 0x05, 0x03, 0x00, 0x00},  // '
    {0x00, 0x00, 0x1c, 0x22, 0x41, 0x00},  // (
    {0x00, 0x00, 0x41, 0x22, 0x1c, 0x00},  // )
    {0x00, 0x14, 0x08, 0x3E, 0x08, 0x14},  // *
    {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08},  // +
    {0x00, 0x00, 0x00, 0xA0, 0x60, 0x00},  // ,
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08},  // -
    {0x00, 0x00, 0x60, 0x60, 0x00, 0x00},  // .
    {0x00, 0x20, 0x10, 0x08, 0x04, 0x02},  // /
    {0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
    {0x00, 0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
    {0x00, 0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x00, 0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
    {0x00, 0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
    {0x00, 0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
    {0x00, 0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x00, 0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x00, 0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
    {0x00, 0x00, 0x36, 0x36, 0x00, 0x00},  // :
    {0x00, 0x00, 0x56, 0x36, 0x00, 0x00},  // ;
    {0x00, 0x08, 0x14, 0x22, 0x41, 0x00},  // <
    {0x00, 0x14, 0x14, 0x14, 0x14, 0x14},  // =
    {0x00, 0x00, 0x41, 0x22, 0x14, 0x08},  // >
    {0x00, 0x02, 0x01, 0x51, 0x09, 0x06},  // ?
    {0x00, 0x32, 0x49, 0x59, 0x51, 0x3E},  // @
    {0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C},  // A
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x36},  // B
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x22},  // C
    {0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
    {0x00, 0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x01},  // F
    {0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A},  // G
    {0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F},  // H
    {0x00, 0x00, 0x41, 0x7F, 0x41, 0x00},  // I
    {0x00, 0x20, 0x40, 0x41, 0x3F, 0x01},  // J
    {0x00, 0x7F, 0x08, 0x14, 0x22, 0x41},  // K
    {0x00, 0x7F, 0x40, 0x40, 0x40, 0x40},  // L
    {0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F},  // M
    {0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F},  // N
    {0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E},  // O
    {0x00, 0x7F, 0x09, 0x09, 0x09, 0x06},  // P
    {0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E},  // Q
    {0x00, 0x7F, 0x09, 0x19, 0x29, 0x46},  // R
    {0x00, 0x46, 0x49, 0x49, 0x49, 0x31},  // S
    {0x00, 0x01, 0x01, 0x7F, 0x01, 0x01},  // T
    {0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F},  // U
    {0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F},  // V
    {0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F},  // W
    {0x00, 0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x00, 0x07, 0x08, 0x70, 0x08, 0x07},  // Y
    {0x00, 0x61, 0x51, 0x49, 0x45, 0x43},  // Z
    {0x00, 0x00, 0x7F, 0x41, 0x41, 0x00},  // [
    {0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55},  // 55
    {0x00, 0x00, 0x41, 0x41, 0x7F, 0x00},  // ]
    {0x00, 0x04, 0x02, 0x01, 0x02, 0x04},  // ^
    {0x00, 0x40, 0x40, 0x40, 0x40, 0x40},  // _
    {0x00, 0x00, 0x01, 0x02, 0x04, 0x00},  // '
    {0x00, 0x20, 0x54, 0x54, 0x54, 0x78},  // a
    {0x00, 0x7F, 0x48, 0x44, 0x44, 0x38},  // b
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x20},  // c
    {0x00, 0x38, 0x44, 0x44, 0x48, 0x7F},  // d
    {0x00, 0x38, 0x54, 0x54, 0x54, 0x18},  // e
    {0x00, 0x08, 0x7E, 0x09, 0x01, 0x02},  // f
    {0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C},  // g
    {0x00, 0x7F, 0x08, 0x04, 0x04, 0x78},  // h
    {0x00, 0x00, 0x44, 0x7D, 0x40, 0x00},  // i
    {0x00, 0x40, 0x80, 0x84, 0x7D, 0x00},  // j
    {0x00, 0x7F, 0x10, 0x28, 0x44, 0x00},  // k
    {0x00, 0x00, 0x41, 0x7F, 0x40, 0x00},  // l
    {0x00, 0x7C, 0x04, 0x18, 0x04, 0x78},  // m
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x78},  // n
    {0x00, 0x38, 0x44, 0x44, 0x44, 0x38},  // o
    {0x00, 0xFC, 0x24, 0x24, 0x24, 0x18},  // p
    {0x00, 0x18, 0x24, 0x24, 0x18, 0xFC},  // q
    {0x00, 0x7C, 0x08, 0x04, 0x04, 0x08},  // r
    {0x00, 0x48, 0x54, 0x54, 0x54, 0x20},  // s
    {0x00, 0x04, 0x3F, 0x44, 0x40, 0x20},  // t
    {0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C},  // u
    {0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C},  // v
    {0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C},  // w
    {0x00, 0x44, 0x28, 0x10, 0x28, 0x44},  // x
    {0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C},  // y
    {0x00, 0x44, 0x64, 0x54, 0x4C, 0x44},  // z
    {0x14, 0x14, 0x14, 0x14, 0x14, 0x14},  // horiz lines
};

// ========================= 基础显示控制函数 =========================

/**
 * @brief 清空OLED显示屏
 * 
 * 遍历所有8页（行），将每页的128列像素全部清零：
 * - OLED采用页面寻址模式，共8页，每页高度8像素
 * - 每页包含128列，对应屏幕宽度128像素
 * - 通过I2C发送命令设置页面地址和列地址
 * - 逐个写入0x00清空像素数据
 * 
 * I2C命令序列：
 * - 0xB0+页号：设置页面地址（0xB0-0xB7）
 * - 0x00：设置列地址低4位为0
 * - 0x10：设置列地址高4位为0
 * - 0x00：写入像素数据（全0 = 熄灭）
 */
void oled_clear(void) {
  for (int x = 0; x < 8; x++) {
    esp32_i2c_write_byte(OLED_ADDR, 0x00, 0xb0 + x);  // 设置页面地址
    esp32_i2c_write_byte(OLED_ADDR, 0x00, 0x00);      // 设置列地址低4位
    esp32_i2c_write_byte(OLED_ADDR, 0x00, 0x10);      // 设置列地址高4位
    for (int y = 0; y < 128; y++) esp32_i2c_write_byte(OLED_ADDR, 0x40, 0x00);  // 清空像素数据
  }
}

/**
 * @brief 设置OLED显示坐标
 * 
 * 在OLED屏幕上定位光标到指定的行列位置：
 * - x：列坐标（0-127），对应屏幕水平像素位置
 * - y：页坐标（0-7），对应8个显示页面
 * 
 * SSD1306地址设置命令：
 * - 页面地址：0xB0 + y，选择显示的页面（行）
 * - 列地址低4位：x的低4位，设置起始列的低位
 * - 列地址高4位：0x10 + x的高4位，设置起始列的高位
 * 
 * 坐标系统：
 * 0 ----------- 127 (x轴，列)
 * |
 * |
 * 7 (y轴，页面)
 * 
 * @param x 列坐标（0-127）
 * @param y 页面坐标（0-7）
 */
void oled_setxy(uint8_t x, uint8_t y) {
  esp32_i2c_write_byte(OLED_ADDR, 0x00, 0xb0 + y);                      // 设置页面地址
  esp32_i2c_write_byte(OLED_ADDR, 0x00, (x & 0x0f));                   // 设置列地址低4位
  esp32_i2c_write_byte(OLED_ADDR, 0x00, ((x & 0xf0) >> 4) | 0x10);     // 设置列地址高4位
}

// ========================= 文本显示函数 =========================

/**
 * @brief 自动换行显示ASCII字符串
 * 
 * 提供类似控制台的滚动显示功能，自动管理行位置：
 * - 检查字符串长度限制（最多21个字符）
 * - 先清空当前行（用空格覆盖）
 * - 在当前行显示新字符串
 * - 自动递增行计数器
 * - 超过最大行数时从起始行重新开始
 * 
 * 滚动机制：
 * - 起始行：OLED_START_LINE (1)
 * - 最大行数：OLED_MAX_LINES (10)
 * - 超出范围时循环回到起始行
 * 
 * @param str 要显示的ASCII字符串
 * @return true 显示成功
 * @return false 字符串过长，显示失败
 */
bool oled_show_ascii_auto_line(char *str) {
  if (strlen(str) > 21) {
    ESP_LOGW(TAG, "len can't >21 char!");
    return false;
  }
  oled_ascii8(0, last_line_, "                       ");  // 清空当前行（23个空格）
  oled_ascii8(0, last_line_, str);                         // 显示新字符串
  last_line_++;
  if (last_line_ > OLED_MAX_LINES) {
    last_line_ = OLED_START_LINE;  // 循环回到起始行
  }
  return true;
}

/**
 * @brief 显示ASCII字符串（16x8像素字体）
 * 
 * 显示大字体ASCII字符串，每个字符占用8x16像素：
 * - 最多显示16个字符
 * - 字符高度跨越两个OLED页面（上下两行）
 * - 先绘制字符的上半部分（前8个字节）
 * - 再绘制字符的下半部分（后8个字节）
 * 
 * 字体特点：
 * - 8x16像素双行字体，显示效果清晰
 * - 适合显示标题或重要信息
 * - 占用垂直空间较大
 * 
 * @param x 起始列坐标
 * @param y 起始页面坐标
 * @param str ASCII字符串
 */
void oled_ascii(uint8_t x, uint8_t y, char *str) {
  // 绘制字符的上半部分
  oled_setxy(x, y);
  for (int i = 0; i < 16 && (str[i] != 0); i++) {
    for (int j = 0; j < 8; j++) {
      esp32_i2c_write_byte(OLED_ADDR, 0x40, ascii[str[i] - 32][j]);
    }
  }
  // 绘制字符的下半部分
  oled_setxy(x, y + 1);
  for (int i = 0; i < 16 && (str[i] != 0); i++) {
    for (int j = 8; j < 16; j++) {
      esp32_i2c_write_byte(OLED_ADDR, 0x40, ascii[str[i] - 32][j]);
    }
  }
}

/**
 * @brief 显示ASCII字符串（6x8像素字体）
 * 
 * 显示小字体ASCII字符串，每个字符占用6x8像素：
 * - 最多显示21个字符（128像素宽度 ÷ 6像素/字符）
 * - 字符高度仅占用一个OLED页面
 * - 显示密度高，适合显示详细数据
 * 
 * 字符处理：
 * - ASCII值减去32得到字符在点阵表中的索引
 * - 逐个字符的6列像素数据写入OLED
 * 
 * 使用场景：
 * - 状态信息显示
 * - 数值数据显示
 * - 多行文本显示
 * 
 * @param x 起始列坐标
 * @param y 页面坐标
 * @param str ASCII字符串
 */
void oled_ascii8(uint8_t x, uint8_t y, char *str) {
  oled_setxy(x, y);
  for (int i = 0; i < 21 && (str[i] != 0); i++) {
    for (int j = 0; j < 6; j++) {
      esp32_i2c_write_byte(OLED_ADDR, 0x40, ascii[str[i] - 32][j]);
    }
  }
}

// ========================= 初始化函数 =========================

/**
 * @brief 初始化SSD1306 OLED显示屏
 * 
 * 配置SSD1306控制器的各项参数并启动显示：
 * 
 * 初始化命令序列说明：
 * - 0xAE：关闭显示
 * - 0x20,0x10：设置内存寻址模式为页面寻址
 * - 0xB0：设置页面起始地址为0
 * - 0xC8：设置扫描方向（COM输出从下到上）
 * - 0x00,0x10：设置列起始地址为0
 * - 0x40：设置显示起始行为0
 * - 0x81,0x7F：设置对比度为127（中等亮度）
 * - 0xA1：设置段重新映射（左右翻转）
 * - 0xA6：设置正常显示（非反色）
 * - 0xA8,0x3F：设置多路复用比为63（64行显示）
 * - 0xA4：全屏显示开启，跟随RAM内容
 * - 0xD3,0x00：设置显示偏移为0
 * - 0xD5,0xF0：设置显示时钟分频比和振荡器频率
 * - 0xD9,0x22：设置预充电周期
 * - 0xDA,0x12：设置COM引脚硬件配置
 * - 0xDB,0x20：设置VCOMH电压
 * - 0x8D,0x14：启用电荷泵
 * - 0xAF：开启显示
 * 
 * 初始化后操作：
 * - 清空显示屏
 * - 显示项目标题"ros1_car"
 * 
 * @return true 初始化成功
 */
bool oled_init() {
  // SSD1306初始化命令序列
  uint8_t cmd_data[] = {
    0xae,       // 关闭显示
    0x20, 0x10, // 设置内存寻址模式为页面寻址
    0xb0,       // 设置页面起始地址
    0xc8,       // 设置COM扫描方向
    0x00, 0x10, // 设置列起始地址
    0x40,       // 设置显示起始行
    0x81, 0x7f, // 设置对比度
    0xa1,       // 设置段重新映射
    0xa6,       // 设置正常显示模式
    0xa8, 0x3f, // 设置多路复用比
    0xa4,       // 全屏显示开启
    0xd3, 0x00, // 设置显示偏移
    0xd5, 0xf0, // 设置显示时钟
    0xd9, 0x22, // 设置预充电周期
    0xda, 0x12, // 设置COM引脚配置
    0xdb, 0x20, // 设置VCOMH电压
    0x8d, 0x14, // 启用电荷泵
    0xaf        // 开启显示
  };
  
  // 发送初始化命令
  esp32_i2c_write_bytes(OLED_ADDR, 0x00, 28, cmd_data);
  
  // 清空屏幕并显示标题
  oled_clear();
  oled_ascii8(0, 0, "     ros1_car     ");
  
  ESP_LOGI(TAG, "oled init success!");
  return true;
}

// ========================= 状态显示任务 =========================

/**
 * @brief OLED状态显示任务主函数
 * 
 * 运行在独立的FreeRTOS任务中，周期性更新OLED显示内容：
 * 
 * 显示布局：
 * - 第0行：项目标题"ros1_car"
 * - 第1行：WiFi状态（在main.c中设置）
 * - 第2行：IP地址（在main.c中设置）
 * - 第3行：陀螺仪Z轴 + 左右电机转速概览
 * - 第4行：机器人整体运动状态（线速度 + 角速度）
 * - 第5行：电机状态对比（左右电机转速）
 * - 第6行：PID控制参数
 * - 第7行：系统运行状态（运行时间 + 电机平衡度）
 * 
 * 更新频率：每100ms更新一次（10Hz刷新率）
 * 
 * 数据源：
 * - MPU6050陀螺仪数据（校准后）
 * - 电机控制器实时状态
 * - PID参数设置
 * 
 * @param pvPara 全局上下文指针（context_pack_t类型）
 */
void oled_show(void* pvPara)
{
  context_pack_t *ctx = (context_pack_t*)pvPara;
  char buf[25];  // 显示缓冲区

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms更新周期

    // ========== 获取电机转速数据 ==========
    float cur_rpm_l = mc_get_real_rpm(ctx->mc.left);
    float cur_rpm_r = mc_get_real_rpm(ctx->mc.right);
    
    // ========== 计算机器人整体运动状态 ==========
    // 考虑硬件抽象层：左轮对向安装需要取反
    float actual_left_rpm = -cur_rpm_l;   
    float actual_right_rpm = cur_rpm_r;   
    
    // 单位转换：mm -> m
    float wheel_perimeter_m = ctx->wheel_perimeter / 1000.0f;
    float wheel_space_m = ctx->wheel_space / 1000.0f;
    
    // RPM转换为线速度 (m/s)
    float left_speed_ms = (actual_left_rpm / 60.0f) * wheel_perimeter_m;
    float right_speed_ms = (actual_right_rpm / 60.0f) * wheel_perimeter_m;
    
    // 差速驱动运动学：计算机器人整体速度
    float robot_linear_velocity = (right_speed_ms + left_speed_ms) / 2.0f;     // 线速度 (m/s)
    float robot_angular_velocity = (right_speed_ms - left_speed_ms) / wheel_space_m; // 角速度 (rad/s)
    
    // ========== IMU陀螺仪数据显示 ==========
    // 显示Z轴陀螺仪数据，用于转向状态监控
    int16_t gyro_z = ctx->mpu6050.curr_gyro.raw_gyro_z - ctx->mpu6050.gyro_cal.raw_gyro_z;
    snprintf(buf, sizeof(buf), "GyZ:%4d ", gyro_z);
    oled_ascii8(0, 3, buf);

    // ========== 机器人整体运动状态显示 ==========
    // 显示机器人的当前线速度和角速度（这是ROS中最重要的运动参数）
    snprintf(buf, sizeof(buf), "Vel:%.3fm/s %.2frd/s", robot_linear_velocity, robot_angular_velocity);
    oled_ascii8(0, 4, buf);

    // ========== 电机状态对比显示 ==========
    // 显示左右电机的当前转速
    snprintf(buf, sizeof(buf), "Motor L:%.1f R:%.1f", cur_rpm_l, cur_rpm_r);
    oled_ascii8(0, 5, buf);
    
    // ========== PID和系统状态显示 ==========
    // 显示PID参数和系统关键状态
    float kp = 0, ki = 0, kd = 0;
    mc_get_pid_params(ctx->mc.right, &kp, &ki, &kd);
    snprintf(buf, sizeof(buf), "PID:%.1f-%.1f-%.2f", kp, ki, kd);
    oled_ascii8(0, 6, buf);


  }
}


/**
 * @brief 创建OLED显示任务
 * 
 * 启动独立的FreeRTOS任务用于周期性更新OLED显示：
 * - 任务名称："oled_show"
 * - 栈大小：8KB（8192字节）
 * - 优先级：10（高优先级，确保显示及时更新）
 * - CPU核心：0（与其他UI相关任务运行在同一核心）
 * 
 * 任务特点：
 * - 独立运行，不阻塞主程序
 * - 高优先级确保显示实时性
 * - 适中的栈空间满足字符串处理需求
 * 
 * @param ctx 全局上下文指针，传递给显示任务
 */
void oled_show_task(context_pack_t *ctx)
{
    xTaskCreatePinnedToCore(oled_show, "oled_show", 8 * 1024, ctx, 10, NULL, 0);
}