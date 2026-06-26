'''
实验名称：颜色识别
实验平台：01Studio CanMV K230
教程：wiki.01studio.cc
'''
import time, os, sys
from media.sensor import * #导入sensor模块，使用摄像头相关接口
from media.display import * #导入display模块，使用display相关接口
from media.media import * #导入media模块，使用meida相关接口
from machine import UART #导入串口模块
from machine import FPIOA, Pin
# 颜色识别阈值 (L Min, L Max, A Min, A Max, B Min, B Max) LAB模型
# 下面的阈值元组是用来识别 红、绿、蓝三种颜色，当然你也可以调整让识别变得更好。
thresholds = [(0, 100, 24, 126, -24, 127), # 红色阈值 (30, 100, 15, 127, 15, 127)
              (42, 92, -23, 55, 31, 127), # 黄色阈值
              (2, 22, -8, 4, -10, 15),#黑色
              (27, 65, -33, 33, -128, -31)] # 蓝色阈值(9, 100, -26, 127, -128, -17)

colors1 = [(255,0,0), (255,255,0), (0,0,0), (0,0,255)]
colors2 = ['RED', 'YELLOW', 'BLACK','BLUE']
color = 3 # 取0or3，表红色和蓝色，初始抓取颜色
mode = color # 取0,1,2,3,表球真实色彩
precs = 0
#面积约束
ball_max = 260
gate_min = 260
#串口标志
r_flag = b'\x03'
y_flag = b'\x01'
B_flag = b'\x02'
b_flag = b'\x04'
send_flag = 0
#初始化串口
fpioa = FPIOA()
# UART1代码
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)
uart=UART(UART.UART1,115200) #设置串口号1和波特率
# 初始化按键
fpioa = FPIOA()
fpioa.set_function(21, FPIOA.GPIO21)
button = Pin(21, Pin.IN, Pin.PULL_UP)
number = 0  # 图片编号
#摄像头初始化
sensor = Sensor() #构建摄像头对象
sensor.reset() #复位和初始化摄像头
sensor.set_framesize(width=800, height=480) #设置帧大小为LCD分辨率(800x480)，默认通道0
sensor.set_pixformat(Sensor.RGB565) #设置输出图像格式，默认通道0
Display.init(Display.ST7701, to_ide=True) #同时使用3.5寸mipi屏和IDE缓冲区显示图像，800x480分辨率
#Display.init(Display.VIRT, sensor.width(), sensor.height()) #只使用IDE缓冲区显示图像
MediaManager.init() #初始化media资源管理器
sensor.run() #启动sensor
clock = time.clock() #定义时钟

def img_save(img):
    global number
    if button.value() == 0:  # 按键按下
            time.sleep_ms(10)  # 消抖
            if button.value() == 0:
                img.save(f"/sdcard/img_{number}.jpg")  # 保存图片
                number += 1
                print(f"图片已保存为 /sdcard/img_{number}.jpg")
def single_send(x):
    num = x
    high_byte = (num >> 8) & 0xFF
    low_byte = num & 0xFF
    uart.write(bytes([high_byte, low_byte])) #{b.cx()}

def send(x1,y1,x2,y2,ins = 0):
    uart.write("a")
    uart.write("a")
    Num = [x1,y1,x2,y2]
    for j in range(4):
        single_send(Num[j])
    single_send(ins)
def send_data(x1,y1,x2,y2,ins1 = 0,ins2 = 0):
    global send_flag
    uart.write("a")
    uart.write("a")
    Num = [x1,y1,x2,y2]
    for j in range(4):
        single_send(Num[j])
    single_send(ins1)
    single_send(ins2)
    if(send_flag):send_flag = 0
def detect(cs = 0,cl = 0):#cs只能取0或3，表红色和蓝色,cl表球真实色彩取0,1,2,3
    global precs
    p = {}
    q = {}
    qarea = {}
    ins = 0
    bx = 0
    by = 0
    qx = 0
    qy = 0
    if cl == 0 or cl == 3:
        cs = cl
    else: 
        cs = precs
    precs = cs
    blobs = img.find_blobs([thresholds[cs]],area_threshold=500,pixels_threshold=10) # 0,1,2分别表示红，绿，蓝色。
    if blobs:
        for b in blobs: #画矩形、箭头和字符表示
            img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cs])
            area_str = f"长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
            img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cs])
            #print("坐标:",b.cx(),b.cy(),"面积:",b.area())
            if b.w() < ball_max and b.h() < ball_max and 0.8<b.w()/b.h()<1.2:
                p[b.cy()] = b.cx()
            if b.w() > gate_min or b.h() > gate_min:
                q[b.cy()] = b.cx()
                qarea[b.cy()] = b.area()
        if len(p):
            by = max(p.keys())
            bx = p[by]
        if len(q):
            qy = max(q.keys())
            qx = q[qy]
            A = qarea[qy]
            if(A/384000>0.6):
                ins = 1
    if cl == 1 or cl == 2:
        p = {}  
        bx = 0
        by = 0
        blobs = img.find_blobs([thresholds[cl]],area_threshold=500,pixels_threshold=10) # 0,1,2分别表示红，绿，蓝色。
        if blobs:
            for b in blobs: #画矩形、箭头和字符表示
                img.draw_rectangle(b[0:4], thickness = 4, color = colors1[cl])
                area_str = f"面积:{b.area()},长宽比:{b.w()/b.h()},长宽:{b.w()},{b.h()}"
                img.draw_string_advanced(b[0], b[1]-35, 30,area_str,color = colors1[cl])
                #print("坐标:",b.cx(),b.cy(),"面积:",b.area())
                if b.w() < ball_max and b.h() < ball_max and 0.8<b.w()/b.h()<1.2:
                    p[b.cy()] = b.cx()
        if len(p):
            by = max(p.keys())
            bx = p[by]
    send_data(bx,by,qx,qy,ins1 = ins,ins2 = send_flag)


def handle_category(category):
    global mode,send_flag
    if category == r_flag:
        mode = 0
        send_flag = 1
    elif category == y_flag:
        mode = 1
        send_flag = 1
    elif category == B_flag:#黑球
        mode = 2
        send_flag = 1
    elif category == b_flag:
        mode = 3
        send_flag = 1
    else:
        pass
while True:
    clock.tick()
    
    img = sensor.snapshot() #拍摄一张图片
    text = uart.read(1)
    if text:
        handle_category(text)
        print(text)
    img_save(img)
    detect(cs = color,cl = mode)
    img.draw_string_advanced(0, 0, 30, 'FPS: '+str("%.3f"%(clock.fps())), color = (255, 255, 255))
    Display.show_image(img) #显示图片
    #print(clock.fps()) #打印FPS

