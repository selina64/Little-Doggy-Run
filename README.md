
# Little-Doggy-Run

本项目是北京理工大学世纪杯参赛作品，现将其开源，欢迎交流。

## 项目视频

[进入B站查看项目视频](https://www.bilibili.com/video/BV127411y7L2?from=search&seid=14781190684565668511)

## 项目图片

![Image text](https://raw.githubusercontent.com/Piamen/Little-Doggy-Run/master/IMG/IMG1.jpg)
![Image text](https://raw.githubusercontent.com/Piamen/Little-Doggy-Run/master/IMG/IMG2.jpg)

## 代码说明

### 开发环境及平台

#### 软件环境

IDE：MDK5  
代码生成器：STM32CUBEMX 4.26.1  
代码生成器固件版本：FW_F4 V1.21.0  
建模软件：SOLIDWORKS 2019

#### 硬件平台

主控单片机：STM32F407VE  
IMU：MPU6050  
稳压模块：LM2596HVS，共五块，每条腿一块，控制部分一块  
电池：2s航模电池  
舵机：KingMax CLS2025

#### 硬件连线

![Image text](https://raw.githubusercontent.com/Piamen/Little-Doggy-Run/master/IMG/IMG3.png)

### 代码架构

代码主要分为控制、遥控、通讯部分，使用者可以着重阅读控制部分，即*Little-Doggy-Run\Src\control_task.c*

## 机械结构

机架设计采用了玻纤板拼接式结构，经验证该结构稳定，但是过重，使用碳纤材料可以解决问题。  
腿部设计采用了五连杆机械结构，是一个两自由度的腿部。  
舵机与大腿的连接采用了柔软的热熔胶连接，原因部分是为设计方便，另一部分为了是尽量吸收大幅度的震动，使得机体运行更稳定。  
