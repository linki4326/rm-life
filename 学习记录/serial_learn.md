说实话 最开始去网上找串口通讯的代码的时候一直没找到这个库 直到问了ai才指导这是第三方写的库 这波马了 应该一开始先去看看这个库的头文件 这个库的作者在头文件里就署名了 然后我就去看了下TJ代码中对这个库的使用 
```
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}
```

对这个库最直接的引用一个是封装在io的云台代码里面 还有一个是就是封装在达妙imu文件里 都是直接封装在构造函数当中 然后在实际的task中 自瞄只需要实例化一个类 即可实现指定端口并打开端口 
```
#include <iostream>
#include <serial/serial.h>

int main()
{
    // 打开串口
    serial::Serial serial("COM1", 9600, serial::Timeout::simpleTimeout(1000));
    if (!serial.isOpen())
    {
        std::cout << "serial open failed" << std::endl;
        return -1;
    }
    // 读取数据
    while(1)
    {
        int len = serial.available();//创建缓冲区 还没有被读取
        if (len > 0)
        {
            std::string data = serial.read(len);//直到这一步才被读取
            std::cout << "read data: " << data << std::endl;
        }
    }
    return 0;
}
```
通过博客搜索 这是最基础的串口接收数据的代码

```
#include "serial/serial.h"
#include <iostream>
#include <chrono>
#include <thread>

struct auto_aim{
    float pitch;
    float yaw;
    uint8_t control;
    uint8_t fire;
}__attribute__((packed));

auto_aim vgd = {1.2,3.0,0,1};

int main(){
    std::cout<<"hello serial"<<std::endl;
    serial::Serial serial;//实例化一个串口对象
    serial.setPort("/dev/ttyACM0");
    serial.setBaudrate(115200);

    try{
        serial.open();//打开串口
        while (true)
        {
            serial.write(reinterpret_cast<uint8_t*>(&vgd),sizeof(vgd));
            std::this_thread::sleep_for(std::chrono::seconds(5));//每5秒发送一次
        }
    }catch (const std::exception & e){
            std::cerr << e.what() << std::endl;
        }

    return 0;
}
```
这是rh写的关于串口发送数据的代码
