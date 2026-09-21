linux内核原生自带对CAN的接口 TJ开源代码中xockdtcan.hpp只是将这些接口再次进行封装 它把打开CAN接口 收发数据 断线重联等接口封装成一个类 然后把一部分功能的实现放在类的构造函数中 这个跟之前学的serial库差不多 
```cpp
namespace io
{
class SocketCAN
{
public:
  SocketCAN(const std::string & interface, std::function<void(const can_frame & frame)> rx_handler)
  : interface_(interface),
    socket_fd_(-1),
    epoll_fd_(-1),
    rx_handler_(rx_handler),
    quit_(false),
    ok_(false)
  {
    try_open();

    // 守护线程
    daemon_thread_ = std::thread{[this] {
      while (!quit_) {
        std::this_thread::sleep_for(100ms);

        if (ok_) continue;

        if (read_thread_.joinable()) read_thread_.join();

        close();
        try_open();
      }
    }};
  }
```
这里通过ai可以知道 这有两个线程 一个负责打开CAN口 还有一个负责断线重联(确实NB) 我们再看一下这个类具体的使用
```cpp
can_(read_yaml(config_path), std::bind(&CBoard::callback, this, std::placeholders::_1))
```
```cpp
void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 8;
  frame.data[0] = (command.control) ? 1 : 0;
  frame.data[1] = (command.shoot) ? 1 : 0;
  frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
  frame.data[3] = (int16_t)(command.yaw * 1e4);
  frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
  frame.data[5] = (int16_t)(command.pitch * 1e4);
  frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
  frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

  try {
    can_.write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}
```
 这里CBOard这个类对刚才的COCKETCAN类进行了二次封装 具体的应用则到了src层 
 ``` io::CBoard cboard(config_path);```
 ```cboard.send(command)```
这里就是对CAN串口的最终的通讯 还是不得不感叹TJ的封装真的好NB