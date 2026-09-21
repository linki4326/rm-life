TJ开源中关于相机的调用与调用串口的方式类似 也是对hikcamera库进行二次封装 但是在运用时比串口更巧妙一点 它用了多态 我们先看一下hikrobot.cpp中对库的二次封装
```cpp
namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  double exposure_us_;
  double gain_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData> queue_;

  int vid_, pid_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP
```
这里是常规的对hikcamera的二次封装 值得注意的是 这里对camera.cpp中的类进行了继承 为接下来的多态铺垫 转到camera.cpp这里
```cpp

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
```
这里就是对刚才的hikrobot.cpp的再一次封装 这里运用了多态 如果是迈德相机就用迈德的驱动 如果是海康的就用海康的驱动 我们这里以海康为例 
```cpp
else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }
  ```
这里在导入配置 值得注意的是```camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid)``` 这里运用了**std::make_unique**这个函数 他在实例化一个HikRobot对象的同时 并返回一个指向这个对像的独占所有权的智能指针(std::unique_ptr)ps:其实也可以用智能指针接受 这个函数是cpp14的新特性 独占所有权的智能指针则是cpp11的新特性 
```cpp

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io```
这里则使用多态 将取到的图像传给read() 再进行一个封装 
到下游的使用```camera.read(img, timestamp);``` 这样就实现了一次完整的取图流程