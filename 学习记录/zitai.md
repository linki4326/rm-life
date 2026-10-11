# 四元数 & 欧拉角 & 旋转矩阵
根据[姿态](https://blog.csdn.net/dabenxiong666/article/details/52861521?ops_request_misc=elastic_search_misc&request_id=2fdcdba154a90289dd8467e95bca5038&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-52861521-null-null.142^v102^pc_search_result_base4&utm_term=%E5%A7%BF%E6%80%81&spm=1018.2226.3001.4187)可知 在三维空间中物体的姿态 旋转可以通过四元数 欧拉角 旋转矩阵来进行描述 这三个通过不同的方式描述同一个姿态 并且可以通过公式相互转换 具体可以参考这篇文章 [转换](https://blog.csdn.net/cao_hao_qi/article/details/162584071?ops_request_misc=&request_id=&biz_id=102&utm_term=%E6%97%8B%E8%BD%AC%E5%90%91%E9%87%8F&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-162584071.142^v102^pc_search_result_base4&spm=1018.2226.3001.4187) 接下来我们一一介绍一下这三种描述姿态的方式
## 欧拉角
这是最易于我们理解的一种描述旋转的方式 它将任意三维旋转分解为绕三个坐标轴的依次旋转 每个旋转对应一个角度 这三个角度统称为欧拉角 通常称为俯仰角（pitch） 滚转角（roll）和偏航角（yaw）
欧拉角分为内旋和外旋 内旋为每次旋转都绕着旋转后新的坐标轴旋转 外旋则相反 它是每次旋转都绕着固定不便的坐标轴旋转
欧拉角还讲究旋转顺序 旋转的顺序不一样 每次得到的结果就不一样 
欧拉角还有个缺点 就是会发生死锁现象 假设我们先转yaw再转pitch最后转row 如果在转pitch的时候旋转了90度 那此时row则会与之前yaw旋转的角度重叠 导致失去一个自由度 
## 四元数
四元数则不会导致这个问题 根据[四元数](https://blog.csdn.net/YuYunTan/article/details/83828258?ops_request_misc=&request_id=&biz_id=102&utm_term=%E5%9B%9B%E5%85%83%E6%95%B0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-7-83828258.142^v102^pc_search_result_base4&spm=1018.2226.3001.4187)可知 四元数相当于将一个物体只绕着一个给定的向量旋转并且一次到位 由此避开了万向节死锁的问题
## 旋转矩阵
看过3b1b的都知道 矩阵本质是一种线性变换的规则 旋转矩阵也是 同时他也能表示姿态 毕竟他们本质都是描述坐标系b相对于坐标系a的朝向 这点非常重要 这是理解后面为什么q能直接作为基变换的R的关键
## 转换
这几个数之间可以自由转换 具体参考[text](https://blog.csdn.net/cao_hao_qi/article/details/162584071?ops_request_misc=&request_id=&biz_id=102&utm_term=%E6%97%8B%E8%BD%AC%E5%90%91%E9%87%8F&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-162584071.142^v102^pc_search_result_base4&spm=1018.2226.3001.4187)
# 使用
我们可以看一下这几个数具体在代码中的使用
```cpp
Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    // recorder.record(img, q, t);

    solver.set_R_gimbal2world(q);
```
我们具体看第1 25 35行 这里定义了一个四元数用来接受云台imu传过来的四元数并传进这个将四元数转成旋转矩阵的函数
```cpp
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}
```
这是这个函数的具体定义 这里先将云台imu四元数转成云台imu旋转矩阵 然后再进行一个基变换得到云台到世界的旋转矩阵 但是这里命名感觉有点诡异 我看的有点恍惚 感觉应该把imuabs换成world会好读一点 在看这里的时候我还有一个疑问就是 为什么云台的imu被转换成旋转矩阵之后就可以直接用作基变换 后来问他们才知道他们本质都是描述坐标系b关于坐标系a的变换 
```cpp
 cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```
  我们在拿到云台到世界的旋转矩阵(R_gimbal2world_)之后就可以拿这个矩阵进行运算 最终得到装甲版在世界坐标系下的坐标 然后在standard.cpp文件里面我们就看不到任何关于姿态的值传入了 原因是在类的实例化的时候 将Solver这个类的一个实例化对象solver以引用的方式传入了tracker的构造函数 也就是说此时tracker持有solver的一切数据 

# 补档
例会上听到了四元数插值 回头看了下 算是个有趣的发现 就是
```cpp
  camera.read(img, t);
    q = cboard.imu_at(t - 1ms);
```
这里相机调用成员函数给t打上时间戳(获取相机画面和画面对应的时间) 然后再传给imu 我们看一下imu_at的函数实现
```cpp
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;//如果相机的时间戳大于最后一次获取四元数的时间戳 就把最后一次获取的四元数作为相机时间戳之前的四元数

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }//获取新的四元数 使得相机的时间戳卡在两次获取的四元数时间戳之间

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;
  ```
这里是一系列的条件判断 然后获取时间差 接下来就是计算四元数(插值)
```cpp
auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();//使用球面线性插值计算四元数的插值
```
具体可以参考[插值](https://zhuanlan.zhihu.com/p/538653027)和[四元数](https://blog.csdn.net/qq_40301351/article/details/140046388)一切的一切都是为了让相机的画面刚好对应当时的四元数 不得不说设计的太巧妙了
