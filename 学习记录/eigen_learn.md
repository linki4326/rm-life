# Eigen::Vector3d
Eigen::Vector3d 是一个表示三维向量的 类 ，底层是一个 3x1 的矩阵（列向量）
可进行下标访问
```cpp
double x = vec[0];  // x = 1.0
double y = vec[1];  // y = 2.0
double z = vec[2];  // z = 3.0
```
or
```cpp
    x = vec(0);  // x = 1.0
    y = vec(1);  // y = 2.0
    z = vec(2);  // z = 3.0
```
也可通过指针或成员函数访问
```cpp
Eigen::Vector3d vec(1.0, 2.0, 3.0);
 
double x = vec.x();  // x = 1.0
double y = vec.y();  // y = 2.0
double z = vec.z();  // z = 3.0
 
// 也可用于修改值
vec.x() = 10.0;  // vec 变为 (10.0, 2.0, 3.0)
```cpp
Eigen::Vector3d vec(1.0, 2.0, 3.0);
double* ptr = vec.data();
 
// 通过指针访问
double x = ptr[0];  // x = 1.0
double y = ptr[1];  // y = 2.0
double z = ptr[2];  // z = 3.0
 
// 批量操作（如与 C 库交互）
for (int i = 0; i < 3; ++i) {
    ptr[i] *= 2.0;  // vec 变为 (2.0, 4.0, 6.0)
}```