
# 1 内存分区模型

C++程序在执行时，将内存大方向划分为**4个区域**

- 代码区：存放函数体的二进制代码，由操作系统进行管理的
- 全局区：存放全局变量和静态变量以及常量
- 栈区：由编译器自动分配释放, 存放函数的参数值,局部变量等
- 堆区：由程序员分配和释放,若程序员不释放,程序结束时由操作系统回收

在程序编译后，生成了exe可执行程序，**未执行该程序前**分为代码区和全局区

## 代码区
就是存放那个二进制代码的

## 全局区
如图所示
![alt text](image.png)
存在全局区的的这些变量的生命周期一直到程序结束后自动释放

## 栈区
由编译器自动分配释放 存放函数的参数值 局部变量等  
生命周期一般在函数运行结束后自动释放

## 堆区
由程序员手动分配和释放的 如果不主动释放 在程序结束时也会自动释放
用```new```来开辟堆区内存 返回的是指针地址 在需要释放内存时使用```delete```删除接受的指针
```cpp
    int *p=new int;
    delete p;
```
# 2 浅拷贝与深拷贝
## 浅拷贝
浅拷贝指的是只复制一个指向某个数据的指针 但是那个数据却不会被拷贝 两个指针指向同一个地址 当其中一个指针将那个数据释放时 另一个指针再释放一次则会导致二次释放 程序就会崩溃
```cpp
#include<iostream>

class shallow{
	public:
		shallow(int a_){
			p=new int(a_);
			
		}
		shallow(const shallow &x){
			p=x.p;
		}
		~shallow(){
			delete p;
		} 
	
	private:
	
		int *p;
	
	
};



int main(){
	
	shallow x(10);
	shallow y(x);
	
	return 0;
} 
```
这里当x被创建时 会有一个指向堆区的指针 y采用拷贝构造拷贝的x 此时y的p指针也指向x中的那个10 在程序结束时y先析构释放指针 此时x再析构造成二次释放 程序崩溃

## 深拷贝
深拷贝则是再申请一个堆内存 存放一样的数据 这样一个指针指向一个数据 释放的时候就不会二次释放了
```cpp
#include<iostream>

class shallow{
	public:
		shallow(int a_){
			p=new int(a_);
			
		}
		shallow(const shallow &x){
			p=new int(x.p);
		}
		~shallow(){
			delete p;
		} 
	
	private:
	
		int *p;
	
	
};



int main(){
	
	shallow x(10);
	shallow y(x);
	
	return 0;
} ```
这就完成一次深拷贝了



```cpp
class test{

	int a;


}


test *a;

int main(){

	a=new test;
	return 0;


}