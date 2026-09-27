# std::shared_ptr<TYPE> NAME
cpp11的新特性 
智能指针
创建一个TYPE类型的指针(name)
# std::make_shared<TYPE>(OBJCET)
cpp11新特性
智能函数 返回一个std::shared_ptr<T>
TYPE为传入对象的类型 OBJCET为用于TYPE构造函数参数
约等于std::shared_ptr<TYPE>(new T(...))
但这个函数只用进行一次分配内存
