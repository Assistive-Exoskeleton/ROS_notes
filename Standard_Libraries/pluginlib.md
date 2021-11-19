# Plugin library
`pluginlib` is a C++ library for loading and unloading plugins within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library.

With pluginlib, one does not have to explicitly link their application against the library containing the classes – instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

With `pluginlib`, a constructor without parameters is required. Therefore, if any parameter is required, we can use the `initialize` method:
```cpp
namespace myPkg
{
    class myClass
    {
        public:
            virtual void initialize(/*parameters*/) = 0;
            virtual ~myClass(){}
        protected:
            myClass(){}
    }
}
```