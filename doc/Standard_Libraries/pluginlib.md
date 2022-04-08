# Plugin library
**Contents**:
- [Plugin library](#plugin-library)
- [Plugin implementation](#plugin-implementation)
  - [Base class](#base-class)
  - [Plugin class](#plugin-class)
  - [XML plugin declaration](#xml-plugin-declaration)
  - [CMake plugin declaration](#cmake-plugin-declaration)
- [Using the plugins](#using-the-plugins)

`pluginlib` is a C++ library for loading and unloading plugins within a ROS package. Plugins are dynamically loadable classes that are loaded from a runtime library.

With pluginlib, one does not have to explicitly link their application against the library containing the classes – instead pluginlib can open a library containing exported classes at any point without the application having any prior awareness of the library or the header file containing the class definition. Plugins are useful for extending/modifying application behavior without needing the application source code.

# Plugin implementation
The implementation of a plugin follows some steps:
1. [Base class](#base-class)<!---->
2. [Plugin class](#plugin-class)
3. [XML plugin declaration](#xml-plugin-declaration)
4. [CMake plugin declaration](#cmake-plugin-declaration)

## Base class
We start declaring a base (abstract) class. With `pluginlib`, a constructor without parameters is required. Therefore, if any parameter is required, we can use the `initialize` method:
```cpp
#ifndef MY_PKG_BASE_MY_CLASS_BASE_HPP
#define MY_PKG_BASE_MY_CLASS_BASE_HPP
namespace my_pkg_base
{
    class MyClassBase
    {
        public:
            virtual void initialize(/*parameters*/) = 0;
            virtual ~MyClassBase(){}
        protected:
            MyClassBase(){}
    };
}
#endif //   MY_PKG_BASE_MY_CLASS_BASE_HPP
```

## Plugin class
Now we implement the class for the plugins (inside `my_plugins` package for example)
```cpp
#include <my_pkg_base/my_class_base.hpp>

namespace my_plugins
{
    class FirstPlugin : public my_pkg_base::MyClassBase
    {
        public:
            void initialize (/*parameters*/) override
            {

            }
        //other methods and attributes
    };

    //other plugins
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(my_plugins::FirstPlugin, my_pkg_base::MyClassBase)
//other plugins' macros
...
```

Plugins are exported using `PLUGINLIB_EXPORT_CLASS` macro, which registers the classes as actual plugins.
- The first argument is the fully-qualified type of the **plugin class**.
- The second argument is the fully-qualified type of the **base class**.

## XML plugin declaration
The **plugin loader** still needs a way to find the library in which plugins exist, in order to create plugin instances. To do so, we create a `plugins.xml` file (inside `my_plugins` pkg) with the necessary information:
```XML
<library path="my_plugins">

    <class type="my_plugins::FirstPlugin" base_class_type="my_pkg_base::MyClassBase">
        <description>FirstPlugin description</description>
    </class>

  <!-- other plugins -->
</library>
```
- The `<library>` tag gives the relative `path` (ROS2 simply uses the name) to a library that contains the plugins that we want to export.
- The `<class>` tag declares a plugin that we want to export from our library, with attributes:

    attribute|description
    -|-
    `type`|fully qualified type of the plugin
    `base_class_type`|fully qualified base class type for the plugin.
    `description`|description of the plugin

## CMake plugin declaration
The last step is to export the plugins via `CMakeLists.txt`. After `find_package(pluginlib REQUIRED)` add:
```cmake
add_library(my_plugins src/my_plugins.cpp)

target_include_directories(my_plugins PUBLIC
<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  my_plugins
  my_pkg_base
  pluginlib
)

pluginlib_export_plugin_description_file(my_pkg_base plugins.xml)

install(
  TARGETS my_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```

And before `ament_package` add:
```cmake
ament_export_libraries(
  my_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)
```

# Using the plugins
Plugins can now be used in any package. For example we can create a node like:
```cpp
#include <pluginlib/class_loader.hpp>
#include <my_pkg_base/my_class_base.hpp>

int main(int argc, char** argv)
{
    //...
    pluginlib::ClassLoader<my_pkg_base::MyClassBase> poly_loader("my_pkg_base", "my_pkg_base::MyClassBase");

    try
    {
        std::shared_ptr<my_pkg_base::MyClassBase> myObject = poly_loader.createSharedInstance("my_plugins::FirstPlugin");
        myObject->initialize(/*parameters*/);

        //other code
    }
    catch(pluginlib::PluginlibException& ex)
    {
        printf("Plugin failed to load. Error: %s\n", ex.what());
    }

    return 0;
}
```

- `ClassLoader` is the key class:
  - It's **templated** with the **base class** (e.g. `my_pkg_base::MyClassBase`)
  - The first argument is a string for the **base class package name** (e.g. `"my_pkg_base"`)
  - The second argument is a string with the fully qualified **base class type** for the plugin (e.g. `my_pkg_base::MyClassBase`)
- There are a number of ways to instantiate an instance of the class. For example we use **shared pointers** (`std::shared_ptr`), calling `createSharedInstance` with the fully-qualified type of the plugin (e.g. `my_plugins::FirstPlugin`)

> Importantly, the package in which this node is defined does NOT depend on the `my_plugins` class. The plugins will be loaded dynamically without any dependency needing to be declared.