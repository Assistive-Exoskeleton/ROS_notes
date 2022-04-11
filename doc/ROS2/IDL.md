# IDL (Interface Definition Language)


[`IDL`](https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html) is the language used by ROS2 to describe interfaces (messages, services, actions). The defined interface in `IDL` is then passed to generators (`rosidl_default_generators` library) in order to obtain language_specific code (`Python`, `C++`, ...) to interact with the interfaces.

> both line (`//`) and block (`/* ... */`) comments are supported by `IDL`.

## Messages
Messages are the basic type of interfaces. The other ROS2 interfaces are built upon messages. Messages are described and defined in `.msg` files in the `msg/` directory of a ROS2 package.

A message definition in `IDL` is written through:
- **Fields**: consists of a type and a name:
  ```
  fieldtype fieldname
  ```
  Field types can be:
  - **built-in** types (`bool`, `char`,...) 
  - names of **message descriptions** defined on their own (`package_name/msg_type`)

  Default values can be set to any field in the message type:
  ```
  fieldtype fieldname defaultvalue
  ```
- **Constants**: Each constant definition is like a field description with a default value, except this value can never be changed programatically:
  ```
  constanttype CONSTANTNAME=constantvalue
  ```
  Constant names have to be UPPERCASE.

## Services
Services are described and defined in `.srv` files inside `srv/` directory of a package. 

A service description file consists of a **request** and a **response** messages, separated by `---`. Any two **messages** concatenated with a `---` are a legal service description:
```C
//REQUEST message
---
//RESPONSE message
```

## Actions
Actions are described and defined in `.action` files inside `action/` directory of a package. 

Similar to services, an action description file consists of a **request**, a **result** and a **feedback** messages, separated by `---`:
```C
//REQUEST message
---
//RESULT message
---
//FEEDBACK message
```