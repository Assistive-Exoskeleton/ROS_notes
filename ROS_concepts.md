# 1. ROS concepts
ROS concepts that are similar between ROS and ROS2.
## 1.1. Table of contents
- [1. ROS concepts](#1-ros-concepts)
  - [1.1. Table of contents](#11-table-of-contents)
- [2. ROS graph concepts](#2-ros-graph-concepts)
  - [2.1. Nodes](#21-nodes)
  - [2.2. Messages](#22-messages)
  - [2.3. Topics](#23-topics)
  - [2.4. Services](#24-services)
  - [2.5. Parameters](#25-parameters)
  - [2.6. Actions (ROS2)](#26-actions-ros2)
- [3. Names](#3-names)
  - [3.1. Graph resource name](#31-graph-resource-name)
  - [3.2. Package resource name](#32-package-resource-name)
# 2. ROS graph concepts
The ROS **graph** or **computation graph** is the peer-to-peer network of ROS elements processing data together at one time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

## 2.1. Nodes
Nodes in ROS are responsible for each single module purpose. Nodes communicate data with one another through [topics](#topics), [services](#services), [actions](#actions-ros2) or [parameters](#parameters).

A single executable (C++, Python...) can contain one or more nodes. 

![Nodes](./images/nodes.gif)

All nodes have a [name](#graph-resource-name) that uniquely identifies them to the rest of the system. Nodes also have a [type](#package-resource-name).

## 2.2. Messages
Communication between nodes happens by exchanging ROS **messages**. A message is a simple data structure.

For example:
- Nodes communicate with each other by publishing messages to [topics](#topics).
- Nodes can also exchange a request and response message as part of a ROS [service](#services) call.

ROS messages use **YAML** syntax (dictionary/json - like. Note that spaces are important)

## 2.3. Topics
Topics act as a bus for nodes to exchange messages. Communication via topics is N-to-N through a **Publisher-Subscribe model**:
- Nodes sending messages are **Publishers** for that topic 
- Nodes receiving messages are **Subscribers** to that topic

![topics](./images/topics.gif)

Nodes need to know the **type** of the message to be able to understand the information. A topic type is defined by the message type published on it.

## 2.4. Services
Services are another method of communication for nodes. Services are based on a **Request-Response model**. Services only provide data when they are specifically called by a client.

![services](./images/services.gif)

Like topics, service have a **service type** which is the **package resource name**

## 2.5. Parameters
A parameter is a configuration state of a node/system. Parameters are dynamically reconfigurable and built off of ROS services: a shared **parameter server** is used, accessible via network APIs, such that tools can easily inspect and modify parameters.

## 2.6. Actions (ROS2)
Another communication type intended for long-running tasks. They consist of:
- **goal**
- **feedback**
- **result**

Actions are built on **Topics** and **Services**. They use a **client-server** model: an **action client** sends a goal to an **action server** that acknowledges the goal and returns a stream of feedback and a result.

>Note: both the **action server** and the **action client** can stop a goal to be processed. When the server stops a goal it's said to *abort* the goal. 

![actions](./images/actions.gif)


# 3. Names
## 3.1. Graph resource name
Graph Resource Names provide a hierarchical naming structure that is used for all resources in a **ROS graph**.

Each resource is defined within a **namespace**, which it may share with other resources. In general, resources can:
- **create** resources within their namespace 
- **access** resources within or above their own namespace.

Names are resolved relatively, so resources do not need to be aware of which namespace they are in.

There are 4 types of Graph Resource Names:
- **base** (`base`)
- **relative** (`relative/name`): default resolution
- **global** (`/global/name`)
- **private** (`~private/name`)

Example:
|Node|Relative `foo`|Global `/foo`|Private `~foo`|
|----------|---------|------|--------------|
|`/wg/node`|`/wg/foo`|`/foo`|`/wg/node/foo`|

>Note: any name within a ROS Node can be remapped when the Node is launched at the command-line.

## 3.2. Package resource name
Package Resource Names are used in ROS with **Filesystem-Level** concepts to simplify the process of referring to files and data types on disk. They are just the name of the package that the resource is in plus the name of the resource

> Because of this, be careful and not produce different executables with the same name in the same package

Some of the ROS-related files that may be referred to using Package Resource Names include:
- **Message** (msg) types
- **Service** (srv) types
- **Node** types
