# ROS 2 Nodes, Topics, and Services

## Introduction to ROS 2

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behaviors.

## Core Concepts

### Nodes

A **node** is an executable process that performs computation. ROS 2 is designed for modularity, where each node is responsible for a single, specific task (e.g., one node for controlling a motor, another for reading sensor data, another for path planning).

### Topics

**Topics** are a named bus over which nodes exchange messages. A node can publish messages to a topic or subscribe to messages from a topic. This is a many-to-many, anonymous, publish/subscribe communication model.

*   **Publishers:** Nodes that send messages to a topic.
*   **Subscribers:** Nodes that receive messages from a topic.

### Services

**Services** are a request/reply communication model. Unlike topics, services are designed for scenarios where a node needs to send a request to another node and receive a response. This is often used for operations that are more "call-and-response" in nature, such as triggering an action or querying a specific piece of information.

*   **Service Servers:** Nodes that offer a service and respond to requests.
*   **Service Clients:** Nodes that send requests to a service server and wait for a response.

## Key Differences from ROS 1

ROS 2 was re-architected to address limitations of ROS 1, including:
*   **Real-time control:** Improved support for real-time applications.
*   **Multi-robot systems:** Better handling of multiple robots and distributed systems.
*   **Security:** Enhanced security features.
*   **Quality of Service (QoS):** More control over communication reliability, latency, and durability.

## Further Reading

*   ROS 2 Documentation: [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
*   ROS 2 Tutorials: [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
