# robotnik_interfaces

A set of packages which contain common interface files from the Robotnik stack (.msg, .srv and .action).


## Purpose

Isolating the messages to communicate between stacks in a shared dependency allows nodes in dependent stacks to communicate without requiring dependencies upon each other.

This repository has been designed to contain the most common messages used between multiple packages to provide a shared dependency which will eliminate a problematic circular dependency.

## Interface packages

### robotnik_battery_msgs

Interfaces used by the power systems of the robots.

### robotnik_common_msgs

Message definitions for common stuff in the Robotnik robots.

### robotnik_controllers_msgs

Interfaces used by the Robotnik controllers:
- https://github.com/RobotnikAutomation/robotnik_controllers

### robotnik_interfaces

Metapackage that includes all Robontik interfaces.

### robotnik_io_msgs

Interfaces to read and write analog and digital inputs and outputs.

### robotnik_navigation_msgs

Interfaces for the different navigation actions of the robot (Dock, Move, etc)

### robotnik_perception_msgs

Message definitions for the Robotnik Perception Stack.

### robotnik_safety_msgs

Interfaces for the Robontiks Safety system of:
- https://github.com/RobotnikAutomation/safety_module

### robotnik_supervisor_msgs

Interfaces to manage Robotnik stacks and modules. Used by
- https://github.com/RobotnikAutomation/robotnik_docker_supervisor


## How to add a new interface

Each package contains these folders to organize the definitions:

- *package*/msg: message defitinion
- *package*/srv: service definition
- *package*/action: action definition

This repository follows the ROS convention for interface packages:
- all packages share the same version,
- if one package is updated, the version of all packages in this repository is also updated.
- versioning uses Semantic Versioning schema [https://semver.org/]. As a hint:
  - MAJOR: changing or removing an existing field, big adding of new interfaces (i.e. API break, requires modify existing software).
  - MINOR: adding a new field, adding comments to message definitions (i.e. no API break, just requires rebuilding software).
  - PATCH: changes in changelog, licenses, etc.
- CMakeLists.txt, package.xml must follow `robotnik_common_msgs` structure.

## How to add new messages

All message definitions must follow the ROS message definition format.

## How to add new services

All service responses must return information about the result of the service call, by including the fields:
- robotnik_common_msgs/Response response
which contains:
  - bool success. Required, indicates if the service call was successful.
  - string message. Optional if success is true, but required if success is false. It should contain a human-readable message with the reason of the failure.

## How to add new actions

All action results must return information about the result of the service call, by including the fields:
- robotnik_common_msgs/Response response
which contains:
  - bool success. Required, indicates if the action call was successful.
  - string message. Optional if success is true, but required if success is false. It should contain a human-readable message with the reason of the failure.
