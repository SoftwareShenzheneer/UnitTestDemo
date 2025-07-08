# Unit Test Demo
Project repository for the CozyCups internal project.
___

## Table of contents
- [Project description](#project-description)
- [Install](#install)
- [Prerequisites and Use](#prerequisites-and-use)
___

## Project Description
The goal of this project is to provide a template to demonstrate a simple unit testing environment. It exists of two parts:
1. esp32 project
2. ceedling project
3. report generation script

The esp32 project implements a PID controller which has a single dependency on a submodule in order to illustrate mocking dependencies. a fake for ```esp_log.h``` has been introduced as an example but is not required to run this project.
The PID controller is an actual implementation and can be implemented into an arbitrary esp project as-is. This has practiccally been copied from psm67's github page: ```https://github.com/pms67/PID```.
The main implements a mass-damper system to simulate an input with the ability to create overshoot in order to properly test the controller.

The ceedling project implements a few unit tests for the pid controller to illustrate how to use ceedling to manage unit tests. Bear in mind that an absolute minimum of unit tests have been implemented since it's sole purpose is explain the core concepts. Additionally, by running the script in the ceedling directory, code coverage reports will be generated.

Execute the script to generate the relevant reports. Currently all options have been enabled.
___

## Install
###### Git clone this repository
```
git clone https://github.com/SoftwareShenzheneer/UnitTestDemo.git
```
___
###### Ensure Ceedling is installed, Version tested is: Ceedling v1.0.1-fb1ce6c
```
ceedling version
```
___
###### Ensure gcov and gcovr are installed
In order to generate reports, gcov and gcovr are required. Gcov and gcovr version 5.0 have been used in this demo.
```
gcov --version
gcovr --version
```
___
###### ESP-IDF
In order to actually use the esp32 project, ensure esp-idf is installed. Feel free to visit their documentation to get access to the details.
___

## Prerequisites and use
1. Install Ceedling, gcov and gcover
2. Verify installed versions
3. Enter the ceedling directory and execute the script: generate_testreport.sh

