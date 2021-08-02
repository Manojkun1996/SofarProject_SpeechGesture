# SOFAR PROJECT
# Commanding MiRo with natural language
This Project consists in developing a simple software architecture to simulate the behaviour of a MIRO robot.
The robot features three possible behaviours, i.e. Normal, Sleep and Play, and responds to user commands.

---

## System Architecture

### Component Diagram

<p align="center"> 
<img src="Components Diagram.png">
</p>

The software architecture is based on **three main components**:

- **Person module**

    This module mimics the behaviour of a person that controls the robot via voice commands and by pointing gestures. 

    It features two pub/sub interfaces to communicate with the finite state machine component: one sends strings containing voice commands on the *"voice_command"* topic and the other one two integers defining a location on the *"pointing_gesture"* topic.

- **Finite state machine**

    This component implements a finite state machine using "Smach".

    The three states, together with the transitions between them, will be further explained in the following paragraph.

- **Robot control service**

    This component implements a server/client pattern and checks the consistency of the requested location with respect to the map's boundaries.
    Each state of the finite state machine calls this service to move the robot to a certain location.

### State Diagram


<img src="State Diagram.png">


This is the state diagram that shows how the finite state machine works:

When the robot is in **Normal** state it simply moves randomly by calling the robot control service to go to a valid location, i.e. inside the map boundaries.
The state transitions to *Sleep* when the sleep counter goes above a defined threshold, which means that the robot has reached a certain number of locations.
The state transitions to *Play* when the robot receives a "play" voice command from the user. 

When the robot is in **Sleep** state, it first reaches the predefined "Home" location, then stays there for some time and finally wakes up, transitioning back to the *Normal* state.

When the robot is in **Play** state, it first reaches the user and then waits for him/her to point a location: when done, it reaches the pointed location and then comes back to the user only to wait for other gestures.
The state automatically transitions back to *Normal* after some time has passed.

### rqt_graph

<img src="<rosgraph.png">


---

## ROS Messages and Parameters

Custom ROS **messages** are:

- **Location.msg**

    ```
    int64 x
    int64 y
    ```

    Defines a location in the 2D environment by using two integers: it's used in the pub/sub interface for the *"pointing_gesture"* topic.

- **VoiceCommand.msg**

    ```
    string command
    ```

    Defines a voice command that the user can send to the robot: it's used in the pub/sub interface for the *"voice_command"* topic.

- **MoveRobot.srv**

    ```
    int64 x
    int64 y
    ---
    bool goalReached
    ```

    Defines the request/response for the robot control server: the two integers define the location the robot has to move to and the bool tells whether the robot was able to reach its goal or not.

Custom ROS **parameters** are:

- `map/xmax` and `map/ymax`

    Define the map's boundaries: the map will be a rectangle with sides defined by these two parameters.

- `person/x` and `person/y`

    Define the position of the person in the 2D plane.

- `home/x` and `home/y`

    Define the position of the key location "Home".

- `time_scale`

    Defines the speed at which the simulation goes: e.g. a value of 2 means 2x speed.

- `robot/state`

    Defines the state which the robot is currently in.

---

## Packages and File List

Going in alphabetical order:

- **diagrams**

    - `Component_Diagram.png`, `State_Diagram.png` and `rosgraph.png`

        The three diagrams shown in this README file.

- **launch**

    - `first_assignment.launch`

        The launchfile used to setup the system, it's called with several arguments which will become ROS parameters.

- **msg**

    - `Location.msg` and `VoiceCommand.msg`

        As mentioned in the *ROS messages and parameters* section, they are custom messages used for the pub/sub interface.

- **scripts**

    - `person.py`, `robot_control_server.py` and `state_machine.py`

        The modules of the architecture, as described in the *System Architecture* section.

- **srv**

    - `MoveRobot.srv`

        As mentioned in the *ROS messages and parameters* section, it's a custom message used for the sever/client interface.

- `CMakeLists.txt` and `package.xml`

    Necessary files to compile and run the system nodes.

---

## Installation and Running Procedure

First of all, clone this repository inside your ROS workspace's *"/src"* folder .

Then, navigate to the *"/scripts"* folder and make the three Python scripts executable with:
```
$ chmod +x person.py
$ chmod +x robot_control_server.py
$ chmod +x state_machine.py
```

Go back to the root folder of your ROS workspace and execute:
```
$ catkin_make
$ catkin_make install
```

In a separate terminal run:
```
$ roscore
```

You're almost ready to go! The last thing left to do is to run the launchfile:
```
$ roslaunch erl_first_assignment first_assignment.launch home_x:=<value> home_y:=<value> map_xmax:=<value> map_ymax:=<value> person_x:=<value> person_y:=<value> time_scale:=<value>
```
Where `<value>` stands for an integer you want the corresponding parameter to be set to (if you want to use default values simply delete one or more arguments).

The system will now setup and run, showing on the terminal the robot behaviour, state transitions, user interaction and movements.

---

## Working Hypothesis and Environment

The **working hypotheses** are: 

- The robot has no modules dedicated to sensing, which is considered implicit, thus the commands sent by the user are understood perfectly.

- The person can, at any time, aknowledge exactly what the robot state is and thus sends commands accordingly; e.g. if the robot is in normal state he/she won't point a location.

- The person only sends "play" voice commands or pointing gestures.

- The robot knows the environment, i.e. the map's boundaries, the key locations position and the person position.

- The robot can't switch states when it's moving, reaching a location is considered an atomic action.

- The robot can go anywhere on the map.

- The robot has infinite battery life, it never has to recharge.

The **enviroment hypotheses** are:

- The map is described by two values, `xmax` and `ymax`: it's assumed to be a rectangle starting from the origin O = (0, 0) and having as sides 2D vectors **OX** and **OY**, with X = (xmax, 0) and Y = (0, ymax).

- The environment is free from obstacles.

- The only key location the robot knows is "Home".

- The person position is constant throughout the entire robot operation.

---

