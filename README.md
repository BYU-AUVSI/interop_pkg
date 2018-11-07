# interop_pkg
ROS wrappers for interacting with the AUVSI interop server using a ROS network.

## Installation and Setup


## Introduction
This repo is a simple ROS node that does the following:
* Logs in to judge's server (as specified in environment variables, set by `params.sh`).
* Subscribes to the `gps_state` and `plans` topics. These provide telemetry data and submitted objects, respectively. When it gets a message here, it POSTS it to judges server.
* Fetches obstacles and missions from judges and publishes on the `moving_obstacles`topic. Makes available a service to get a mission by id (`get_mission_with_id`) with an int parameter representing the mission id to fetch.

Former team members tell us that with the amount of HTTP requests this node is making, it is best to run it on a beefy dedicated machine during competition.

## Running it
Do the following:
1. Make sure a RosMaster is running. You can either run `roscore` or run a launchfile that starts this up automatically.
2. Run `rosrun interop_pkg client.py`.
3. Run `rostopic echo /moving_obstacles` to see if missions are coming through. To see anything, the judge's server must be running on the same machine. Instructions to do this can be found [here](http://auvsi-suas-competition-interoperability-system.readthedocs.io/en/latest/).
