# interop_pkg
## Introduction
This repo is a simple ROS node that does the following:
* Logs in to judge's server (as specified in environment variables, set by `params.sh`).
* Subscribes to the `gps_state` and `plans` topics. These provide telemetry data and submitted objects, respectively. When it gets a message here, it POSTS it to judges server.
* Fetches obstacles and missions from judges and publishes on the `obstacles`, `obstacles/stationary`, `obstacles/moving` and `missions` topics.

Former team members tell us that with the amount of HTTP requests this node is making, it is best to run it on a beefy dedicated machine during competition.
