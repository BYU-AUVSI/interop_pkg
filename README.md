# interop_pkg
ROS wrappers for interacting with the AUVSI interop server using a ROS network.

## Installation and Setup

Install docker on your system:

```
sudo apt-get install docker.io
```

Configure docker to be run without sudo:

```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Log out, then back into your machine to implement the changes. Test with the following command:

```
docker run hello-world
```

In order for the server launch file to work on your machine, you need to initialize a docker "container" with the most recent version of the interop server (replace the [...] with the port number):

```
docker run -d --restart=unless-stopped --interactive --tty \
    --publish [PORT NUMBER (i.e. 8000)]:80 \
    --name interop-server \
    auvsisuas/interop-server
docker stop interop-server
```
If you want to change the port number of your server, you will need to destroy the server you've already created and create a new one:
```
docker rm interop-server
docker run -d --restart=unless-stopped --interactive --tty \
    --publish [NEW PORT NUMBER]:80 \
    --name interop-server \
    auvsisuas/interop-server
docker stop interop-server
```
You may now use the launch files to run the server, client, or both on your machine, as described below.

## Usage
This repo is a simple ROS node that does the following:
* Logs in to judge's server (set relevant parameters in `param/client_params.yaml`).
* Subscribes to the `gps_state` and `plans` topics. These provide telemetry data and submitted objects, respectively. When it gets a message here, it POSTS it to judges server.
* Fetches obstacles and missions from judges and publishes on the `moving_obstacles`topic. Makes available a service to get a mission by id (`get_mission_with_id`) with an int parameter representing the mission id to fetch.

Former team members tell us that with the amount of HTTP requests this node is making, it is best to run it on a beefy dedicated machine during competition.

See the launch files for package usage:
```
server.launch # launch the interop server on your machine
client.launch # launch the interop client on your machine
full.launch # launch both the server and client on your machine
```
