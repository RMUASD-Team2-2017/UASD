# UASD Project Source Repository

## How to Build the ROS Packages

###### Bjarki Sigurðsson 13.09.2017 (Updated 18.09.2017)

- Clone this repository into a local directory of your choice:

```
$ cd <repo-path>/ && git clone https://github.com/RMUASD-Team2-2017/UASD.git
```

- To build the GCS ROS package(named gcs), navigate to a catkin workspace and create a symbolic link before running catkin_make:

```
$ cd <path-to>/catkin_ws/src && ln -s <repo-path>/UASD/GCS <linkname>
$ cd .. && catkin_make
```

- You may need to run the catkin-generated setup script before using the package:

```
$ source devel/setup.bash
```

- To verify that everything builds and the nodes run smoothly, run the gcs launch script:

```
$ roslaunch gcs ground_control_station.launch
```

