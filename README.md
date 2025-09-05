# CCOM Autonomous Mission Planner

Planning software for autonomous surface and underwater vehicles.

This was initially a stand-alone offline application for planning missions. A ROS based monitoring component got added and this is now a ROS package part of Project11.

## Building documentation

In the project root directory, run:

    ./build_docs.bash

Sphinx documentation will be at: [./docs_build/output/camp/index.html](./docs_build/output/camp/index.html)

Doxygen docs will be at: [./docs_build/output/camp/generated/doxygen/html/index.html](./docs_build/output/camp/generated/doxygen/html/index.html)

## Software architecture

QT5 based using QGraphics for the map display and a Model/View (MV) architecture for listing items in a MV view.

The map view is provided by MapView which implements a QGraphicsView.

### Map class

QAbstractItemModel is the base for the map::Map class that implements the model as well as contains the QGraphicsScene.

Map
  settings (SettingManager)
  tools (ToolsManager)
    ROS layers (RosManager)
    background layers (BackgroundManager)
    measuring tool (MeasuringTool)
  platforms (PlatformManager)
    robot1
    mothership1
  tasks (TaskManager)
    survey_area_1
  layers (LayerList)
    layer1
    layer2
 ...

### Map items

The base class for items to be indexed in the model is map::MapItem. It may also be represented on the map.

MapTool: Shows up in "tools" in tree view.

LayerManager: MapTool that create layers and interact with them.

  BackgroundManager:
    BackgroundLayer: Layer to provide context. Source may be local drive or network.

  RosManager:
    RosLayer: Layer to display ROS data.

TaskManager:
  TaskItem: Contains mission tasks.

PlatformManager: Contains tracked plarforms.
  Platform: Represents a Projec11 platform

LayerList: Allows layers to be reordered.

## JSON base format for sending missions to a robot and saving a project to file

This is the format used in September 2023 to send and save missions before the planned switch to a format based on project11_nav_msgs/TaskInformation messages for sending missions to a robot.
To send a mission to a robot, a JSON array containing an object for each CAMP MissionItem is constructed.
To save a project to a file, a top level Group item as described below is constructed containing all the mission items.

Members for all items include:

- label
- speed (if not 0)
- type
- children: Array of behaviors (optional, only for items that can be sent to the robot)

### Current object types

Current types defined in the CAMP source code:

- BackgroundRaster (can't be sent to the robot)
- Behavior
- Group
- Orbit
- SearchPattern
- SurveyArea
- SurveyPattern
- TrackLine
- Waypoint
- VectorDataset (can't be sent to the robot)

Following are the object specific members of the above object types.

#### BackgroundRaster

- filename

#### Behavior

- behaviorType
- active

#### Group

- children: Array of mission item objects

#### Orbit

- radius
- safetyDistance
- targetFrame

#### SearchPattern

- startLocation: Waypoint object
- endLocation: Waypoint object
- first_leg_heading
- first_leg_distance
- direction: clockwise or counterclockwise
- children: Array of TrackLine objects

#### SurveyArea

- children: Array of mission item objects

#### SurveyPattern

- startLocation: Waypoint object
- endLocation: Waypoint object
- spacing
- direction
- alignment: start, center, or finish
- children: Array of TrackLine objects

#### TrackLine

- waypoints: Array of Waypoint objects

#### Waypoint

- latitude
- longitude

#### VectorDataset

- filename
