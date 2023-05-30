# CCOM Autonomous Mission Planner software architecture.

QT5 based using QGraphics for the map display and a Model/View (MV) architecture for listing items in a MV view.

The map view is provided by MapView which implements a QGraphicsView.

## Map class

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

## Map items

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
