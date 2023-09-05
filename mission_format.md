# JSON base format for sending missions to a robot and saving a project to file

This is the format used in September 2023 before the planned switch to a format based on project11_nav_msgs/TaskInformation messages.

To send a mission to a robot, a JSON array containing an object for each CAMP MissionItem is constructed.
To save a project to a file, a top level Group item as described below is constructed containing all the mission items.

Members for all items include:

- label
- speed (if not 0)
- type

## Current object types

Current types defined in the CAMP source code:

- BackgroundRaster
- Behavior
- Group
- Orbit
- SearchPattern
- SurveyArea
- SurveyPattern
- TrackLine
- Waypoint
- VectorDataset

Following are the object specific members of the above object types.

### BackgroundRaster

- filename

### Behavior

- behaviorType
- active

### Group

- children: Array of mission item objects

### Orbit

- radius
- safetyDistance
- targetFrame
- children: Array of mission item objects

### SearchPattern

- startLocation: Waypoint object
- endLocation: Waypoint object
- first_leg_heading
- first_leg_distance
- direction: clockwise or counterclockwise
- children: Array of TrackLine objects

### SurveyArea

- children: Array of mission item objects

### SurveyPattern

- startLocation: Waypoint object
- endLocation: Waypoint object
- spacing
- direction
- alignment: start, center, or finish
- children: Array of TrackLine objects

### TrackLine

- waypoints: Array of Waypoint objects

### Waypoint

- latitude
- longitude

### VectorDataset

- filename
