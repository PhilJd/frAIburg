# frAIburg Ultrasonic To Map Filter

author: Markus Merklinger

**abstract**:
frAIburg_UltrasonicToMapFilter with dynamic ultrasonic pins
- if the pin is connected the the emergency function will be enabled
-  xml configuration files can be set in the filter properties:
   Ultrasonic config: calibration of the ultrasonic sensors and transformation
	 to the local car frame
- Ultrasonic data is stored in a circular buffer, if a cycle is compleat the,
	mean is used a map box element to the map. The size is set by the view angle
	and distance to the car
- Buffer size and max value to add to buffer can be set in the filter properties

**inputs:**
- dynamic ultrasonic pins (name to select the calibration sensor target
 in the Ultrasonic sensor configuration xml)

**Ultrasonic sensor configuration**
```xml
<?xml version="1.0" encoding="utf-8"?>
<configuration datetime="03-Sep-2017" target="car2">

	<sensor description="05-Sep-2017" target="UltrasonicFrontLeft">
			<!-- Transformation that maps to the car coordinate system-->
			<value name="offset_meter_x" value="-0.02"/>
			<value name="offset_meter_y" value="0.12"/>
			<value name="rotation_degree" value="45"/>
			<!-- calibration for the raw us sensor value-->
			<value name="calibration_factor" value="0.01"/>
			<value name="calibration_bias" value="0"/>
       <value name="field_of_view_angle_degree" value="30"/>
	 </sensor>
</configuration>
```
