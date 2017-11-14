# frAIburg Dynamic Emergency Break

author: Markus Merklinger

**abstract**:
emergency_filter with dynamic ultrasonic pins, position for a drivable
   area, map collision detection
- if the pin is connected the the emergency function will be enabled
-  two xml configuration files can be set in the filter properties:
   Ultrasonic config: calibration of the ultrasonic sensors
   filter config: map collision box size, ultrasonic thresholds a
       and position limits
- Ultrasonic data is stored in a circular buffer, if a cycle is compleat the,
	mean is used to check if a threshold is reached. Buffer size and max value
	to add to buffer can be set in the filter properties
- the time interval to check for map collision can be set in the filter
   properties, the collision box size ca be set with an xml file
- in the debug modus the map collision detection box in show in green if no
   collision else it will be remove_elements_over_distance
- use the base ultrasonic pin names of the
   addc arduino communication filter pin names to load the correct config.
   in the filter xml (see filter configuration xml)
- if a emergency is detected the speed signal 0 and jury bool false signal
   is transmitted, no true signal is send

**inputs:**
- dynamic ultrasonic pins (name to select the calibration sensor target
 in the Ultrasonic sensor configuration xml)
- current car position to set the a limit for the drivable area

**outputs:**
- speed signal, transmits zero if emergency is triggered
- jury bool value, transmits false if emergency is triggered


**filter configuration xml**:

```xml
<?xml version="1.0" encoding="utf-8"?>
	<configuration datetime="05-Sep-2017" target="car2">
	<DynamicEmergencyBreak description="05-Sep-2017" target="PositionLimits">
		 <!-- emergency position limits in meter-->
		 <value name="threshold_meter_x_min" value="0"/>
		 <value name="threshold_meter_x_max" value="6"/>
		 <value name="threshold_meter_y_min" value="0"/>
		 <value name="threshold_meter_y_max" value="5"/>
	</DynamicEmergencyBreak>

  <DynamicEmergencyBreak description="16-Sep-2017" target="MapCollisionCheckBox">
     <!-- box the check with collision in the local car frame-->
     <value name="center_local_meter_x" value="0.1"/>
     <value name="center_local_meter_y" value="0"/>
     <value name="size_half_meter_x" value="0.1"/>
     <value name="size_half_meter_y" value="0.15"/>
     <value name="box_orientation_clockwise_degree" value="0"/>
  </DynamicEmergencyBreak>

	<!-- emergency threshold in meter for 8 us sensors-->
	<DynamicEmergencyBreak description="05-Sep-2017" target="UltrasonicFrontLeft">
		 <value name="threshold_meter" value="0.3"/>
	</DynamicEmergencyBreak>
</configuration>
```

**Ultrasonic sensor configuration xml**:
 calibration for the ultrasonic input pin data.

```xml
<?xml version="1.0" encoding="utf-8"?>
<configuration datetime="05-Sep-2017" target="car2">
	<sensor description="05-Sep-2017" target="UltrasonicFrontLeft">
			<!-- lin. calibration-->
			<value name="calibration_factor" value="0.01"/>
      <value name="calibration_bias" value="0"/>
	 </sensor>
</configuration>
```
