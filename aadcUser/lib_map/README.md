frAIburg MAP
======================
**version: 0.1 **
**Author: Markus Merklinger**

overview:
- mapping with known poses
- Map elements are defined by type, orientation and a polygon
- simplifies working with the local car and global frame
- fuse map elements with different option: replace, mean and append
- handle reposition jumps of the kalman filter used for positioning
- collision detection
- event listener to get updates for the map content
- multithread support for updating the map content
- print, save to .txt and .svg for debugging
- warning and error messages errors are detected
- map live display

Components
======================

**ADTF Map Filter**: the map is available as a shared library

filter that integrate the map in the adtf project:
- map filter: update the position of the car in the map
- map Visualization filter: live display of the global map in ADTF,
  loading of a test maps for reference
- map library: singleton map with event listener for a efficient
  and simple use with ADTF.

filter that add sensor data to the map:
- road sign filter: adds detected sign and known landmark to the map
- ultrasonic to map filter: adds ultrasonic data  
- Bounding box perception filter: adding pedestrian and car detection

filter that are using the map for development:
- planner filter debug mode: a path polygon and waypoints to the map
- laneDetection filter debug mode: shows detected lanes  
- keyboard remote filter: printing the map and saving a .txt an .svg file

filter using content of the map:
- StateMachine: sensing and sate update  

**Map types**: defines fuse, element, data, points, car position
               and polygon types

**Map helper**: polygon transformation rotation and box map element

**Map**: map which holds map elements, calls event listener, fuses elements

**Map elements**: map elements with angle type polygon

**Display widget**: qt widget which can be used with ADTF or as a QApplication


Coordinate system
======================
the local frame of the car is defined at the front center of the car.

```
 y_global[m]
 ^
 |               ^ y_local[m]
 |               |
 |               |
 |           (car)>–––>x_local[m]
 |
 +–––––––––––>x_global[m]
```

Minimal example
======================
updating the car position in the map and addd an Debug point to the map
```c++
#include <iostream>
#include <vector>

#include "global_map.hpp"
#include "map_element.hpp"
#include "map_event_listener.hpp"

int main(int argc, char* argv[]) {
  //get the singelton map
  GlobalMap* map=frAIburg::map::getInstance();
  //set car position in map
  tMapCarPosition car_pos_zero = {.x = 1, .y = 0, .heading = 0.0};
  map->update_car_pos(car_pos_zero);
  //sensor points in the locale frame of the car
  std::vector<tMapPoint> points={tMapPoint(1/*x*/,1/*y*/)};
  //use boost::assign::list_of in  for adtf proj. (not c++11)

  //create a map element and add it to the map
  tSptrMapElement el(new MapElement(DEBUG_POINT, points));
  map->add_element(el);
  //print out information about the element
  LOG_INFO_PRINTF("road sign map element: %s",
                    el->to_string().c_str());
  //print out all DEBUG_POINT types in the map
  std::vector<MapElementType> save_exclude_types={DEBUG_POINT};
  map->print("all DEBUG_POINTS");
  //save the global map to an svg
  map->save_global_poly_svg("angle_test_global_poly_map_end.svg");
}
```
Build
======================

the map as a stand alone version
as a qt project .pro
cmake project configured for atom text editor atom-build package: .atom-build.yml

requirements:  
				- boost >=v.1.58.0: filesystem, thread, geometry
				- qt 4 or 5 for the map ui (displaywidget)

Build with cmake:
```
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
```
Run:
```
    $ build/bin/./map
```
