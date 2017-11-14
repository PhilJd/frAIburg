#ifndef event_struct_h
#define event_struct_h


struct event{
    int event_priority;
    float event_distance;
    int event_id;
    int event_type;
    float angle;
    /*bool operator<(const event& rhs) const
    {
        return event_distance > rhs.event_distance;
    }
    */
      bool operator<(const event& rhs) const
    {
        if (event_priority > rhs.event_priority) {return true;}
        else if (event_priority == rhs.event_priority)
        {
            if(event_distance > rhs.event_distance) {return true;}
            else {return false;}
        }
        else{return false;}
    }
};

enum event_type{
    UNKNOWN_EVENT=0,
    PEDESTRIAN_ON_ZEBRA_CROSSING,
    PEDESTRAIN_WAITS_AT_ZEBRA_CROSSING,
    EMPTY_ZEBRA_CROSSING,
    X_CROSSING_RIGHT_OF_WAY,
    X_CROSSING_GIVE_WAY,
    X_CROSSING_STOP,
    X_CROSSING_RIGHT_OF_WAY_RIGHT,
    T_CROSSING_RIGHT_OF_WAY,
    T_CROSSING_GIVE_WAY,
    T_CROSSING_STOP,
    T_CROSSING_RIGHT_OF_WAY_RIGHT,
    OBSTACLE,
    PARKING_SPOT,
    FREE_PARKING_SPOT,
    OCCUPIED_PARKING_SPOT,
    PARKING_SIGN,
    PARKING_STOP,
    ROAD_WORKS_SIGN,
    YOUNG_PEDESTRIAN_NEAR_ROAD,
    STOP_SIGN,
    STOP_LINE,
    RIGHT_OF_WAY,
    RIGHT_OF_WAY_LINE,
    SLOW_DOWN,
    SPEED_UP,
      
    
};

#endif 
