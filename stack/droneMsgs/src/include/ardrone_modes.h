#ifndef ARDRONE_MODES_H
#define ARDRONE_MODES_H

namespace ARDroneModes {
    // ARDrone navdata.state enum type
    enum ModeType {
        // See comment on ardrone_autonomy/msg/Navdata.msg
        // # 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
        // # 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
        // # Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
        UNKNOWN         = 0,    // Emergency mode
        INIT            = 1,
        LANDED          = 2,    //
        FLYING          = 3,    //
        HOVERING        = 4,    //
        TEST            = 5,
        TAKING_OFF      = 6,    //
        GOTO_FIX_POINT  = 7,
        LANDING         = 8,    //
        LOOPING         = 9
    };
}

#endif // ARDRONE_MODES_H
