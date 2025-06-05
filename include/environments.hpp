#pragma once

#include "types.hpp"

namespace nas {

const std::vector<std::vector<Point_3>> Stairs = {
    // Floor
    {
        Point_3(-1.8, -1., 0.0),   // bottom left
        Point_3(0.3, -1., 0.0),   // bottom right
        Point_3(0.3, 1., 0.0),    // top right
        Point_3(-1.8, 1., 0.0)     // top left
    },

    // Step 1
    {
        Point_3(0.3, -0.16, 0.1),   // bottom left
        Point_3(0.6, -0.16, 0.1),    // bottom right
        Point_3(0.6, 0.6, 0.1),      // top right
        Point_3(0.3, 0.6, 0.1)      // top left
    },

    // Step 2
    {
        Point_3(0.6, -0.16, 0.2),    // bottom left
        Point_3(0.9, -0.16, 0.2),    // bottom right
        Point_3(0.9, 0.6, 0.2),      // top right
        Point_3(0.6, 0.6, 0.2)       // top left
    },

    // Step 3
    {
        Point_3(0.9, -0.16, 0.3),    // bottom left
        Point_3(1.2, -0.16, 0.3),    // bottom right
        Point_3(1.2, 0.6, 0.3),      // top right
        Point_3(0.9, 0.6, 0.3)       // top left
    },

    // Step 4
    {
        Point_3(1.2, -0.16, 0.4),    // bottom left
        Point_3(1.5, -0.16, 0.4),    // bottom right
        Point_3(1.5, 0.6, 0.4),      // top right
        Point_3(1.2, 0.6, 0.4)       // top left
    },
};

const std::vector<std::vector<Point_3>> TwoFlatSurfaces = {
    // Surface 1
    {
        Point_3(0.0, 0.0, 0.0),  // bottom left
        Point_3(5.45, 0.0, 0.0),  // bottom right
        Point_3(5.45, 1.0, 0.0),  // top right
        Point_3(0.0, 1.0, 0.0)   // top left
    },
    // Surface 2: Another rectangle or surface
    {
        Point_3(5.5,  0.0, 0.0),  // bottom left
        Point_3(7.0, 0.0, 0.0),  // bottom right
        Point_3(7.0, 1.0, 0.0),  // top right
        Point_3(5.5,  1.0, 0.0)   // top left
    },
};

const std::vector<std::vector<Point_3>> LongStairs = {
    // Floor
    {
        Point_3(0.12, 1.0, 0.0),    // bottom left
        Point_3(-0.5, 1.0, 0.0),    // bottom right
        Point_3(-0.5, -1.0, 0.0),   // top right
        Point_3(0.12, -1.0, 0.0)    // top left
    },
    
    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1),    // bottom left
        Point_3(0.12, 1.0, 0.1),    // bottom right
        Point_3(0.12, 0.5, 0.1),    // top right
        Point_3(0.42, 0.5, 0.1)     // top left
    },
    {
        Point_3(0.42, -0.5, 0.1),   // bottom left
        Point_3(0.12, -0.5, 0.1),   // bottom right
        Point_3(0.12, -1.0, 0.1),   // top right
        Point_3(0.42, -1.0, 0.1)    // top left
    },
    // 2nd stair
    {
        Point_3(0.725, 0.8, 0.2),   // bottom left
        Point_3(0.425, 0.8, 0.2),   // bottom right
        Point_3(0.425, 0, 0.2),     // top right
        Point_3(0.725, 0, 0.2)      // top left
    },
    {
        Point_3(0.725, -0.5, 0.2),  // bottom left
        Point_3(0.425, -0.5, 0.2),  // bottom right
        Point_3(0.425, -1.0, 0.2),  // top right
        Point_3(0.725, -1.0, 0.2)   // top left
    },

    // 3rd stair
    {
        Point_3(1.03, 1.0, 0.3),    // bottom left
        Point_3(0.73, 1.0, 0.3),    // bottom right
        Point_3(0.73, -1.0, 0.3),   // top right
        Point_3(1.03, -1.0, 0.3)    // top left
    },

    // 4th stair
    {
        Point_3(1.33, 1.0, 0.4),    // bottom left
        Point_3(1.03, 1.0, 0.4),    // bottom right
        Point_3(1.03, 0, 0.4),      // top right
        Point_3(1.33, 0, 0.4)       // top left
    },
    {
        Point_3(1.33, -0.5, 0.4),   // bottom left
        Point_3(1.03, -0.5, 0.4),   // bottom right
        Point_3(1.03, -1.0, 0.4),   // top right
        Point_3(1.33, -1.0, 0.4)    // top left
    },

    // 5th stair
    {
        Point_3(1.63, 1.0, 0.5),    // bottom left
        Point_3(1.33, 1.0, 0.5),    // bottom right
        Point_3(1.33, 0.5, 0.5),    // top right
        Point_3(1.63, 0.5, 0.5)     // top left
    },
    {
        Point_3(1.63, -0.5, 0.5),   // bottom left
        Point_3(1.33, -0.5, 0.5),   // bottom right
        Point_3(1.33, -1.0, 0.5),   // top right
        Point_3(1.63, -1.0, 0.5)    // top left
    },

    // 6th stair
    {
        Point_3(1.93, 1.0, 0.6),    // bottom left
        Point_3(1.63, 1.0, 0.6),    // bottom right
        Point_3(1.63, -1.0, 0.6),   // top right
        Point_3(1.93, -1.0, 0.6)    // top left
    },

    // 7th stair
    {
        Point_3(2.23, 0.75, 0.7),   // bottom left
        Point_3(1.93, 0.75, 0.7),   // bottom right
        Point_3(1.93, 0.25, 0.7),   // top right
        Point_3(2.23, 0.25, 0.7)    // top left
    },
    {
        Point_3(2.23, -0.5, 0.7),   // bottom left
        Point_3(1.93, -0.5, 0.7),   // bottom right
        Point_3(1.93, -1.0, 0.7),   // top right
        Point_3(2.23, -1.0, 0.7)    // top left
    },

    // 8th stair
    {
        Point_3(2.53, 0.75, 0.8),   // bottom left
        Point_3(2.23, 0.75, 0.8),   // bottom right
        Point_3(2.23, -0.7, 0.8),   // top right
        Point_3(2.53, -0.7, 0.8)    // top left
    },
    {
        Point_3(2.53, -0.5, 0.8),   // bottom left
        Point_3(2.23, -0.5, 0.8),   // bottom right
        Point_3(2.23, -1.0, 0.8),   // top right
        Point_3(2.53, -1.0, 0.8)    // top left
    },

    // 9th stair (skip breaking)
    {
        Point_3(2.83, 0.75, 0.9),   // bottom left
        Point_3(2.53, 0.75, 0.9),   // bottom right
        Point_3(2.53, 0.25, 0.9),   // top right
        Point_3(2.83, 0.25, 0.9)    // top left
    },

    {
        Point_3(2.83, -0.5, 0.9),   // bottom left
        Point_3(2.53, -0.5, 0.9),   // bottom right
        Point_3(2.53, -1.0, 0.9),   // top right
        Point_3(2.83, -1.0, 0.9)    // top left
    },


    // 10th stair
    {
        Point_3(3.13, 1.0, 1.0),    // bottom left
        Point_3(2.83, 1.0, 1.0),    // bottom right
        Point_3(2.83, 0.0, 1.0),    // top right
        Point_3(3.13, 0.0, 1.0)     // top left
    },
    {
        Point_3(3.13, -0.5, 1.0),   // bottom left
        Point_3(2.83, -0.5, 1.0),   // bottom right
        Point_3(2.83, -1.0, 1.0),   // top right
        Point_3(3.13, -1.0, 1.0)    // top left
    },

    // 11th stair
    {
        Point_3(3.43, 0.5, 1.1),    // bottom left
        Point_3(3.13, 0.5, 1.1),    // bottom right
        Point_3(3.13, 0.0, 1.1),    // top right
        Point_3(3.43, 0.0, 1.1)     // top left
    },
    {
        Point_3(3.43, -0.5, 1.1),   // bottom left
        Point_3(3.13, -0.5, 1.1),   // bottom right
        Point_3(3.13, -1.0, 1.1),   // top right
        Point_3(3.43, -1.0, 1.1)    // top left
    },

    // Target floor (12th stair)
    {
        Point_3(3.73, 1.0, 1.2),    // bottom left
        Point_3(3.43, 1.0, 1.2),    // bottom right
        Point_3(3.43, -1.0, 1.2),   // top right
        Point_3(3.73, -1.0, 1.2)    // top left
    }
};

const std::vector<std::vector<Point_3>> LongLongStairs = {

    //floor
    {
        Point_3(0.12, 1.0, 0.0),    // bottom left
        Point_3(-0.5, 1.0, 0.0),    // bottom right
        Point_3(-0.5, -1.0, 0.0),   // top right
        Point_3(0.12, -1.0, 0.0)    // top left
    },

    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1),    // bottom left
        Point_3(0.12, 1.0, 0.1),    // bottom right
        Point_3(0.12, 0.5, 0.1),    // top right
        Point_3(0.42, 0.5, 0.1)     // top left
    },
    {
        Point_3(0.42, -0.5, 0.1),   // bottom left
        Point_3(0.12, -0.5, 0.1),   // bottom right
        Point_3(0.12, -1.0, 0.1),   // top right
        Point_3(0.42, -1.0, 0.1)    // top left
    },

    // 2nd stair
    {
        Point_3(0.725, 0.8, 0.2),   // bottom left
        Point_3(0.425, 0.8, 0.2),   // bottom right
        Point_3(0.425, 0, 0.2),     // top right
        Point_3(0.725, 0, 0.2)      // top left
    },
    {
        Point_3(0.725, -0.5, 0.2),  // bottom left
        Point_3(0.425, -0.5, 0.2),  // bottom right
        Point_3(0.425, -1.0, 0.2),  // top right
        Point_3(0.725, -1.0, 0.2)   // top left
    },

    // 3rd stair
    {
        Point_3(1.03, 0.5, 0.3),    // bottom left
        Point_3(0.73, 0.5, 0.3),    // bottom right
        Point_3(0.73, -1.0, 0.3),   // top right
        Point_3(1.03, -1.0, 0.3)    // top left
    },
    
    // 4th stair
    {
        Point_3(1.33, 1.0, 0.4),    // bottom left
        Point_3(1.03, 1.0, 0.4),    // bottom right
        Point_3(1.03, 0, 0.4),      // top right
        Point_3(1.33, 0, 0.4)       // top left
    },
    {
        Point_3(1.33, -0.5, 0.4),   // bottom left
        Point_3(1.03, -0.5, 0.4),   // bottom right
        Point_3(1.03, -1.0, 0.4),   // top right
        Point_3(1.33, -1.0, 0.4)    // top left
    },

    // 5th stair
    {
        Point_3(1.63, 1.0, 0.5),    // bottom left
        Point_3(1.33, 1.0, 0.5),    // bottom right
        Point_3(1.33, 0.5, 0.5),    // top right
        Point_3(1.63, 0.5, 0.5)     // top left
    },
    {
        Point_3(1.63, -0.5, 0.5),   // bottom left
        Point_3(1.33, -0.5, 0.5),   // bottom right
        Point_3(1.33, -1.0, 0.5),   // top right
        Point_3(1.63, -1.0, 0.5)    // top left
    },

    // 6th stair
    {
        Point_3(1.93, 1.0, 0.6),    // bottom left
        Point_3(1.63, 1.0, 0.6),    // bottom right
        Point_3(1.63, -1.0, 0.6),   // top right
        Point_3(1.93, -1.0, 0.6)    // top left
    },

    // 7th stair
    {
        Point_3(2.23, 0.75, 0.7),   // bottom left
        Point_3(1.93, 0.75, 0.7),   // bottom right
        Point_3(1.93, 0.25, 0.7),   // top right
        Point_3(2.23, 0.25, 0.7)    // top left
    },
    {
        Point_3(2.23, -0.5, 0.7),   // bottom left
        Point_3(1.93, -0.5, 0.7),   // bottom right
        Point_3(1.93, -1.0, 0.7),   // top right
        Point_3(2.23, -1.0, 0.7)    // top left
    },

    // 8th stair
    {
        Point_3(2.53, 0.75, 0.8),   // bottom left
        Point_3(2.23, 0.75, 0.8),   // bottom right
        Point_3(2.23, -0.7, 0.8),   // top right
        Point_3(2.53, -0.7, 0.8)    // top left
    },
    {
        Point_3(2.53, -0.5, 0.8),   // bottom left
        Point_3(2.23, -0.5, 0.8),   // bottom right
        Point_3(2.23, -1.0, 0.8),   // top right
        Point_3(2.53, -1.0, 0.8)    // top left
    },

    // 9th stair
    {
        Point_3(2.83, 0.75, 0.9),   // bottom left
        Point_3(2.53, 0.75, 0.9),   // bottom right
        Point_3(2.53, 0.25, 0.9),   // top right
        Point_3(2.83, 0.25, 0.9)    // top left
    },
    {
        Point_3(2.83, -0.5, 0.9),   // bottom left
        Point_3(2.53, -0.5, 0.9),   // bottom right
        Point_3(2.53, -1.0, 0.9),   // top right
        Point_3(2.83, -1.0, 0.9)    // top left
    },

    // 10th stair
    {
        Point_3(3.13, 1.0, 1.0),    // bottom left
        Point_3(2.83, 1.0, 1.0),    // bottom right
        Point_3(2.83, 0.0, 1.0),    // top right
        Point_3(3.13, 0.0, 1.0)     // top left
    },
    {
        Point_3(3.13, -0.5, 1.0),   // bottom left
        Point_3(2.83, -0.5, 1.0),   // bottom right
        Point_3(2.83, -1.0, 1.0),   // top right
        Point_3(3.13, -1.0, 1.0)    // top left
    },

    // 11th stair
    {
        Point_3(3.43, 0.5, 1.1),    // bottom left
        Point_3(3.13, 0.5, 1.1),    // bottom right
        Point_3(3.13, 0.0, 1.1),    // top right
        Point_3(3.43, 0.0, 1.1)     // top left
    },
    {
        Point_3(3.43, -0.5, 1.1),   // bottom left
        Point_3(3.13, -0.5, 1.1),   // bottom right
        Point_3(3.13, -1.0, 1.1),   // top right
        Point_3(3.43, -1.0, 1.1)    // top left
    },

    // 12th stair
    {
        Point_3(3.73, 1.0, 1.2),    // bottom left
        Point_3(3.43, 1.0, 1.2),    // bottom right
        Point_3(3.43, -1.0, 1.2),   // top right
        Point_3(3.73, -1.0, 1.2)    // top left
    },    

    // step1_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step2_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step3_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },

    // step4_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step5_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step6_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },

    // step7_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step8_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step9_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step10_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step11_1_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
    {
        Point_3(4.03, -0.5, 1.3),   // bottom left
        Point_3(3.73, -0.5, 1.3),   // bottom right
        Point_3(3.73, -1.0, 1.3),   // top right
        Point_3(4.03, -1.0, 1.3)    // top left
    },

    // step12_stage_2
    {
        Point_3(4.03, 1.0, 1.3),    // bottom left
        Point_3(3.73, 1.0, 1.3),    // bottom right
        Point_3(3.73, 0.5, 1.3),    // top right
        Point_3(4.03, 0.5, 1.3)     // top left
    },
};

}