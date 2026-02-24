#pragma once

#include "types.hpp"

namespace nas {

const std::vector<std::vector<Point_3>> Stairs = {
    // Floor
    {
        Point_3(-1.8, -1., 0.0), // bottom left
        Point_3(0.3, -1., 0.0),  // bottom right
        Point_3(0.3, 1., 0.0),   // top right
        Point_3(-1.8, 1., 0.0)   // top left
    },

    // Step 1
    {
        Point_3(0.3, -0.16, 0.1), // bottom left
        Point_3(0.6, -0.16, 0.1), // bottom right
        Point_3(0.6, 0.6, 0.1),   // top right
        Point_3(0.3, 0.6, 0.1)    // top left
    },

    // Step 2
    {
        Point_3(0.6, -0.16, 0.2), // bottom left
        Point_3(0.9, -0.16, 0.2), // bottom right
        Point_3(0.9, 0.6, 0.2),   // top right
        Point_3(0.6, 0.6, 0.2)    // top left
    },

    // Step 3
    {
        Point_3(0.9, -0.16, 0.3), // bottom left
        Point_3(1.2, -0.16, 0.3), // bottom right
        Point_3(1.2, 0.6, 0.3),   // top right
        Point_3(0.9, 0.6, 0.3)    // top left
    },

    // Step 4
    {
        Point_3(1.2, -0.16, 0.4), // bottom left
        Point_3(1.5, -0.16, 0.4), // bottom right
        Point_3(1.5, 0.6, 0.4),   // top right
        Point_3(1.2, 0.6, 0.4)    // top left
    },
};

const std::vector<std::vector<Point_3>> TwoFlatSurfaces = {
    // Surface 1
    {
        Point_3(0.0, 0.0, 0.0),  // bottom left
        Point_3(5.45, 0.0, 0.0), // bottom right
        Point_3(5.45, 1.0, 0.0), // top right
        Point_3(0.0, 1.0, 0.0)   // top left
    },
    // Surface 2: Another rectangle or surface
    {
        Point_3(5.5, 0.0, 0.0), // bottom left
        Point_3(7.0, 0.0, 0.0), // bottom right
        Point_3(7.0, 1.0, 0.0), // top right
        Point_3(5.5, 1.0, 0.0)  // top left
    },
};

const std::vector<std::vector<Point_3>> Flat = {
    // Surface 1
    {
        Point_3(-2.0, -1.0, 0.0), // bottom left
        Point_3(5.45, -1.0, 0.0), // bottom right
        Point_3(5.45, 1.0, 0.0),  // top right
        Point_3(-2.0, 1.0, 0.0)   // top left
    },
    // // Surface 2: Another rectangle or surface
    // {
    //     Point_3(5.5,  0.0, 0.0),  // bottom left
    //     Point_3(7.0, 0.0, 0.0),  // bottom right
    //     Point_3(7.0, 1.0, 0.0),  // top right
    //     Point_3(5.5,  1.0, 0.0)   // top left
    // },
};

const std::vector<std::vector<Point_3>> LongStairs = {
    // Floor
    {
        Point_3(0.12, 1.0, 0.0),  // bottom left
        Point_3(-0.5, 1.0, 0.0),  // bottom right
        Point_3(-0.5, -1.0, 0.0), // top right
        Point_3(0.12, -1.0, 0.0)  // top left
    },

    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1), // bottom left
        Point_3(0.12, 1.0, 0.1), // bottom right
        Point_3(0.12, 0.5, 0.1), // top right
        Point_3(0.42, 0.5, 0.1)  // top left
    },
    {
        Point_3(0.42, -0.5, 0.1), // bottom left
        Point_3(0.12, -0.5, 0.1), // bottom right
        Point_3(0.12, -1.0, 0.1), // top right
        Point_3(0.42, -1.0, 0.1)  // top left
    },
    // 2nd stair
    {
        Point_3(0.725, 0.8, 0.2), // bottom left
        Point_3(0.425, 0.8, 0.2), // bottom right
        Point_3(0.425, 0, 0.2),   // top right
        Point_3(0.725, 0, 0.2)    // top left
    },
    {
        Point_3(0.725, -0.5, 0.2), // bottom left
        Point_3(0.425, -0.5, 0.2), // bottom right
        Point_3(0.425, -1.0, 0.2), // top right
        Point_3(0.725, -1.0, 0.2)  // top left
    },

    // 3rd stair
    {
        Point_3(1.03, 1.0, 0.3),  // bottom left
        Point_3(0.73, 1.0, 0.3),  // bottom right
        Point_3(0.73, -1.0, 0.3), // top right
        Point_3(1.03, -1.0, 0.3)  // top left
    },

    // 4th stair
    {
        Point_3(1.33, 1.0, 0.4), // bottom left
        Point_3(1.03, 1.0, 0.4), // bottom right
        Point_3(1.03, 0, 0.4),   // top right
        Point_3(1.33, 0, 0.4)    // top left
    },
    {
        Point_3(1.33, -0.5, 0.4), // bottom left
        Point_3(1.03, -0.5, 0.4), // bottom right
        Point_3(1.03, -1.0, 0.4), // top right
        Point_3(1.33, -1.0, 0.4)  // top left
    },

    // 5th stair
    {
        Point_3(1.63, 1.0, 0.5), // bottom left
        Point_3(1.33, 1.0, 0.5), // bottom right
        Point_3(1.33, 0.5, 0.5), // top right
        Point_3(1.63, 0.5, 0.5)  // top left
    },
    {
        Point_3(1.63, -0.5, 0.5), // bottom left
        Point_3(1.33, -0.5, 0.5), // bottom right
        Point_3(1.33, -1.0, 0.5), // top right
        Point_3(1.63, -1.0, 0.5)  // top left
    },

    // 6th stair
    {
        Point_3(1.93, 1.0, 0.6),  // bottom left
        Point_3(1.63, 1.0, 0.6),  // bottom right
        Point_3(1.63, -1.0, 0.6), // top right
        Point_3(1.93, -1.0, 0.6)  // top left
    },

    // 7th stair
    {
        Point_3(2.23, 0.75, 0.7), // bottom left
        Point_3(1.93, 0.75, 0.7), // bottom right
        Point_3(1.93, 0.25, 0.7), // top right
        Point_3(2.23, 0.25, 0.7)  // top left
    },
    {
        Point_3(2.23, -0.5, 0.7), // bottom left
        Point_3(1.93, -0.5, 0.7), // bottom right
        Point_3(1.93, -1.0, 0.7), // top right
        Point_3(2.23, -1.0, 0.7)  // top left
    },

    // 8th stair
    {
        Point_3(2.53, 0.75, 0.8), // bottom left
        Point_3(2.23, 0.75, 0.8), // bottom right
        Point_3(2.23, -0.7, 0.8), // top right
        Point_3(2.53, -0.7, 0.8)  // top left
    },
    {
        Point_3(2.53, -0.5, 0.8), // bottom left
        Point_3(2.23, -0.5, 0.8), // bottom right
        Point_3(2.23, -1.0, 0.8), // top right
        Point_3(2.53, -1.0, 0.8)  // top left
    },

    // 9th stair (skip breaking)
    {
        Point_3(2.83, 0.75, 0.9), // bottom left
        Point_3(2.53, 0.75, 0.9), // bottom right
        Point_3(2.53, 0.25, 0.9), // top right
        Point_3(2.83, 0.25, 0.9)  // top left
    },

    {
        Point_3(2.83, -0.5, 0.9), // bottom left
        Point_3(2.53, -0.5, 0.9), // bottom right
        Point_3(2.53, -1.0, 0.9), // top right
        Point_3(2.83, -1.0, 0.9)  // top left
    },

    // 10th stair
    {
        Point_3(3.13, 1.0, 1.0), // bottom left
        Point_3(2.83, 1.0, 1.0), // bottom right
        Point_3(2.83, 0.0, 1.0), // top right
        Point_3(3.13, 0.0, 1.0)  // top left
    },
    {
        Point_3(3.13, -0.5, 1.0), // bottom left
        Point_3(2.83, -0.5, 1.0), // bottom right
        Point_3(2.83, -1.0, 1.0), // top right
        Point_3(3.13, -1.0, 1.0)  // top left
    },

    // 11th stair
    {
        Point_3(3.43, 0.5, 1.1), // bottom left
        Point_3(3.13, 0.5, 1.1), // bottom right
        Point_3(3.13, 0.0, 1.1), // top right
        Point_3(3.43, 0.0, 1.1)  // top left
    },
    {
        Point_3(3.43, -0.5, 1.1), // bottom left
        Point_3(3.13, -0.5, 1.1), // bottom right
        Point_3(3.13, -1.0, 1.1), // top right
        Point_3(3.43, -1.0, 1.1)  // top left
    },

    // Target floor (12th stair)
    {
        Point_3(3.73, 1.0, 1.2),  // bottom left
        Point_3(3.43, 1.0, 1.2),  // bottom right
        Point_3(3.43, -1.0, 1.2), // top right
        Point_3(3.73, -1.0, 1.2)  // top left
    }};

const std::vector<std::vector<Point_3>> LongLongStairs = {
    // Floor
    {
        Point_3(0.12, 1.0, 0.0),  // top right
        Point_3(-0.5, 1.0, 0.0),  // top left
        Point_3(-0.5, -1.0, 0.0), // bottom left
        Point_3(0.12, -1.0, 0.0)  // bottom right
    },
    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1), // top right
        Point_3(0.12, 1.0, 0.1), // top left
        Point_3(0.12, 0.5, 0.1), // bottom left
        Point_3(0.42, 0.5, 0.1)  // bottom right
    },
    {
        Point_3(0.42, -0.5, 0.1), // top right
        Point_3(0.12, -0.5, 0.1), // top left
        Point_3(0.12, -1.0, 0.1), // bottom left
        Point_3(0.42, -1.0, 0.1)  // bottom right
    },
    // 2nd stair
    {
        Point_3(0.725, 0.8, 0.2), // top right
        Point_3(0.425, 0.8, 0.2), // top left
        Point_3(0.425, 0, 0.2),   // bottom left
        Point_3(0.725, 0, 0.2)    // bottom right
    },
    {
        Point_3(0.725, -0.5, 0.2), // top right
        Point_3(0.425, -0.5, 0.2), // top left
        Point_3(0.425, -1.0, 0.2), // bottom left
        Point_3(0.725, -1.0, 0.2)  // bottom right
    },
    // 3rd stair
    {
        Point_3(1.03, 0.5, 0.3),  // top right
        Point_3(0.73, 0.5, 0.3),  // top left
        Point_3(0.73, -1.0, 0.3), // bottom left
        Point_3(1.03, -1.0, 0.3)  // bottom right
    },
    // 4th stair
    {
        Point_3(1.33, 1.0, 0.4), // top right
        Point_3(1.03, 1.0, 0.4), // top left
        Point_3(1.03, 0, 0.4),   // bottom left
        Point_3(1.33, 0, 0.4)    // bottom right
    },
    {
        Point_3(1.33, -0.5, 0.4), // top right
        Point_3(1.03, -0.5, 0.4), // top left
        Point_3(1.03, -1.0, 0.4), // bottom left
        Point_3(1.33, -1.0, 0.4)  // bottom right
    },
    // 5th stair
    {
        Point_3(1.63, 1.0, 0.5), // top right
        Point_3(1.33, 1.0, 0.5), // top left
        Point_3(1.33, 0.5, 0.5), // bottom left
        Point_3(1.63, 0.5, 0.5)  // bottom right
    },
    {
        Point_3(1.63, -0.5, 0.5), // top right
        Point_3(1.33, -0.5, 0.5), // top left
        Point_3(1.33, -1.0, 0.5), // bottom left
        Point_3(1.63, -1.0, 0.5)  // bottom right
    },
    // 6th stair
    {
        Point_3(1.93, 1.0, 0.6),  // top right
        Point_3(1.63, 1.0, 0.6),  // top left
        Point_3(1.63, -1.0, 0.6), // bottom left
        Point_3(1.93, -1.0, 0.6)  // bottom right
    },
    // 7th stair
    {
        Point_3(2.23, 0.75, 0.7), // top right
        Point_3(1.93, 0.75, 0.7), // top left
        Point_3(1.93, 0.25, 0.7), // bottom left
        Point_3(2.23, 0.25, 0.7)  // bottom right
    },
    {
        Point_3(2.23, -0.5, 0.7), // top right
        Point_3(1.93, -0.5, 0.7), // top left
        Point_3(1.93, -1.0, 0.7), // bottom left
        Point_3(2.23, -1.0, 0.7)  // bottom right
    },
    // 8th stair
    {
        Point_3(2.53, 0.75, 0.8), // top right
        Point_3(2.23, 0.75, 0.8), // top left
        Point_3(2.23, -0.7, 0.8), // bottom left
        Point_3(2.53, -0.7, 0.8)  // bottom right
    },
    {
        Point_3(2.53, -0.5, 0.8), // top right
        Point_3(2.23, -0.5, 0.8), // top left
        Point_3(2.23, -1.0, 0.8), // bottom left
        Point_3(2.53, -1.0, 0.8)  // bottom right
    },
    // 9th stair
    {
        Point_3(2.83, 0.75, 0.9), // top right
        Point_3(2.53, 0.75, 0.9), // top left
        Point_3(2.53, 0.25, 0.9), // bottom left
        Point_3(2.83, 0.25, 0.9)  // bottom right
    },
    {
        Point_3(2.83, -0.5, 0.9), // top right
        Point_3(2.53, -0.5, 0.9), // top left
        Point_3(2.53, -1.0, 0.9), // bottom left
        Point_3(2.83, -1.0, 0.9)  // bottom right
    },
    // 10th stair
    {
        Point_3(3.13, 1.0, 1.0), // top right
        Point_3(2.83, 1.0, 1.0), // top left
        Point_3(2.83, 0.0, 1.0), // bottom left
        Point_3(3.13, 0.0, 1.0)  // bottom right
    },
    {
        Point_3(3.13, -0.5, 1.0), // top right
        Point_3(2.83, -0.5, 1.0), // top left
        Point_3(2.83, -1.0, 1.0), // bottom left
        Point_3(3.13, -1.0, 1.0)  // bottom right
    },
    // 11th stair
    {
        Point_3(3.43, 0.5, 1.1), // top right
        Point_3(3.13, 0.5, 1.1), // top left
        Point_3(3.13, 0.0, 1.1), // bottom left
        Point_3(3.43, 0.0, 1.1)  // bottom right
    },
    {
        Point_3(3.43, -0.5, 1.1), // top right
        Point_3(3.13, -0.5, 1.1), // top left
        Point_3(3.13, -1.0, 1.1), // bottom left
        Point_3(3.43, -1.0, 1.1)  // bottom right
    },
    // 12th floor
    {
        Point_3(3.73, 1.0, 1.2),  // top right
        Point_3(3.43, 1.0, 1.2),  // top left
        Point_3(3.43, -1.0, 1.2), // bottom left
        Point_3(3.73, -1.0, 1.2)  // bottom right
    },
    // Stage 2 surfaces
    // 1st stair stage 2
    {
        Point_3(4.03, 1.0, 1.3), // top right
        Point_3(3.73, 1.0, 1.3), // top left
        Point_3(3.73, 0.5, 1.3), // bottom left
        Point_3(4.03, 0.5, 1.3)  // bottom right
    },
    {
        Point_3(4.03, -0.5, 1.3), // top right
        Point_3(3.73, -0.5, 1.3), // top left
        Point_3(3.73, -1.0, 1.3), // bottom left
        Point_3(4.03, -1.0, 1.3)  // bottom right
    },
    // 2nd stair stage 2
    {
        Point_3(4.335, 0.8, 1.4), // top right
        Point_3(4.035, 0.8, 1.4), // top left
        Point_3(4.035, 0, 1.4),   // bottom left
        Point_3(4.335, 0, 1.4)    // bottom right
    },
    {
        Point_3(4.335, -0.25, 1.4), // top right
        Point_3(4.035, -0.25, 1.4), // top left
        Point_3(4.035, -1.0, 1.4),  // bottom left
        Point_3(4.335, -1.0, 1.4)   // bottom right
    },
    // 3rd stair stage 2
    {
        Point_3(4.64, 0.5, 1.5),  // top right
        Point_3(4.34, 0.5, 1.5),  // top left
        Point_3(4.34, -1.0, 1.5), // bottom left
        Point_3(4.64, -1.0, 1.5)  // bottom right
    },
    // 4th stair stage 2
    {
        Point_3(4.94, 1.0, 1.6), // top right
        Point_3(4.64, 1.0, 1.6), // top left
        Point_3(4.64, 0, 1.6),   // bottom left
        Point_3(4.94, 0, 1.6)    // bottom right
    },
    {
        Point_3(4.94, -0.5, 1.6), // top right
        Point_3(4.64, -0.5, 1.6), // top left
        Point_3(4.64, -1.0, 1.6), // bottom left
        Point_3(4.94, -1.0, 1.6)  // bottom right
    },
    // 5th stair stage 2
    {
        Point_3(5.24, 1.0, 1.7), // top right
        Point_3(4.94, 1.0, 1.7), // top left
        Point_3(4.94, 0.5, 1.7), // bottom left
        Point_3(5.24, 0.5, 1.7)  // bottom right
    },
    {
        Point_3(5.24, -0.5, 1.7), // top right
        Point_3(4.94, -0.5, 1.7), // top left
        Point_3(4.94, -1.0, 1.7), // bottom left
        Point_3(5.24, -1.0, 1.7)  // bottom right
    },
    // 6th stair stage 2
    {
        Point_3(5.54, 1.0, 1.8),  // top right
        Point_3(5.24, 1.0, 1.8),  // top left
        Point_3(5.24, -1.0, 1.8), // bottom left
        Point_3(5.54, -1.0, 1.8)  // bottom right
    },
    // 7th stair stage 2
    {
        Point_3(5.84, 0.75, 1.9), // top right
        Point_3(5.54, 0.75, 1.9), // top left
        Point_3(5.54, 0.25, 1.9), // bottom left
        Point_3(5.84, 0.25, 1.9)  // bottom right
    },
    {
        Point_3(5.84, -0.5, 1.9), // top right
        Point_3(5.54, -0.5, 1.9), // top left
        Point_3(5.54, -1.0, 1.9), // bottom left
        Point_3(5.84, -1.0, 1.9)  // bottom right
    },
    // 8th stair stage 2
    {
        Point_3(6.14, 0.75, 2.0),  // top right
        Point_3(5.84, 0.75, 2.0),  // top left
        Point_3(5.84, -0.45, 2.0), // bottom left
        Point_3(6.14, -0.45, 2.0)  // bottom right
    },
    {
        Point_3(6.14, -0.5, 2.0), // top right
        Point_3(5.84, -0.5, 2.0), // top left
        Point_3(5.84, -1.0, 2.0), // bottom left
        Point_3(6.14, -1.0, 2.0)  // bottom right
    },
    // 9th stair stage 2
    {
        Point_3(6.44, 0.75, 2.1), // top right
        Point_3(6.14, 0.75, 2.1), // top left
        Point_3(6.14, 0.25, 2.1), // bottom left
        Point_3(6.44, 0.25, 2.1)  // bottom right
    },
    {
        Point_3(6.44, -0.5, 2.1), // top right
        Point_3(6.14, -0.5, 2.1), // top left
        Point_3(6.14, -1.0, 2.1), // bottom left
        Point_3(6.44, -1.0, 2.1)  // bottom right
    },
    // 10th stair stage 2
    {
        Point_3(6.74, 1.0, 2.2), // top right
        Point_3(6.44, 1.0, 2.2), // top left
        Point_3(6.44, 0.0, 2.2), // bottom left
        Point_3(6.74, 0.0, 2.2)  // bottom right
    },
    {
        Point_3(6.74, -0.5, 2.2), // top right
        Point_3(6.44, -0.5, 2.2), // top left
        Point_3(6.44, -1.0, 2.2), // bottom left
        Point_3(6.74, -1.0, 2.2)  // bottom right
    },
    // 11th stair stage 2
    {
        Point_3(7.04, 0.5, 2.3), // top right
        Point_3(6.74, 0.5, 2.3), // top left
        Point_3(6.74, 0.0, 2.3), // bottom left
        Point_3(7.04, 0.0, 2.3)  // bottom right
    },
    {
        Point_3(7.04, -0.5, 2.3), // top right
        Point_3(6.74, -0.5, 2.3), // top left
        Point_3(6.74, -1.0, 2.3), // bottom left
        Point_3(7.04, -1.0, 2.3)  // bottom right
    },
    // Target floor stage 2
    {
        Point_3(7.34, 1.0, 2.4),  // top right
        Point_3(7.04, 1.0, 2.4),  // top left
        Point_3(7.04, -1.0, 2.4), // bottom left
        Point_3(7.34, -1.0, 2.4)  // bottom right
    }};

const std::vector<std::vector<Point_3>> LongStairsComplete = {
    // Floor
    {
        Point_3(0.12, 1.0, 0.0),  // bottom left
        Point_3(-0.5, 1.0, 0.0),  // bottom right
        Point_3(-0.5, -1.0, 0.0), // top right
        Point_3(0.12, -1.0, 0.0)  // top left
    },

    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1),  // bottom left
        Point_3(0.12, 1.0, 0.1),  // bottom right
        Point_3(0.12, -1.0, 0.1), // top right
        Point_3(0.42, -1.0, 0.1)  // top left
    },
    // 2nd stair
    {
        Point_3(0.725, 1.0, 0.2),  // bottom left
        Point_3(0.425, 1.0, 0.2),  // bottom right
        Point_3(0.425, -1.0, 0.2), // top right
        Point_3(0.725, -1.0, 0.2)  // top left
    },

    // 3rd stair
    {
        Point_3(1.03, 1.0, 0.3),  // bottom left
        Point_3(0.73, 1.0, 0.3),  // bottom right
        Point_3(0.73, -1.0, 0.3), // top right
        Point_3(1.03, -1.0, 0.3)  // top left
    },

    // 4th stair
    {
        Point_3(1.33, 1.0, 0.4),  // bottom left
        Point_3(1.03, 1.0, 0.4),  // bottom right
        Point_3(1.03, -1.0, 0.4), // top right
        Point_3(1.33, -1.0, 0.4)  // top left
    },

    // 5th stair
    {
        Point_3(1.63, 1.0, 0.5),  // bottom left
        Point_3(1.33, 1.0, 0.5),  // bottom right
        Point_3(1.33, -1.0, 0.5), // top right
        Point_3(1.63, -1.0, 0.5)  // top left
    },

    // 6th stair
    {
        Point_3(1.93, 1.0, 0.6),  // bottom left
        Point_3(1.63, 1.0, 0.6),  // bottom right
        Point_3(1.63, -1.0, 0.6), // top right
        Point_3(1.93, -1.0, 0.6)  // top left
    },

    // 7th stair
    {
        Point_3(2.23, 1.0, 0.7),  // bottom left
        Point_3(1.93, 1.0, 0.7),  // bottom right
        Point_3(1.93, -1.0, 0.7), // top right
        Point_3(2.23, -1.0, 0.7)  // top left
    },

    // 8th stair
    {
        Point_3(2.53, 1.0, 0.8),  // bottom left
        Point_3(2.23, 1.0, 0.8),  // bottom right
        Point_3(2.23, -1.0, 0.8), // top right
        Point_3(2.53, -1.0, 0.8)  // top left
    },

    // 9th stair (skip breaking)
    {
        Point_3(2.83, 1.0, 0.9),  // bottom left
        Point_3(2.53, 1.0, 0.9),  // bottom right
        Point_3(2.53, -1.0, 0.9), // top right
        Point_3(2.83, -1.0, 0.9)  // top left
    },

    // 10th stair
    {
        Point_3(3.13, 1.0, 1.0),  // bottom left
        Point_3(2.83, 1.0, 1.0),  // bottom right
        Point_3(2.83, -1.0, 1.0), // top right
        Point_3(3.13, -1.0, 1.0)  // top left
    },

    // 11th stair
    {
        Point_3(3.43, 1.0, 1.1),  // bottom left
        Point_3(3.13, 1.0, 1.1),  // bottom right
        Point_3(3.13, -1.0, 1.1), // top right
        Point_3(3.43, -1.0, 1.1)  // top left
    },

    // Target floor (12th stair)
    {
        Point_3(3.73, 1.0, 1.2),  // bottom left
        Point_3(3.43, 1.0, 1.2),  // bottom right
        Point_3(3.43, -1.0, 1.2), // top right
        Point_3(3.73, -1.0, 1.2)  // top left
    }};

const std::vector<std::vector<Point_3>> LongStairsExp = {
    // Floor
    {
        Point_3(0.12, 1.0, 0.0),  // bottom left
        Point_3(-0.5, 1.0, 0.0),  // bottom right
        Point_3(-0.5, -1.0, 0.0), // top right
        Point_3(0.12, -1.0, 0.0)  // top left
    },

    // 1st stair
    {
        Point_3(0.42, 1.0, 0.1), // bottom left
        Point_3(0.12, 1.0, 0.1), // bottom right
        Point_3(0.12, 0.5, 0.1), // top right
        Point_3(0.42, 0.5, 0.1)  // top left
    },
    {
        Point_3(0.42, -0.5, 0.1), // bottom left
        Point_3(0.12, -0.5, 0.1), // bottom right
        Point_3(0.12, -1.0, 0.1), // top right
        Point_3(0.42, -1.0, 0.1)  // top left
    },
    // // 2nd stair
    // {
    //     Point_3(0.725, 0.8, 0.2),   // bottom left
    //     Point_3(0.425, 0.8, 0.2),   // bottom right
    //     Point_3(0.425, 0, 0.2),     // top right
    //     Point_3(0.725, 0, 0.2)      // top left
    // },
    // {
    //     Point_3(0.725, -0.5, 0.2),  // bottom left
    //     Point_3(0.425, -0.5, 0.2),  // bottom right
    //     Point_3(0.425, -1.0, 0.2),  // top right
    //     Point_3(0.725, -1.0, 0.2)   // top left
    // },

    // // 3rd stair
    // {
    //     Point_3(1.03, 1.0, 0.3),    // bottom left
    //     Point_3(0.73, 1.0, 0.3),    // bottom right
    //     Point_3(0.73, -1.0, 0.3),   // top right
    //     Point_3(1.03, -1.0, 0.3)    // top left
    // },

    // // 4th stair
    // {
    //     Point_3(1.33, 1.0, 0.4),    // bottom left
    //     Point_3(1.03, 1.0, 0.4),    // bottom right
    //     Point_3(1.03, 0, 0.4),      // top right
    //     Point_3(1.33, 0, 0.4)       // top left
    // },
    // {
    //     Point_3(1.33, -0.5, 0.4),   // bottom left
    //     Point_3(1.03, -0.5, 0.4),   // bottom right
    //     Point_3(1.03, -1.0, 0.4),   // top right
    //     Point_3(1.33, -1.0, 0.4)    // top left
    // },

    // // 5th stair
    // {
    //     Point_3(1.63, 1.0, 0.5),    // bottom left
    //     Point_3(1.33, 1.0, 0.5),    // bottom right
    //     Point_3(1.33, 0.5, 0.5),    // top right
    //     Point_3(1.63, 0.5, 0.5)     // top left
    // },
    // {
    //     Point_3(1.63, -0.5, 0.5),   // bottom left
    //     Point_3(1.33, -0.5, 0.5),   // bottom right
    //     Point_3(1.33, -1.0, 0.5),   // top right
    //     Point_3(1.63, -1.0, 0.5)    // top left
    // },

    // // 6th stair
    // {
    //     Point_3(1.93, 1.0, 0.6),    // bottom left
    //     Point_3(1.63, 1.0, 0.6),    // bottom right
    //     Point_3(1.63, -1.0, 0.6),   // top right
    //     Point_3(1.93, -1.0, 0.6)    // top left
    // },

    // // 7th stair
    // {
    //     Point_3(2.23, 0.75, 0.7),   // bottom left
    //     Point_3(1.93, 0.75, 0.7),   // bottom right
    //     Point_3(1.93, 0.25, 0.7),   // top right
    //     Point_3(2.23, 0.25, 0.7)    // top left
    // },
    // {
    //     Point_3(2.23, -0.5, 0.7),   // bottom left
    //     Point_3(1.93, -0.5, 0.7),   // bottom right
    //     Point_3(1.93, -1.0, 0.7),   // top right
    //     Point_3(2.23, -1.0, 0.7)    // top left
    // },

    // // 8th stair
    // {
    //     Point_3(2.53, 0.75, 0.8),   // bottom left
    //     Point_3(2.23, 0.75, 0.8),   // bottom right
    //     Point_3(2.23, -0.7, 0.8),   // top right
    //     Point_3(2.53, -0.7, 0.8)    // top left
    // },
    // {
    //     Point_3(2.53, -0.5, 0.8),   // bottom left
    //     Point_3(2.23, -0.5, 0.8),   // bottom right
    //     Point_3(2.23, -1.0, 0.8),   // top right
    //     Point_3(2.53, -1.0, 0.8)    // top left
    // },

    // // 9th stair (skip breaking)
    // {
    //     Point_3(2.83, 0.75, 0.9),   // bottom left
    //     Point_3(2.53, 0.75, 0.9),   // bottom right
    //     Point_3(2.53, 0.25, 0.9),   // top right
    //     Point_3(2.83, 0.25, 0.9)    // top left
    // },

    // {
    //     Point_3(2.83, -0.5, 0.9),   // bottom left
    //     Point_3(2.53, -0.5, 0.9),   // bottom right
    //     Point_3(2.53, -1.0, 0.9),   // top right
    //     Point_3(2.83, -1.0, 0.9)    // top left
    // },

    // // 10th stair
    // {
    //     Point_3(3.13, 1.0, 1.0),    // bottom left
    //     Point_3(2.83, 1.0, 1.0),    // bottom right
    //     Point_3(2.83, 0.0, 1.0),    // top right
    //     Point_3(3.13, 0.0, 1.0)     // top left
    // },
    // {
    //     Point_3(3.13, -0.5, 1.0),   // bottom left
    //     Point_3(2.83, -0.5, 1.0),   // bottom right
    //     Point_3(2.83, -1.0, 1.0),   // top right
    //     Point_3(3.13, -1.0, 1.0)    // top left
    // },

    // // 11th stair
    // {
    //     Point_3(3.43, 0.5, 1.1),    // bottom left
    //     Point_3(3.13, 0.5, 1.1),    // bottom right
    //     Point_3(3.13, 0.0, 1.1),    // top right
    //     Point_3(3.43, 0.0, 1.1)     // top left
    // },
    // {
    //     Point_3(3.43, -0.5, 1.1),   // bottom left
    //     Point_3(3.13, -0.5, 1.1),   // bottom right
    //     Point_3(3.13, -1.0, 1.1),   // top right
    //     Point_3(3.43, -1.0, 1.1)    // top left
    // },

    // // Target floor (12th stair)
    // {
    //     Point_3(3.73, 1.0, 1.2),    // bottom left
    //     Point_3(3.43, 1.0, 1.2),    // bottom right
    //     Point_3(3.43, -1.0, 1.2),   // top right
    //     Point_3(3.73, -1.0, 1.2)    // top left
    // }
};

const std::vector<std::vector<Point_3>> ThreePathsScene = {
    // Floor (start)
    {
        Point_3(0.15, 0.3, 0.0),    // bottom left
        Point_3(-0.15, 0.3, 0.0),   // bottom right
        Point_3(-0.15, -0.15, 0.0), // top right
        Point_3(0.15, -0.15, 0.0)   // top left
    },

    // Path Up - pu1
    {
        Point_3(0.15, 0.62, 0.0),  // bottom left
        Point_3(-0.15, 0.62, 0.0), // bottom right
        Point_3(-0.15, 0.32, 0.0), // top right
        Point_3(0.15, 0.32, 0.0)   // top left
    },

    // Path Up - pu2
    {
        Point_3(0.15, 0.94, 0.0),  // bottom left
        Point_3(-0.15, 0.94, 0.0), // bottom right
        Point_3(-0.15, 0.64, 0.0), // top right
        Point_3(0.15, 0.64, 0.0)   // top left
    },

    // Path Up - pu3
    {
        Point_3(0.25, 1.175, 0.0),  // bottom left
        Point_3(-0.05, 1.175, 0.0), // bottom right
        Point_3(-0.05, 0.875, 0.0), // top right
        Point_3(0.25, 0.875, 0.0)   // top left
    },

    // Path Up - pu4
    {
        Point_3(0.55, 1.175, 0.0), // bottom left
        Point_3(0.3, 1.175, 0.0),  // bottom right
        Point_3(0.3, 0.875, 0.0),  // top right
        Point_3(0.55, 0.875, 0.0)  // top left
    },

    // Path Up - pu5
    {
        Point_3(0.9, 1.175, 0.0), // bottom left
        Point_3(0.6, 1.175, 0.0), // bottom right
        Point_3(0.6, 0.875, 0.0), // top right
        Point_3(0.9, 0.875, 0.0)  // top left
    },

    // Path Up - pu6
    {
        Point_3(1.25, 1.175, 0.0), // bottom left
        Point_3(0.95, 1.175, 0.0), // bottom right
        Point_3(0.95, 0.875, 0.0), // top right
        Point_3(1.25, 0.875, 0.0)  // top left
    },

    // Path Up - pu7
    {
        Point_3(1.6, 1.175, 0.0), // bottom left
        Point_3(1.3, 1.175, 0.0), // bottom right
        Point_3(1.3, 0.875, 0.0), // top right
        Point_3(1.6, 0.875, 0.0)  // top left
    },

    // Path Up - pu8
    {
        Point_3(1.95, 1.175, 0.0), // bottom left
        Point_3(1.65, 1.175, 0.0), // bottom right
        Point_3(1.65, 0.875, 0.0), // top right
        Point_3(1.95, 0.875, 0.0)  // top left
    },

    // Path Up - pu9
    {
        Point_3(2.05, 0.85, 0.0), // bottom left
        Point_3(1.75, 0.85, 0.0), // bottom right
        Point_3(1.75, 0.55, 0.0), // top right
        Point_3(2.05, 0.55, 0.0)  // top left
    },

    // Path Up - pu10
    {
        Point_3(2.05, 0.5, 0.0), // bottom left
        Point_3(1.75, 0.5, 0.0), // bottom right
        Point_3(1.75, 0.2, 0.0), // top right
        Point_3(2.05, 0.2, 0.0)  // top left
    },

    // Path Down - pd1
    {
        Point_3(0.15, -0.5, 0.0),  // bottom left
        Point_3(-0.15, -0.5, 0.0), // bottom right
        Point_3(-0.15, -0.2, 0.0), // top right
        Point_3(0.15, -0.2, 0.0)   // top left
    },

    // Path Down - pd2
    {
        Point_3(0.15, -0.85, 0.0),  // bottom left
        Point_3(-0.15, -0.85, 0.0), // bottom right
        Point_3(-0.15, -0.55, 0.0), // top right
        Point_3(0.15, -0.55, 0.0)   // top left
    },

    // Path Down - pd3
    {
        Point_3(0.25, -1.175, 0.0),  // bottom left
        Point_3(-0.05, -1.175, 0.0), // bottom right
        Point_3(-0.05, -0.875, 0.0), // top right
        Point_3(0.25, -0.875, 0.0)   // top left
    },

    // Path Down - pd4
    {
        Point_3(0.55, -1.175, 0.0), // bottom left
        Point_3(0.3, -1.175, 0.0),  // bottom right
        Point_3(0.3, -0.875, 0.0),  // top right
        Point_3(0.55, -0.875, 0.0)  // top left
    },

    // Path Down - pd5
    {
        Point_3(0.9, -1.175, 0.0), // bottom left
        Point_3(0.6, -1.175, 0.0), // bottom right
        Point_3(0.6, -0.875, 0.0), // top right
        Point_3(0.9, -0.875, 0.0)  // top left
    },

    // Path Down - pd6
    {
        Point_3(1.25, -1.175, 0.0), // bottom left
        Point_3(0.95, -1.175, 0.0), // bottom right
        Point_3(0.95, -0.875, 0.0), // top right
        Point_3(1.25, -0.875, 0.0)  // top left
    },

    // Path Down - pd7
    {
        Point_3(1.6, -1.175, 0.0), // bottom left
        Point_3(1.3, -1.175, 0.0), // bottom right
        Point_3(1.3, -0.875, 0.0), // top right
        Point_3(1.6, -0.875, 0.0)  // top left
    },

    // Path Down - pd8
    {
        Point_3(1.95, -1.175, 0.0), // bottom left
        Point_3(1.65, -1.175, 0.0), // bottom right
        Point_3(1.65, -0.875, 0.0), // top right
        Point_3(1.95, -0.875, 0.0)  // top left
    },

    // Path Down - pd9
    {
        Point_3(2.05, -0.85, 0.0), // bottom left
        Point_3(1.75, -0.85, 0.0), // bottom right
        Point_3(1.75, -0.55, 0.0), // top right
        Point_3(2.05, -0.55, 0.0)  // top left
    },

    // Path Down - pd10
    {
        Point_3(2.05, -0.5, 0.0), // bottom left
        Point_3(1.75, -0.5, 0.0), // bottom right
        Point_3(1.75, -0.2, 0.0), // top right
        Point_3(2.05, -0.2, 0.0)  // top left
    },

    // Dead end path
    {
        Point_3(0.5, 0.15, 0.0),  // bottom left
        Point_3(0.2, 0.15, 0.0),  // bottom right
        Point_3(0.2, -0.15, 0.0), // top right
        Point_3(0.5, -0.15, 0.0)  // top left
    },

    // Target
    {
        Point_3(2.05, 0.15, 0.0),  // bottom left
        Point_3(1.75, 0.15, 0.0),  // bottom right
        Point_3(1.75, -0.15, 0.0), // top right
        Point_3(2.05, -0.15, 0.0)  // top left
    }};

const std::vector<std::vector<Point_3>> Stairs_Up_Down = {
    // Floor
    {
        Point_3(-1.8, -1., 0.0), // bottom left
        Point_3(0.6, -1., 0.0),  // bottom right
        Point_3(0.6, 1., 0.0),   // top right
        Point_3(-1.8, 1., 0.0)   // top left
    },

    // Step 1
    {
        Point_3(0.6, -0.16, 0.0), // bottom left
        Point_3(1.2, -0.16, 0.0), // bottom right
        Point_3(1.2, 0.6, 0.0),   // top right
        Point_3(0.6, 0.6, 0.0)    // top left
    },

    // Step 2
    {
        Point_3(1.2, -0.16, 0.1), // bottom left
        Point_3(1.8, -0.16, 0.1), // bottom right
        Point_3(1.8, 0.6, 0.1),   // top right
        Point_3(1.2, 0.6, 0.1)    // top left
    },

    // Step 3
    {
        Point_3(1.8, -0.16, 0.2), // bottom left
        Point_3(2.4, -0.16, 0.2), // bottom right
        Point_3(2.4, 0.6, 0.2),   // top right
        Point_3(1.8, 0.6, 0.2)    // top left
    },

    // Step 4
    {
        Point_3(2.4, -0.16, 0.3), // bottom left
        Point_3(3.0, -0.16, 0.3), // bottom right
        Point_3(3.0, 0.6, 0.3),   // top right
        Point_3(2.4, 0.6, 0.3)    // top left
    },

    // Step 5
    {
        Point_3(3.0, -0.16, 0.2), // bottom left
        Point_3(3.6, -0.16, 0.2), // bottom right
        Point_3(3.6, 0.6, 0.2),   // top right
        Point_3(3.0, 0.6, 0.2)    // top left
    },

    // Step 6
    {
        Point_3(3.6, -0.16, 0.1), // bottom left
        Point_3(4.2, -0.16, 0.1), // bottom right
        Point_3(4.2, 0.6, 0.1),   // top right
        Point_3(3.6, 0.6, 0.1)    // top left
    },

    // Step 7
    {
        Point_3(4.2, -0.16, 0.0), // bottom left
        Point_3(4.8, -0.16, 0.0), // bottom right
        Point_3(4.8, 0.6, 0.0),   // top right
        Point_3(4.2, 0.6, 0.0)    // top left
    }

};

const std::vector<std::vector<Point_3>> ThreePathsNAS = {
    // Starting platform (larger for initial stance)
    {
        Point_3(-0.3, -4.0, 0.0), // bottom left
        Point_3(0.3, -4.0, 0.0),  // bottom right
        Point_3(0.3, 2.0, 0.0),   // top right
        Point_3(-0.3, 2.0, 0.0)   // top left
    },

    // Up Path 1
    {
        Point_3(0.32, 1.4, 0.0), // bottom left
        Point_3(0.72, 1.4, 0.0), // bottom right
        Point_3(0.72, 2.0, 0.0), // top right
        Point_3(0.32, 2.0, 0.0)  // top left
    },

    // Up Path 2
    {
        Point_3(0.74, 1.4, 0.0), // bottom left
        Point_3(1.32, 1.4, 0.0), // bottom right
        Point_3(1.32, 2.0, 0.0), // top right
        Point_3(0.74, 2.0, 0.0)  // top left
    },

    // Up Path 3
    {
        Point_3(1.34, 1.4, 0.0), // bottom left
        Point_3(2.52, 1.4, 0.0), // bottom right
        Point_3(2.52, 2.0, 0.0), // top right
        Point_3(1.34, 2.0, 0.0)  // top left
    },

    // Up Path 4
    {
        Point_3(2.54, 1.4, 0.0), // bottom left
        Point_3(3.12, 1.4, 0.0), // bottom right
        Point_3(3.12, 2.0, 0.0), // top right
        Point_3(2.54, 2.0, 0.0)  // top left
    },

    // Up Path 5
    {
        Point_3(3.14, 1.4, 0.0), // bottom left
        Point_3(3.72, 1.4, 0.0), // bottom right
        Point_3(3.72, 2.0, 0.0), // top right
        Point_3(3.14, 2.0, 0.0)  // top left
    },

    // Up Path 5
    {
        Point_3(3.74, 1.4, 0.0), // bottom left
        Point_3(4.32, 1.4, 0.0), // bottom right
        Point_3(4.32, 2.0, 0.0), // top right
        Point_3(3.74, 2.0, 0.0)  // top left
    },

    // Down
    {
        Point_3(0.32, -3.2, 0.0), // bottom left
        Point_3(1.2, -3.2, 0.0),  // bottom right
        Point_3(1.2, -4.0, 0.0),  // top right
        Point_3(0.32, -4.0, 0.0)  // top left
    },

    {
        Point_3(1.22, -3.2, 0.0), // bottom left
        Point_3(2.2, -3.2, 0.0),  // bottom right
        Point_3(2.2, -4.0, 0.0),  // top right
        Point_3(1.22, -4.0, 0.0)  // top left
    },

    {
        Point_3(2.22, -3.2, 0.0), // bottom left
        Point_3(3.2, -3.2, 0.0),  // bottom right
        Point_3(3.2, -4.0, 0.0),  // top right
        Point_3(2.22, -4.0, 0.0)  // top left
    },

    {
        Point_3(3.22, -3.2, 0.0), // bottom left
        Point_3(4.32, -3.2, 0.0), // bottom right
        Point_3(4.32, -4.0, 0.0), // top right
        Point_3(3.22, -4.0, 0.0)  // top left
    },

    // Dead end
    {
        Point_3(0.32, -1.0, 0.0), // bottom left
        Point_3(1.5, -1.0, 0.0),  // bottom right
        Point_3(1.5, -2.0, 0.0),  // top right
        Point_3(0.32, -2.0, 0.0)  // top left
    },

    // End Surface
    {
        Point_3(4.34, -4.0, 0.0), // bottom left
        Point_3(4.94, -4.0, 0.0), // bottom right
        Point_3(4.94, 2.0, 0.0),  // top right
        Point_3(4.34, 2.0, 0.0)   // top left
    },
};

const std::vector<std::vector<Point_3>> NarrowPassage = {

    // Starting Floor
    {
        Point_3(-2.0, -2.0, 0.0), // bottom left
        Point_3(2.0, -2.0, 0.0),  // bottom right
        Point_3(2.0, 2.0, 0.0),   // top right
        Point_3(-2.0, 2.0, 0.0)   // top left
    },

    // Passage
    {
        Point_3(2.0, -0.1, 0.0), // bottom left
        Point_3(6.0, -0.1, 0.0), // bottom right
        Point_3(6.0, 0.1, 0.0),  // top right
        Point_3(2.0, 0.1, 0.0)   // top left
    },

    // Last Floor
    {
        Point_3(6.0, -2.0, 0.0),  // bottom left
        Point_3(10.0, -2.0, 0.0), // bottom right
        Point_3(10.0, 2.0, 0.0),  // top right
        Point_3(6.0, 2.0, 0.0)    // top left
    }

};

const std::vector<std::vector<Point_3>> AllTerrain = []() {
  std::vector<std::vector<Point_3>> combined;

  // 1. Add Stairs_Up_Down (X range: [-1.8, 4.2])
  combined.insert(combined.end(), Stairs_Up_Down.begin(), Stairs_Up_Down.end());

  // 2. Add ThreePathsNAS (Original X: [-0.3, 4.94])
  // Shift to connect to Stairs_Up_Down end (X=4.8). Offset = 4.8 - (-0.3)
  // = 5.1
  Vector_3 shift2(5.1, 0.0, 0.0);
  for (const auto &surf : ThreePathsNAS) {
    std::vector<Point_3> shifted_surf;
    for (const auto &pt : surf) {
      shifted_surf.push_back(pt + shift2);
    }
    combined.push_back(shifted_surf);
  }

  //   // 3. Add NarrowPassage (Original X: [-2.0, 10.0])
  //   // Shift to connect to shifted ThreePathsNAS end (X=9.44). Offset = 9.44
  //   + 0.6
  //   // -
  //   // (-2.0) = 12.04 //11.44
  //   Vector_3 shift3(12.04, 0.0, 0.0);
  //   for (const auto &surf : NarrowPassage) {
  //     std::vector<Point_3> shifted_surf;
  //     for (const auto &pt : surf) {
  //       shifted_surf.push_back(pt + shift3);
  //     }
  //     combined.push_back(shifted_surf);
  //   }

  return combined;
}();

} // namespace nas
