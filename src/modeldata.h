#pragma once

#include "def.h"

// Cubeデータ
Vector cubePos[8] = {
    {-2.5f, -5.0f, 1.5f},
    {2.5f, -5.0f, 1.5f},
    {2.5f, 5.0f, 1.5f},
    {-2.5f, 5.0f, 1.5f},
    {-2.5f, -5.0f, -1.5f},
    {2.5f, -5.0f, -1.5f},
    {2.5f, 5.0f, -1.5f},
    {-2.5f, 5.0f, -1.5f},
};
Vector cubeBuff[8];
Edge cubeIdx[12] = {
    {0, 1, TFT_YELLOW},
    {1, 2, TFT_YELLOW},
    {2, 3, TFT_YELLOW},
    {3, 0, TFT_YELLOW},
    {0, 4, TFT_YELLOW},
    {3, 7, TFT_YELLOW},
    {2, 6, TFT_YELLOW},
    {1, 5, TFT_YELLOW},
    {4, 5, TFT_RED},
    {5, 6, TFT_RED},
    {6, 7, TFT_RED},
    {7, 4, TFT_RED},
};
Model cubeMdl = {
    cubePos,
    cubeIdx,
    8,
    12};

// 戦闘機
Vector planePos[14] = {
    // ceter
    {0.0f, 0.0f, 1.4f},  // 0
    {0.0f, 0.0f, -1.5f}, // 1
    {0.0f, 0.6f, 0.0f},  // 2
    {0.0f, 0.3f, 1.4f},  // 3
    {0.0f, -0.1f, 2.0f}, // 4
    {0.0f, 1.0f, -2.0f}, // 5
    // right
    {0.6f, 0.0f, 0.0f},   // 6
    {0.4f, 0.0f, -1.5f},  // 7
    {0.2f, -0.4f, -1.5f}, // 8
    {1.2f, -0.6f, -1.2f}, // 9
    // left
    {-0.6f, 0.0f, 0.0f},   // 10
    {-0.4f, 0.0f, -1.5f},  // 11
    {-0.2f, -0.4f, -1.5f}, // 12
    {-1.2f, -0.6f, -1.2f}, // 13
};
Vector planeBuff[14];
Edge planeIdx[] = {
    // 6
    {1, 5, TFT_SKYBLUE},
    {5, 2, TFT_SKYBLUE},
    {2, 3, TFT_SKYBLUE},
    {3, 4, TFT_SKYBLUE},
    {4, 0, TFT_SKYBLUE},
    {2, 1, TFT_SKYBLUE},
    // 7
    {0, 6, TFT_YELLOW},
    //{6, 7, TFT_YELLOW},
    {1, 7, TFT_YELLOW},
    {7, 8, TFT_YELLOW},
    {7, 9, TFT_YELLOW},
    {9, 6, TFT_YELLOW},
    {8, 6, TFT_RED},
    {8, 9, TFT_YELLOW},
    // 7
    {0, 10, TFT_YELLOW},
    //{10, 11, TFT_YELLOW},
    {1, 11, TFT_YELLOW},
    {11, 12, TFT_YELLOW},
    {11, 13, TFT_YELLOW},
    {13, 10, TFT_YELLOW},
    {12, 10, TFT_RED},
    {12, 13, TFT_YELLOW},
};
Model planeMdl = {
    planePos,
    planeIdx,
    14,
    20};

// 五角形
Vector pentaPos[5] = {
    {0.0f, 0.0f, 1.0f},
    {0.951f, 0.0f, 0.309f},
    {0.5877f, 0.0f, -0.809f},
    {-0.5877f, 0.0f, -0.809f},
    {-0.951f, 0.0f, 0.309f},
};
Vector pentaBuff[5];
Edge pentaIdx[] = {
    {0, 1, TFT_YELLOW},
    {1, 2, TFT_YELLOW},
    {2, 3, TFT_YELLOW},
    {3, 4, TFT_YELLOW},
    {4, 0, TFT_YELLOW},
};
Model pentaMdl = {pentaPos, pentaIdx, 5, 5};
