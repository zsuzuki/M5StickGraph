#pragma once

#include "def.h"

// model on map
Vector basePos[6] = {
    {0.0f, 0.0f, 1.0f},
    {0.951f, 0.0f, 0.309f},
    {0.5877f, 0.0f, -0.809f},
    {-0.5877f, 0.0f, -0.809f},
    {-0.951f, 0.0f, 0.309f},
    {0.0f, 1.0f, 0.0f},
};
// yellow base
Edge baseIdx[10] = {
    {0, 1, TFT_YELLOW},
    {1, 2, TFT_YELLOW},
    {2, 3, TFT_YELLOW},
    {3, 4, TFT_YELLOW},
    {4, 0, TFT_YELLOW},
    {0, 5, TFT_YELLOW},
    {1, 5, TFT_YELLOW},
    {2, 5, TFT_YELLOW},
    {3, 5, TFT_YELLOW},
    {4, 5, TFT_YELLOW},
};
Model baseMdl = {basePos, baseIdx, 6, 10};
// brown base
Edge baseIdx2[10] = {
    {0, 1, TFT_BROWN},
    {1, 2, TFT_BROWN},
    {2, 3, TFT_BROWN},
    {3, 4, TFT_BROWN},
    {4, 0, TFT_BROWN},
    {0, 5, TFT_BROWN},
    {1, 5, TFT_BROWN},
    {2, 5, TFT_BROWN},
    {3, 5, TFT_BROWN},
    {4, 5, TFT_BROWN},
};
Model baseMdl2 = {basePos, baseIdx2, 6, 10};

Model *mUseMdlList[] = {&baseMdl, &baseMdl2};

MapGenDef mGenData = {
    mUseMdlList, (int)(sizeof(mUseMdlList) / sizeof(Model *)),
    16.0f, 0.1f, 16.0f,
    8,
    20};

MapData *mapData;
