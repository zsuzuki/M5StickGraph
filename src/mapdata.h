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

ModelUnit mMdlUnit[] = {
    {&baseMdl, {10.0f, 0.0f, -8.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, -1},
    {&baseMdl, {-10.0f, 0.0f, -3.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {0.7f, 1.2f, 1.0f}, -1},
    {&baseMdl, {15.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 2.0f, 1.0f}, -1},
    {&baseMdl, {-13.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.5f, 1.0f, 1.5f}, -1},
    {&baseMdl, {11.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 0.8f, 1.0f}, -1},
    {&baseMdl, {9.0f, 0.0f, 10.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.1f, 1.1f, 1.3f}, -1},
    {&baseMdl, {2.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, -1},
    {&baseMdl, {-8.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.0f, 1.3f, 1.3f}, -1},
    {&baseMdl, {16.0f, 0.0f, -3.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.4f, 1.8f, 1.1f}, -1},
    {&baseMdl, {-4.0f, 0.0f, -6.0f}, {0.0f, 0.0f, 0.0f, 1.0f}, {1.8f, 0.9f, 1.8f}, -1},
};

int mMdlList[5] = {0, 1, 2, 3, 4};
int mMdlList2[5] = {6, 7, 8, 9, 5};
int mIdxList[64];

MapBlock mBlocks[] = {
    {mMdlList, mIdxList + 0, 25.0f, 5},
    {mMdlList2, mIdxList + 5, 45.0f, 5},
    {mMdlList, mIdxList + 10, 65.0f, 5},
    {mMdlList2, mIdxList + 15, 85.0f, 5},
    {mMdlList, mIdxList + 20, 105.0f, 5},
    {mMdlList2, mIdxList + 25, 125.0f, 5},
    {mMdlList, mIdxList + 30, 145.0f, 5},
    {mMdlList, mIdxList + 35, 165.0f, 5},
    {mMdlList, mIdxList + 40, 185.0f, 5},
    {mMdlList, mIdxList + 45, 205.0f, 5},
};

MapData mData = {mBlocks, mMdlUnit, 215.0f, 10};
