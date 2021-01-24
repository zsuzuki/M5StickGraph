#pragma once

using Matrix = float[16];
using Vector = float[3];
using Quaternion = float[4];

// 辺
struct Edge
{
    int p0;
    int p1;
    int color;
};

// 簡易モデル構造
struct Model
{
    Vector *point;
    Edge *edge;
    int numPos;
    int numEdge;
};

// モデルインスタンス
struct ModelUnit
{
    Model *model;
    Vector position;
    Quaternion posture;
    Vector scale;
    int transIdx;
};

// マップ1ブロック
struct MapBlock
{
    int *modelList;
    int *idxList;
    float pos;
    int nbModels;
    bool draw;
};

// バウンディングボックス
struct BoundingBox
{
    int left, right, top, bottom;
    int dl, dt, w, h;
    int W, H;
    bool enabled;

    BoundingBox(int ww, int hh) : W(ww), H(hh) { clear(); }

    void clear()
    {
        left = W;
        right = 0;
        top = H;
        bottom = 0;
    }

    void update(int x, int y)
    {
        if (left > x)
            left = x;
        if (right < x)
            right = x;
        if (top > y)
            top = y;
        if (bottom < y)
            bottom = y;
    }

    void fix()
    {
        w = right - left;
        h = bottom - top;
        if (w < 0)
        {
            w = -w;
            dl = right;
        }
        else
            dl = left;
        if (h < 0)
        {
            h = -h;
            dt = bottom;
        }
        else
            dt = top;
        w++;
        h++;
        enabled = (dl < W && dt < H && (dl + W >= 0) && (dt + H >= 0));
    }
};

// マップデータ
struct MapData
{
    MapBlock *blocks;
    ModelUnit *models;
    float length;
    int nbBlock;
};

// マップ生成データ
struct MapGenDef
{
    Model **mdlList;
    int nbMdlList;
    float blockWidth;
    float blockHeight;
    float blockLength;
    int nbMdlInBlock;
    int nbBlocks;
};
