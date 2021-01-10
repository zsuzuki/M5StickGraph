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
    int transIdx;
};

// バウンディングボックス
template <int W, int H>
struct BoundingBox
{
    int left, right, top, bottom;
    int dl, dt, w, h;

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
    }
};
