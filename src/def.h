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
    Vector *transBuff;
};
