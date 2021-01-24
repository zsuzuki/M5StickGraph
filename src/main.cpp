#include <M5StickC.h>
#include <time.h>
#define LGFX_AUTODETECT
#include <LovyanGFX.hpp>

static LGFX lcd;

static constexpr const int ScreenWidth = 80;
static constexpr const int ScreenHeight = 160;
static constexpr const float Left = -1.0f;
static constexpr const float Right = 1.0f;
static constexpr const float Top = ((float)ScreenHeight / (float)ScreenWidth);
static constexpr const float Bottom = -((float)ScreenHeight / (float)ScreenWidth);
static constexpr const float Near = 1.0f;
static constexpr const float Far = 100.0f;
static constexpr const int NbTransBuff = 1024;

#include "def.h"
//
#include "modeldata.h"
#include "mapdata.h"

ModelUnit model = {
    &planeMdl,
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    -1};
ModelUnit cursor = {
    &pentaMdl,
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    -1};

// 透視変換行列
const Matrix frustum = {
    2.0f * Near / (Right - Left),
    0.0f,
    (Right + Left) / (Right - Left),
    0.0f,

    0.0f,
    2.0f * Near / (Top - Bottom),
    (Top + Bottom) / (Top - Bottom),
    0.0f,

    0.0f,
    0.0f,
    -((Far + Near) / (Far - Near)),
    (-2.0f * Far * Near) / (Far - Near),

    0.0f,
    0.0f,
    -1.0f,
    0.0f,
};
// ビュー変換
Quaternion modelViewQ, modelViewQN;
// ビュースタック
Matrix modelView;
Matrix mvStack[2];
int mvStackIdx = 0;
Vector transBuffer[NbTransBuff];
Vector eraseBuffer[NbTransBuff];
int transIdx;

// 画面消去用
struct DrawBBox : public BoundingBox
{
  DrawBBox() : BoundingBox(ScreenWidth, ScreenHeight) {}

  void draw()
  {
    // 一応データは取るけど描画が遅いので消去には使わない
    // if (enabled)
    //   lcd.fillRect(dl, dt, w, h, TFT_BLACK);
    enabled = false;
  }
};

DrawBBox bboxList[64];
int nbUseBBox;
int nbReqBBox;

// 前のフレームで描画した部分を消去
void clearPrevDraw()
{
  for (int i = 0; i < nbUseBBox; i++)
  {
    auto &bb = bboxList[i];
    bb.draw();
  }
  for (int i = 0; i < nbReqBBox; i++)
  {
    auto &bb = bboxList[i];
    bb.fix();
    bb.clear();
  }
  nbUseBBox = nbReqBBox;
  nbReqBBox = 0;
}
//
DrawBBox &getBBox()
{
  return bboxList[nbReqBBox++];
}

//
void IdentityMatrix(Matrix &m)
{
  for (int i = 0; i < 16; i++)
  {
    m[i] = (i % 5 == 0) ? 1.0f : 0.0f;
  }
}

// クォータニアンからマトリクスへ
void Quat2Mat(Matrix &m, const Quaternion &q)
{
  const float x = q[0];
  const float y = q[1];
  const float z = q[2];
  const float w = q[3];
  const float xy2 = 2.0f * x * y;
  const float wz2 = 2.0f * w * z;
  const float xz2 = 2.0f * x * z;
  const float wy2 = 2.0f * w * y;
  const float yz2 = 2.0f * y * z;
  const float wx2 = 2.0f * w * x;

  m[0] = 1.0f - 2.0f * y * y - 2.0f * z * z;
  m[1] = xy2 + wz2;
  m[2] = xz2 - wy2;
  m[3] = 0.0f;
  m[4] = xy2 - wz2;
  m[5] = 1.0f - 2.0f * x * x - 2.0f * z * z;
  m[6] = yz2 + wx2;
  m[7] = 0.0f;
  m[8] = xz2 + wy2;
  m[9] = yz2 - wx2;
  m[10] = 1.0f - 2.0f * x * x - 2.0f * y * y;
  m[11] = 0.0f;
  m[12] = 0.0f;
  m[13] = 0.0f;
  m[14] = 0.0f;
  m[15] = 1.0f;
}

// 逆変換クォータニアン
void QuatNeg(Quaternion &rq, const Quaternion &q)
{
  rq[3] = q[3];
  rq[0] = -q[0];
  rq[1] = -q[1];
  rq[2] = -q[2];
}

// 正規化
void QuatUnit(Quaternion &q)
{
  auto l = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  l = 1.0f / sqrt(l);
  q[0] *= l;
  q[1] *= l;
  q[2] *= l;
  q[3] *= l;
}

// クォータニアン乗算
void QuatMul(Quaternion &rq, const Quaternion &q0, const Quaternion &q1)
{
  double w0 = q0[3], w1 = q1[3];
  double x0 = q0[0], x1 = q1[0];
  double y0 = q0[1], y1 = q1[1];
  double z0 = q0[2], z1 = q1[2];

  double d0, d1, d2, d3;
  d0 = w0 * w1;
  d1 = x0 * x1;
  d2 = y0 * y1;
  d3 = z0 * z1;
  rq[3] = d0 - d1 - d2 - d3;

  d0 = w0 * x1;
  d1 = x0 * w1;
  d2 = y0 * z1;
  d3 = z0 * y1;
  rq[0] = d0 + d1 + d2 - d3;

  d0 = w0 * y1;
  d1 = y0 * w1;
  d2 = z0 * x1;
  d3 = x0 * z1;
  rq[1] = d0 + d1 + d2 - d3;

  d0 = w0 * z1;
  d1 = z0 * w1;
  d2 = x0 * y1;
  d3 = y0 * x1;
  rq[2] = d0 + d1 + d2 - d3;
}

// クォータニアンの回転
void QuatRot(Quaternion &rq, const Quaternion &q, const Quaternion &b)
{
  Quaternion n, tmp;
  QuatNeg(n, q);
  QuatMul(tmp, q, b);
  QuatMul(rq, tmp, n);
}

// ベクトル正規化
void UnitVector(Vector &v)
{
  float x = v[0];
  float y = v[1];
  float z = v[2];
  float l = 1.0f / sqrt(x * x + y * y + z * z);
  v[0] = x * l;
  v[1] = y * l;
  v[2] = z * l;
}

// マトリクスにベクタをかける
void MultiVector(Vector &r, const Matrix &m, const Vector &v)
{
  const float tx = v[0];
  const float ty = v[1];
  const float tz = v[2];
  float x = tx * m[0] + ty * m[1] + tz * m[2];
  float y = tx * m[4] + ty * m[5] + tz * m[6];
  float z = tx * m[8] + ty * m[9] + tz * m[10];
  r[0] = x;
  r[1] = y;
  r[2] = z;
}
// 移動
void MoveMatrix(Matrix &m, float x, float y, float z)
{
  float tx = x * m[0] + y * m[1] + z * m[2];
  float ty = x * m[4] + y * m[5] + z * m[6];
  float tz = x * m[8] + y * m[9] + z * m[10];
  m[3] += tx;
  m[7] += ty;
  m[11] += tz;
}
// ベクタの座標変換
void Translate(Vector &r, const Matrix &m, const Vector &v)
{
  const float tx = v[0];
  const float ty = v[1];
  const float tz = v[2];
  float x = tx * m[0] + ty * m[1] + tz * m[2] + m[3];
  float y = tx * m[4] + ty * m[5] + tz * m[6] + m[7];
  float z = tx * m[8] + ty * m[9] + tz * m[10] + m[11];
  r[0] = x;
  r[1] = y;
  r[2] = z;
}
// 転置
void TransposeMatrix(Matrix &m)
{
  auto m01 = m[1];
  auto m02 = m[2];
  auto m10 = m[4];
  auto m12 = m[6];
  auto m20 = m[8];
  auto m21 = m[9];
  m[1] = m10;
  m[2] = m20;
  m[4] = m01;
  m[6] = m21;
  m[8] = m02;
  m[9] = m12;
}

// マトリクスでの回転
void RotateMatrix(Matrix &t, const Matrix &r)
{
  for (int i = 0; i < 3; i++)
  {
    int idx = i * 4;
    float x = t[idx];
    float y = t[idx + 1];
    float z = t[idx + 2];
    t[idx + 0] = x * r[0] + y * r[4] + z * r[8];
    t[idx + 1] = x * r[1] + y * r[5] + z * r[9];
    t[idx + 2] = x * r[2] + y * r[6] + z * r[10];
  }
}

// マトリクスのスケール
void ScaleMatrix(Matrix &m, float x, float y, float z)
{
  m[0] *= x;
  m[1] *= y;
  m[2] *= z;
  m[4] *= x;
  m[5] *= y;
  m[6] *= z;
  m[8] *= x;
  m[9] *= y;
  m[10] *= z;
}

// マトリクス同士の乗算
void MultiMatrix(Matrix &rm, const Matrix &m0, const Matrix &m1)
{
  for (int i = 0; i < 4; i++)
  {
    int r = i * 4;
    for (int j = 0; j < 4; j++)
    {
      int ri = r + j;
      int ci = j + i;
      rm[ri] = m0[ri + 0] * m1[ci + 0] + m0[ri + 1] * m1[ci + 4] + m0[ri + 2] * m1[ci + 8] + m0[ri + 3] * m1[ci + 12];
    }
  }
}

// 透視変換
void TranslateView(Vector &r, const Vector &v)
{
  Translate(r, frustum, v);

  auto z = r[2];
  if (z < 0.0f)
  {
    auto x = r[0] / z;
    auto y = r[1] / z;
    // 中心のずれた行列には対応しない
    if (abs(x) < Right && abs(y) < Top)
    {
      constexpr const float halfW = (float)ScreenWidth * 0.5f;
      constexpr const float halfH = (float)ScreenHeight * 0.5f;

      r[0] = x * halfW + halfW;
      r[1] = y * halfH + halfH;
    }
    else
      r[2] = 1.0f;
  }
}

// 外積
void OuterVector(Vector &r, const Vector &v0, const Vector &v1)
{
  float x0 = v0[0], x1 = v1[0];
  float y0 = v0[1], y1 = v1[1];
  float z0 = v0[2], z1 = v1[2];
  r[0] = y0 * z1 - y1 * z0;
  r[1] = z0 * x1 - z1 * x0;
  r[2] = x0 * y1 - x1 * y0;
}

// カメラ
void SetCamera(const Vector &eye, const Vector &target, const Vector &up)
{
  float ex = -eye[0];
  float ey = -eye[1];
  float ez = -eye[2];
  float evx = target[0] + ex;
  float evy = target[1] + ey;
  float evz = target[2] + ez;
  float evl = 1.0f / sqrt(evx * evx + evy * evy + evz * evz);
  Vector front, side, tup;
  front[0] = evx * evl;
  front[1] = evy * evl;
  front[2] = evz * evl;
  OuterVector(side, up, front);
  UnitVector(side);
  OuterVector(tup, front, side);
  UnitVector(tup);
  modelView[0] = side[0];
  modelView[1] = tup[0];
  modelView[2] = front[0];
  modelView[3] = 0.0f;
  modelView[4] = side[1];
  modelView[5] = tup[1];
  modelView[6] = front[1];
  modelView[7] = 0.0f;
  modelView[8] = side[2];
  modelView[9] = tup[2];
  modelView[10] = front[2];
  modelView[11] = 0.0f;
  modelView[12] = 0.0f;
  modelView[13] = 0.0f;
  modelView[14] = 0.0f;
  modelView[15] = 1.0f;
  TransposeMatrix(modelView);
  MoveMatrix(modelView, ex, ey, ez);
}

// マトリックススタック操作
void PushMV()
{
  auto &t = mvStack[mvStackIdx++];
  memcpy(t, modelView, sizeof(Matrix));
}
void PopMV()
{
  auto &t = mvStack[--mvStackIdx];
  memcpy(modelView, t, sizeof(Matrix));
}

//
void transSetup()
{
  transIdx = 0;
}

// モデル座標変換
void CalcModel(ModelUnit &mdlunit)
{
  auto &mdl = *mdlunit.model;
  if (transIdx + mdl.numPos >= NbTransBuff)
  {
    // 頂点バッファが足りない
    mdlunit.transIdx = -1;
    return;
  }

  PushMV();
  MoveMatrix(modelView, mdlunit.position[0], mdlunit.position[1], mdlunit.position[2]);
  Matrix p;
  Quat2Mat(p, mdlunit.posture);
  RotateMatrix(modelView, p);
  ScaleMatrix(modelView, mdlunit.scale[0], mdlunit.scale[1], mdlunit.scale[2]);
  mdlunit.transIdx = transIdx;
  auto &dbb = getBBox();
  for (int i = 0; i < mdl.numPos; i++)
  {
    Vector dst;
    Translate(dst, modelView, mdl.point[i]);
    auto &tb = transBuffer[transIdx++];
    TranslateView(tb, dst);
    dbb.update(tb[0], tb[1]);
  }
  PopMV();
}

// 描画したエッジ単位で消去する
struct EraseEdge
{
  float x0, y0, x1, y1;
};
EraseEdge eraseEdge[1024];
int nbErase = 0;

// モデル描画
void DrawModel(const ModelUnit &mdlunit)
{
  if (mdlunit.transIdx < 0)
  {
    // バッファが足りない or 変換されていない
    return;
  }
  auto &mdl = *mdlunit.model;
  Vector *renderBuff = transBuffer + mdlunit.transIdx;
  for (int i = 0; i < mdl.numEdge; i++)
  {
    auto &e = mdl.edge[i];
    auto p0 = renderBuff[e.p0];
    auto p1 = renderBuff[e.p1];
    if (p0[2] < 0.0f && p1[2] < 0.0f)
    {
      // 描画しつつそのエッジ情報を保存する
      auto x0 = p0[0];
      auto y0 = p0[1];
      auto x1 = p1[0];
      auto y1 = p1[1];
      lcd.drawLine(x0, y0, x1, y1, e.color);
      auto &ee = eraseEdge[nbErase++];
      ee.x0 = x0;
      ee.y0 = y0;
      ee.x1 = x1;
      ee.y1 = y1;
    }
  }
}

// 描画したエッジを消去
void eraseEdges()
{
  for (int i = 0; i < nbErase; i++)
  {
    auto &e = eraseEdge[i];
    lcd.drawLine(e.x0, e.y0, e.x1, e.y1, TFT_BLACK);
  }
  nbErase = 0;
}

// マップ生成
MapData *genMap(const MapGenDef *mgd)
{
  auto mapData = new MapData;
  const int nbMdl = mgd->nbMdlList;
  const int nbInBlk = mgd->nbMdlInBlock;
  const int nbBlk = mgd->nbBlocks;
  const auto nbTotalMdl = nbInBlk * nbBlk;
  const auto length = mgd->blockLength;
  mapData->blocks = new MapBlock[nbBlk];
  mapData->models = new ModelUnit[nbTotalMdl];
  mapData->nbBlock = mgd->nbBlocks;
  randomSeed(millis());
  // generate model
  for (int mi = 0; mi < nbTotalMdl; mi++)
  {
    auto &mdl = mapData->models[mi];
    mdl.model = mgd->mdlList[mi % nbMdl];
    auto px = (float)(random(0, 2000) - 1000) * 0.001f;
    auto py = (float)(random(0, 2000) - 1000) * 0.001f;
    auto pz = (float)(random(0, 2000) - 1000) * 0.001f;
    mdl.position[0] = px * mgd->blockWidth;
    mdl.position[1] = py * mgd->blockHeight;
    mdl.position[2] = pz * length;
    mdl.transIdx = -1;
    // TODO
    mdl.posture[0] = 0.0f;
    mdl.posture[1] = 0.0f;
    mdl.posture[2] = 0.0f;
    mdl.posture[3] = 1.0f;
    mdl.scale[0] = 1.0f;
    mdl.scale[1] = 1.0f;
    mdl.scale[2] = 1.0f;
  }
  // generate block
  constexpr float offsetLength = 20.0f;
  mapData->length = offsetLength;
  for (int bi = 0; bi < nbBlk; bi++)
  {
    auto &block = mapData->blocks[bi];
    block.modelList = new int[nbInBlk];
    for (int i = 0; i < nbInBlk; i++)
    {
      block.modelList[i] = bi * nbInBlk + i;
    }
    block.idxList = new int[nbInBlk];
    block.nbModels = nbInBlk;
    block.pos = bi * length + offsetLength;

    mapData->length += mgd->blockLength;
  }
  return mapData;
}

//
void CalcMap(MapData &mapdata)
{
  static float v = 0.0f;
  for (int i = 0; i < mapdata.nbBlock; i++)
  {
    auto &block = mapdata.blocks[i];
    if ((v - 10.0f) > block.pos || (v + 50.0f) < block.pos)
    {
      block.draw = false;
      continue;
    }
    block.draw = true;
    PushMV();
    MoveMatrix(modelView, 0.0f, 0.0f, block.pos - v);
    for (int j = 0; j < block.nbModels; j++)
    {
      int mi = block.modelList[j];
      auto &mdl = mapdata.models[mi];
      int oi = mdl.transIdx;
      CalcModel(mdl);
      block.idxList[j] = mdl.transIdx;
      mdl.transIdx = oi;
    }
    PopMV();
  }
  v += 0.2f;
  if (v > mapdata.length)
    v = 0.0f;
}
//
void DrawMap(MapData &mapdata)
{
  for (int i = 0; i < mapdata.nbBlock; i++)
  {
    auto &block = mapdata.blocks[i];
    if (block.draw == false)
      continue;
    for (int j = 0; j < block.nbModels; j++)
    {
      int mi = block.modelList[j];
      auto &mdl = mapdata.models[mi];
      int oi = mdl.transIdx;
      mdl.transIdx = block.idxList[j];
      DrawModel(mdl);
      mdl.transIdx = oi;
    }
  }
}

// アプリ初期化
void setup()
{
  lcd.init();
  lcd.setRotation(0);
  lcd.setBrightness(80);

  lcd.setColor(TFT_NAVY);

  M5.MPU6886.Init();

  for (auto &bb : bboxList)
  {
    bb.clear();
    bb.fix();
  }
  nbUseBBox = 0;
  nbReqBBox = 0;

  mapData = genMap(&mGenData);
  delay(100);
}

// アプリループ
void loop()
{
  auto st = millis();

  static unsigned long dt = 0;
  static unsigned long udt = 0;
  static float accX = 0.0f;
  static float accY = 0.0f;
  static float accZ = 0.0f;
  static float gyroX = 0.0f;
  static float gyroY = 0.0f;
  static float gyroZ = 0.0f;

  M5.MPU6886.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.MPU6886.getAccelData(&accX, &accY, &accZ);

  if (digitalRead(M5_BUTTON_HOME) == LOW)
  {
  }

  transSetup();
  {
    constexpr const float Pi = 3.14159265f;
    static int cnt = 0;
    float rb = (float)cnt / 360.0f;
    cnt = (cnt + 1) % 360;

    float rad = rb * Pi;

    // object
    model.posture[0] = 0.0f;
    model.posture[1] = sin(rad);
    model.posture[2] = 0.0f;
    model.posture[3] = cos(rad);
    model.position[0] = 0.0f;
    model.position[1] = 2.5f;
    model.position[2] = 3.0f;
    // camera
    Quaternion rot, rotn, rotx, roty;
    int rxi = max(min(accZ * 50.0f, 100.0f), -100.0f);
    int ryi = max(min(accX * 50.0f, 100.0f), -100.0f);
    float rx = (float)rxi * Pi * 0.003f;
    float ry = (float)ryi * Pi * 0.003f;
    rotx[0] = sin(rx);
    rotx[1] = 0.0f;
    rotx[2] = 0.0f;
    rotx[3] = cos(rx);
    roty[0] = 0.0f;
    roty[1] = sin(ry);
    roty[2] = 0.0f;
    roty[3] = cos(ry);
    QuatMul(rot, rotx, roty);
    QuatUnit(rot);
    QuatNeg(rotn, rot);
    Quaternion te, ev = {0.0f, 5.0f, -10.0f, 0.0f}, eu = {0.0f, 1.0f, 0.0f, 0.0f};
    QuatMul(te, rot, ev);
    QuatMul(ev, te, rotn);
    QuatMul(te, rot, eu);
    QuatMul(eu, te, rotn);
    Vector eye, up;
    eye[0] = ev[0];
    eye[1] = ev[1];
    eye[2] = ev[2] + 10.0f;
    up[0] = eu[0];
    up[1] = eu[1];
    up[2] = eu[2];
    Vector tgt = {0.0f, 2.0f, 10.0f};
    SetCamera(eye, tgt, up);
    // display
    CalcMap(*mapData);
    CalcModel(model);
  }

  udt = millis() - st;
  lcd.startWrite();
  {
    clearPrevDraw();
    eraseEdges();
    lcd.fillRect(0, 4, 32, 8, TFT_LIGHTGREY);
    lcd.fillRect(0, 5, udt * 2, 6, TFT_GREEN);
    lcd.fillRect(udt * 2 + 1, 5, dt * 2, 6, TFT_RED);

    DrawMap(*mapData);
    DrawModel(model);
  }
  lcd.endWrite();
  dt = millis() - st;
  if (dt < 15)
    delay(16 - dt);
}
