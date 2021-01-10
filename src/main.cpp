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

ModelUnit kage = {
    &cubeMdl,
    {0.0f, 0.0f, 7.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    -1};
ModelUnit model = {
    &planeMdl,
    {0.0f, 0.0f, 7.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    -1};
ModelUnit cursor = {
    &pentaMdl,
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
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
int transIdx;

// 画面消去用
struct DrawBBox : public BoundingBox<ScreenWidth, ScreenHeight>
{
  void draw()
  {
    lcd.fillRect(dl, dt, w, h, TFT_BLACK);
  }
};
DrawBBox dbb;

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
  float evx = target[0] - eye[0];
  float evy = target[1] - eye[1];
  float evz = target[2] - eye[2];
  float evl = 1.0f / sqrt(evx * evx + evy * evy + evz * evz);
  Vector front, side, tup;
  front[0] = evx * evl;
  front[1] = evy * evl;
  front[2] = evz * evl;
  OuterVector(side, front, up);
  UnitVector(side);
  OuterVector(tup, side, front);
  UnitVector(tup);
  modelView[0] = side[0];
  modelView[1] = side[1];
  modelView[2] = side[2];
  modelView[3] = eye[0];
  modelView[4] = tup[0];
  modelView[5] = tup[1];
  modelView[6] = tup[2];
  modelView[7] = eye[1];
  modelView[8] = front[0];
  modelView[9] = front[1];
  modelView[10] = front[2];
  modelView[11] = eye[2];
  modelView[12] = 0.0f;
  modelView[13] = 0.0f;
  modelView[14] = 0.0f;
  modelView[15] = 1.0f;
  TransposeMatrix(modelView);
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
  mdlunit.transIdx = transIdx;
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

// モデル描画
void DrawModel(const ModelUnit &mdlunit)
{
  if (mdlunit.transIdx < 0)
  {
    // バッファが足りない or 変換されていない
    return;
  }
  auto &mdl = *mdlunit.model;
  Vector *transBuff = &transBuffer[mdlunit.transIdx];
  for (int i = 0; i < mdl.numEdge; i++)
  {
    auto &e = mdl.edge[i];
    auto p0 = transBuff[e.p0];
    auto p1 = transBuff[e.p1];
    if (p0[2] < 0.0f && p1[2] < 0.0f)
      lcd.drawLine(p0[0], p0[1], p1[0], p1[1], e.color);
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

  dbb.clear();
  dbb.fix();

  delay(100);
}

float accX = 0;
float accY = 0;
float accZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

float mX = 80;
float mY = 40;
float omX = 80;
float omY = 40;
float aX = 0;
float aY = 0;

unsigned long dt = 0;
unsigned long udt = 0;

// アプリループ
void loop()
{
  auto st = millis();

  M5.MPU6886.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.MPU6886.getAccelData(&accX, &accY, &accZ);

  //FastLED.clear();
  //for (int i = 0; i < NUM_LEDS; i++) {
  //leds[i] = CRGB( 0, 255, 0);
  //FastLED.show();
  //delay(100);
  //}

  if (digitalRead(M5_BUTTON_HOME) == LOW)
  {
    mX = 80;
    mY = 40;
  }
  omX = mX;
  omY = mY;
  aX += accX * 2.0f;
  aY += accY * 2.0f;
  mX -= aX * 0.5f;
  mY += aY * 0.5f;
  aX *= 0.98f;
  aY *= 0.98f;
  constexpr const float margin = 5.0f;
  if (mX < margin)
  {
    mX = margin;
    aX = -aX;
  }
  else if (mX > (ScreenWidth - margin - 1.0f))
  {
    mX = ScreenWidth - margin - 1.0f;
    aX = -aX;
  }
  if (mY < margin)
  {
    mY = margin;
    aY = -aY;
  }
  else if (mY > (ScreenHeight - margin - 1.0f))
  {
    mY = ScreenHeight - margin - 1.0f;
    aY = -aY;
  }

  transSetup();
  {
    constexpr const float Pi = 3.14159265f;
    static int cnt = 0;
    float rb = (float)cnt / 360.0f;
    cnt = (cnt + 1) % 360;

    float rad = rb * Pi;
    float prad = rb * Pi * 2.0f;

    // object
    model.posture[0] = 0.0f;
    model.posture[1] = sin(rad);
    model.posture[2] = 0.0f;
    model.posture[3] = cos(rad);
    model.position[0] = cos(prad) * 4.0f;
    model.position[1] = 0.0f;
    model.position[2] = sin(prad) * 4.0f + 7.0f;
    cursor.posture[0] = 0.7071f;
    cursor.posture[1] = 0.0f;
    cursor.posture[2] = 0.0f;
    cursor.posture[3] = 0.7071f;
    cursor.position[0] = mX / 16.0f - 2.5f;
    cursor.position[1] = -(mY / 16.0f - 5.0f);
    cursor.position[2] = 7.0f;
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
    Quaternion te, ev = {0.0f, 0.0f, -8.0f, 0.0f}, eu = {0.0f, 1.0f, 0.0f, 0.0f};
    QuatMul(te, rot, ev);
    QuatMul(ev, te, rotn);
    QuatMul(te, rot, eu);
    QuatMul(eu, te, rotn);
    Vector eye, up;
    eye[0] = ev[0];
    eye[1] = ev[1];
    eye[2] = ev[2] + 7.0f;
    up[0] = eu[0];
    up[1] = eu[1];
    up[2] = eu[2];
    Vector tgt = {0.0f, 0.0f, 7.0f};
    SetCamera(eye, tgt, up);
    // display
    CalcModel(model);
    CalcModel(kage);
    CalcModel(cursor);
  }

  udt = millis() - st;
  lcd.startWrite();
  {
    //lcd.fillRect(0, 0, 80, 160, TFT_NAVY);
    dbb.draw();
    dbb.fix();
    dbb.clear();
    lcd.fillRect(0, 4, 32, 8, TFT_LIGHTGREY);
    lcd.fillRect(0, 5, udt * 2, 6, TFT_GREEN);
    lcd.fillRect(udt * 2 + 1, 5, dt * 2, 6, TFT_RED);

    DrawModel(model);
    DrawModel(kage);
    DrawModel(cursor);
  }
  lcd.endWrite();
  dt = millis() - st;
  delay(16 - dt);
}
