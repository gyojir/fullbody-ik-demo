
import * as math from "mathjs";
import * as ml from "ml-matrix";

export const RAD_TO_DEG = 180.0 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180.0;

export function inv(mat){
  return math.matrix(ml.inverse(new ml.Matrix(mat.toArray()), true).data.map(e=>[].slice.call(e)));
}

export function mul(...matrices) {
  return matrices.reduce((prev, curr) => math.multiply(prev, curr));
}
export function add(...matrices) {
  return matrices.reduce((prev, curr) => math.add(prev, curr));
}

// math.matrix -> Array
export function matrixToArray3(mat){
  return math.squeeze(mat).toArray().slice(0,3);
}

// 正規化
export function normalize(mat){
  return math.divide(mat, math.norm(mat));
}

// 行列成分抽出
// T*Rx*Rz*Ry*S
// = | Sx*R00 Sy*R01 Sz*R02 Tx |
//   | Sx*R10 Sy*R11 Sz*R12 Ty |
//   | Sx*R20 Sy*R21 Sz*R22 Tz |
//   |      0      0      0  0 |

// スケール
export function getScale(mat){
  return [
    math.norm([mat._data[0][0], mat._data[1][0], mat._data[2][0]]),
    math.norm([mat._data[0][1], mat._data[1][1], mat._data[2][1]]),
    math.norm([mat._data[0][2], mat._data[1][2], mat._data[2][2]]),
  ]
}
// 回転
export function getRotationXYZ(mat){
  // rotX(x) * rotY(y) * rotZ(z) =
  //  |  CyCz        -CySz         Sy   |
  //  |  SxSyCz+CxSz -SxSySz+CxCz -SxCy |
  //  | -CxSyCz+SxSz  CxSySz+SxCz  CxCy |

  const m = cancelScaling(mat).toArray();
  const sq_m12_m22 = Math.sqrt(Math.pow(m[1][2],2) + Math.pow(m[2][2],2));
  
  // Cy==0のときはxを0として計算
  if(Math.abs(sq_m12_m22) < Number.EPSILON){
    return [
      0,
      (Math.PI/2) * m[0][1],
      Math.atan2(m[0][1], m[1][1])]
  }
  else{
    return [
      Math.atan2(-m[1][2], m[2][2]),
      Math.atan2(m[0][2], sq_m12_m22),
      Math.atan2(-m[0][1], m[0][0])]
  }
}

export function getCrossMatrix(v){
  return math.matrix([
    [   0, -v[2],  v[1]],
    [ v[2],    0, -v[0]],
    [-v[1], v[0],     0],
  ]);
}

// 軸を交換する行列
export function getSwapMatrix(x,y,z){
  return math.matrix([
    [ Number(x==0), Number(y==0), Number(z==0), 0 ],
    [ Number(x==1), Number(y==1), Number(z==1), 0 ],
    [ Number(x==2), Number(y==2), Number(z==2), 0 ],
    [            0,            0,            0, 1 ],
  ]);
}

// 3x3行列を4x4行列に拡張
export function expandToMatrix44(mat){
  const ret = math.resize(mat, [4,4]);
  ret._data[3][3] = 1;
  return ret;
}

// 回転行列の誤差ベクトル A - B
export function getRotationError(matA, matB){  
  const diff = mul(math.transpose(matB), matA).toArray();
  return mul(0.5, [
    diff[2][1] - diff[1][2],
    diff[0][2] - diff[2][0],
    diff[1][0] - diff[0][1]
  ]);
}

// 転軸回りに回転する行列を計算
export function getRotationFromAxis(axis, rad){
  const norm = math.norm(axis);
  if(Math.abs(rad) < Number.EPSILON ||
     norm < Number.EPSILON){
    return math.identity(4);
  }
  const normalized_axis = math.divide(axis, norm);
  const crossAxis = getCrossMatrix(normalized_axis);
  // Rodrigues's formula
  // R(θ) = I + nx * sinθ + nx^2 * (1-cosθ)
  return expandToMatrix44(add(math.identity(3), mul(crossAxis, Math.sin(rad)), mul(crossAxis,crossAxis, 1-Math.cos(rad))));
}

// 回転行列から、x軸に直行する回転軸a と 回転後の軸xまわりの回転量を取り出す
// y軸を基準にしたい場合はgetSwapMatrix(2,1,0)とする(x,y,z)->(z,x,y)
export function getRotationSpherical(rot, base = math.identity(4), swap = math.identity(4)){
  // ベースから見た回転
  let m = mul(math.transpose(math.resize(base,[3,3])), math.resize(rot,[3,3]));
  // 軸入れ替え手順(基準ベクトルから見た場合)
  // 1. [1,0,0]を基準にしたい軸になるよう変換。例) [1,0,0](x,y,z) -> [0,1,0](z,x,y)
  // 2. 通常通りの回転を行う。
  // 3. 後の手順でx軸を基準として考えているので戻す。例) z,x,y -> x,y,z
  m = mul(math.transpose(math.resize(swap,[3,3])), m, math.resize(swap,[3,3])).toArray();

  // x軸を合わせるための回転軸a
  const ay = -m[2][0] / Math.sqrt(2*(1 + m[0][0]));
  const az = m[1][0] / Math.sqrt(2*(1 + m[0][0]));

  // a軸まわりにγ回転する座標 と baseから見たrot(m) を比較
  const gamma = Math.asin(Math.sqrt(ay*ay+az*az)) * 2;
  const rot_a = math.resize(getRotationFromAxis([0,ay,az], gamma), [3,3]);
  const rot_twist = mul(math.transpose(rot_a), m).toArray();

  // y軸がどれだけ回転(x軸回り)したか見る
  const ey = [rot_twist[0][1], rot_twist[1][1], rot_twist[2][1]];
  const twist = Math.atan2(ey[2], ey[1]);
  
  return [ay, az, twist, gamma];
}

export function translate(x,y,z) {
  return math.matrix([
    [1.0, 0.0, 0.0, x],
    [0.0, 1.0, 0.0, y],
    [0.0, 0.0, 1.0, z],
    [0.0, 0.0, 0.0, 1.0]])
}

export function translateX(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, dLen],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function translateY(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function translateZ(dLen) {
  return math.matrix(
    [[1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, dLen],
    [0.0, 0.0, 0.0, 1.0]])
}

export function translateAxis(axis, len){
  let mat = axis == 0 ? translateX :
            axis == 1 ? translateY :
            axis == 2 ? translateZ : ()=>1;
  return mat(len);
}

export function diffTranslateX() {
  return math.matrix([
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

export function diffTranslateY() {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

export function diffTranslateZ() {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

export function diffTranslateAxis(axis){
  let mat = axis == 0 ? diffTranslateX :
            axis == 1 ? diffTranslateY :
            axis == 2 ? diffTranslateZ : ()=>1;
  return mat();
}

export function rotX(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, s, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function rotY(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, 0.0, s, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function rotZ(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, -s, 0.0, 0.0],
    [s, c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function rotXYZ(x,y,z) {
  return mul(rotX(x), rotY(y), rotZ(z));
}

export function rotAxis(axis, angle){
  let mat = axis == 0 ? rotX :
            axis == 1 ? rotY :
            axis == 2 ? rotZ : ()=>1;
  return mat(angle);
}

export function diffRotX(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -s, -c, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

export function diffRotY(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [-c, 0.0, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

export function diffRotZ(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, -c, 0.0, 0.0],
    [c, -s, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

export function diffRotAxis(axis, angle){
  let mat = axis == 0 ? diffRotX :
            axis == 1 ? diffRotY :
            axis == 2 ? diffRotZ : ()=>1;
  return mat(angle);
}

export function scale(x,y,z) {
  return math.matrix([
    [  x, 0.0, 0.0, 0.0],
    [0.0,   y, 0.0, 0.0],
    [0.0, 0.0,   z, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

export function axisToVec(axis){
  return [Number(axis == 0),Number(axis == 1),Number(axis == 2)];
}

export function cancelScaling(mat){
  return mul(mat, scale(...getScale(mat).map(e=> Math.abs(e) < Number.EPSILON ? 1 : 1/e)));
}