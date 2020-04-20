
import * as math from "mathjs";
import * as ml from "ml-matrix";
import { FArray3, FArray4 } from "./def";

export const RAD_TO_DEG = 180.0 / Math.PI;
export const DEG_TO_RAD = Math.PI / 180.0;

// 逆行列ラップ
export function inv(mat: math.Matrix): math.Matrix {
  return math.matrix(ml.inverse(new ml.Matrix(mat.toArray() as number[][]), true).to2DArray());
}

// 行列積ラップ
export function mul<T extends math.MathType = math.Matrix>(...matrices: math.MathType[]): T {
  return matrices.reduce((prev, curr) => math.multiply(prev, curr)) as T;
}

// 行列和ラップ
export function add<T extends math.MathType = math.Matrix>(...matrices: math.MathType[]): T {
  return matrices.reduce((prev, curr) => math.add(prev, curr)) as T;
}

// 単位行列ラップ
export function identity(dim: number): math.Matrix {
  return math.identity(dim) as math.Matrix;
}

// 0行列ラップ
export function zeros(m: number, n?: number): math.Matrix {
  if(n === undefined){
    return math.zeros(m) as math.Matrix;
  }
  return math.zeros(m, n) as math.Matrix;
}

// math.matrix -> Array
export function matrixToArray3(mat: math.Matrix): FArray3 {
  return (math.squeeze(mat).toArray() as number[]).slice(0,3) as FArray3;
}

// 正規化
export function normalize<T extends number | number[] | math.Matrix>(mat: T): T {
  const norm = math.norm(mat);
  return norm < Number.EPSILON ? mat : math.divide(mat, norm) as T;
}

// 行列成分抽出
// T*Rx*Rz*Ry*S
// = | Sx*R00 Sy*R01 Sz*R02 Tx |
//   | Sx*R10 Sy*R11 Sz*R12 Ty |
//   | Sx*R20 Sy*R21 Sz*R22 Tz |
//   |      0      0      0  0 |

/**
 * スケール抽出
 * @param mat 
 */
export function getScale(mat: math.Matrix): FArray3 {
  const tmp = mat.toArray() as number[][];
  return [
    math.norm([tmp[0][0], tmp[1][0], tmp[2][0]]) as number,
    math.norm([tmp[0][1], tmp[1][1], tmp[2][1]]) as number,
    math.norm([tmp[0][2], tmp[1][2], tmp[2][2]]) as number,
  ]
}

/**
 * 回転抽出
 * @param mat 
 */
export function getRotationXYZ(mat: math.Matrix): FArray3 {
  // rotX(x) * rotY(y) * rotZ(z) =
  //  |  CyCz        -CySz         Sy   |
  //  |  SxSyCz+CxSz -SxSySz+CxCz -SxCy |
  //  | -CxSyCz+SxSz  CxSySz+SxCz  CxCy |

  const m = cancelScaling(mat).toArray() as number[][];
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

/**
 * 外積行列 nx
 * @param v 
 */
export function getCrossMatrix(v: FArray3): math.Matrix{
  return math.matrix([
    [   0, -v[2],  v[1]],
    [ v[2],    0, -v[0]],
    [-v[1], v[0],     0],
  ]);
}

/**
 * 軸を交換する行列
 * getSwapMatrix(1,0,2) * [1,0,0] = [0,1,0]
 * @param x 
 * @param y 
 * @param z 
 */
export function getSwapMatrix(x: number, y: number, z: number): math.Matrix{
  return math.matrix([
    [ Number(x==0), Number(y==0), Number(z==0), 0 ],
    [ Number(x==1), Number(y==1), Number(z==1), 0 ],
    [ Number(x==2), Number(y==2), Number(z==2), 0 ],
    [            0,            0,            0, 1 ],
  ]);
}

/**
 * 3x3行列を4x4行列に拡張
 * @param mat 
 */
export function expandToMatrix44(mat: math.Matrix): math.Matrix{
  const ret = math.resize(mat, [4,4]).toArray() as number[][];
  ret[3][3] = 1;
  return math.matrix(ret);
}

/**
 * 回転行列の誤差ベクトル A - B
 * @param matA 
 * @param matB 
 */
export function getRotationError(matA: math.Matrix, matB: math.Matrix): FArray3 {  
  const diff = mul(math.transpose(matB), matA).toArray() as number[][];
  return mul(0.5, [
    diff[2][1] - diff[1][2],
    diff[0][2] - diff[2][0],
    diff[1][0] - diff[0][1]
  ]);
}

/**
 * 転軸回りに回転する行列を計算
 * @param axis 
 * @param rad 
 */
export function getRotationFromAxis(axis: FArray3, rad: number): math.Matrix{
  const norm = math.norm(axis);
  if(Math.abs(rad) < Number.EPSILON ||
     norm < Number.EPSILON){
    return identity(4);
  }
  const normalized_axis = math.divide(axis, norm) as FArray3;
  const crossAxis = getCrossMatrix(normalized_axis);
  // Rodrigues's formula
  // R(θ) = I + nx * sinθ + nx^2 * (1-cosθ)
  return expandToMatrix44(add(math.identity(3), mul(crossAxis, Math.sin(rad)), mul(crossAxis,crossAxis, 1-Math.cos(rad))));
}

/**
 * 回転行列から、x軸に直行する回転軸a と 回転後の軸xまわりの回転量を取り出す
 * y軸を基準にしたい場合はgetSwapMatrix(2,1,0)とする(x,y,z)->(z,x,y)
 * @param rot 
 * @param base 
 * @param swap 
 */
export function getRotationSpherical(rot: math.Matrix, base = identity(4), swap = identity(4)){
  // ベースから見た回転
  let tmp = mul(math.transpose(math.resize(base,[3,3])), math.resize(rot,[3,3]));
  // 軸入れ替え手順(基準ベクトルから見た場合)
  // 1. [1,0,0]を基準にしたい軸になるよう変換。例) [1,0,0](x,y,z) -> [0,1,0](z,x,y)
  // 2. 通常通りの回転を行う。
  // 3. 後の手順でx軸を基準として考えているので戻す。例) z,x,y -> x,y,z
  let m = mul(math.transpose(math.resize(swap,[3,3])), tmp, math.resize(swap,[3,3])).toArray() as number[][];

  // x軸を合わせるための回転軸a
  const ay = -m[2][0] / Math.sqrt(2*(1 + m[0][0]));
  const az = m[1][0] / Math.sqrt(2*(1 + m[0][0]));

  // a軸まわりにγ回転する座標 と baseから見たrot(m) を比較
  const gamma = Math.asin(Math.sqrt(ay*ay+az*az)) * 2;
  const rot_a = math.resize(getRotationFromAxis([0,ay,az], gamma), [3,3]);
  const rot_twist = mul(math.transpose(rot_a), m).toArray() as number[][];

  // y軸がどれだけ回転(x軸回り)したか見る
  const ey = [rot_twist[0][1], rot_twist[1][1], rot_twist[2][1]];
  const twist = Math.atan2(ey[2], ey[1]);
  
  return [ay, az, twist, gamma];
}

/**
 * 平行移動行列
 * @param x 
 * @param y 
 * @param z 
 */
export function translate(x: number,y: number,z: number): math.Matrix {
  return math.matrix([
    [1.0, 0.0, 0.0, x],
    [0.0, 1.0, 0.0, y],
    [0.0, 0.0, 1.0, z],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * X軸平行移動行列
 * @param dLen 
 */
export function translateX(dLen: number): math.Matrix {
  return math.matrix([
    [1.0, 0.0, 0.0, dLen],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * Y軸平行移動行列
 * @param dLen 
 */
export function translateY(dLen: number): math.Matrix {
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * Z軸平行移動行列
 * @param dLen 
 */
export function translateZ(dLen: number): math.Matrix {
  return math.matrix(
    [[1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, dLen],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * 平行移動行列
 * @param axis 
 * @param len 
 */
export function translateAxis(axis: number, len: number): math.Matrix {
  let mat = axis == 0 ? translateX :
            axis == 1 ? translateY :
            axis == 2 ? translateZ : ()=>identity(4);
  return mat(len);
}

/**
 * X軸平行移動行列の偏微分
 */
export function diffTranslateX(): math.Matrix {
  return math.matrix([
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

/**
 * Y軸平行移動行列の偏微分
 */
export function diffTranslateY(): math.Matrix {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

/**
 * Z軸平行移動行列の偏微分
 */
export function diffTranslateZ(): math.Matrix {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

/**
 * 平行移動行列の偏微分
 */
export function diffTranslateAxis(axis: number): math.Matrix{
  let mat = axis == 0 ? diffTranslateX :
            axis == 1 ? diffTranslateY :
            axis == 2 ? diffTranslateZ : ()=>identity(4);
  return mat();
}

/**
 * X軸回りの回転行列
 * @param r 
 */
export function rotX(r: number): math.Matrix {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, s, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * Y軸回りの回転行列
 * @param r 
 */
export function rotY(r: number): math.Matrix {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, 0.0, s, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * Z軸回りの回転行列
 * @param r 
 */
export function rotZ(r: number): math.Matrix {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, -s, 0.0, 0.0],
    [s, c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * XYZ回転行列
 * @param r 
 */
export function rotXYZ(x: number,y: number,z: number): math.Matrix {
  return mul(rotX(x), rotY(y), rotZ(z));
}

/**
 * 軸回りの回転行列
 * @param r 
 */
export function rotAxis(axis: number, angle: number): math.Matrix {
  let mat = axis == 0 ? rotX :
            axis == 1 ? rotY :
            axis == 2 ? rotZ : ()=>identity(4);
  return mat(angle);
}

/**
 * X軸回りの回転行列の偏微分
 * @param r 
 */
export function diffRotX(r: number): math.Matrix  {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -s, -c, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

/**
 * Y軸回りの回転行列の偏微分
 * @param r 
 */
export function diffRotY(r: number): math.Matrix  {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [-c, 0.0, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

/**
 * Z軸回りの回転行列の偏微分
 * @param r 
 */
export function diffRotZ(r: number): math.Matrix  {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, -c, 0.0, 0.0],
    [c, -s, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

/**
 * 軸回りの回転行列の偏微分
 * @param r 
 */
export function diffRotAxis(axis: number, angle: number): math.Matrix{
  let mat = axis == 0 ? diffRotX :
            axis == 1 ? diffRotY :
            axis == 2 ? diffRotZ : ()=>identity(4);
  return mat(angle);
}

/**
 * 拡大縮小行列
 * @param r 
 */
export function scale(...args: number[]): math.Matrix;
export function scale(x: number, y: number, z: number): math.Matrix {
  return math.matrix([
    [  x, 0.0, 0.0, 0.0],
    [0.0,   y, 0.0, 0.0],
    [0.0, 0.0,   z, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

/**
 * 拡大縮小成分をキャンセルする
 * @param mat 
 */
export function cancelScaling(mat: math.Matrix): math.Matrix{
  return mul(mat, scale(...getScale(mat).map(e=> Math.abs(e) < Number.EPSILON ? 1 : 1/e)));
}

/**
 * 軸をベクトルに変換
 * @param axis 
 */
export function axisToVec(axis: number): FArray3 {
  return [Number(axis == 0),Number(axis == 1),Number(axis == 2)];
}
