
import * as math from "mathjs";
import * as ml from "ml-matrix";
import {range, zip, rotClamp} from "./util";

export const ConstrainType = {
  Position: 0,
  Orientation: 1,
  OrientationBound: 2,
  RefPose: 3,
};

const ConstrainDim = [
  3,
  3,
  3,
  1
]

export const JointType = {
  Revolution: 0,
  Slide: 1,
  Static: 2,
};


const RAD_TO_DEG = 180.0 / Math.PI;
const DEG_TO_RAD = Math.PI / 180.0;

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

// エフェクタの姿勢行列取得
export function getBoneLocalMatrix(joints, bone) {
  let tmp = math.identity(4)

  if(bone === -1){
    return tmp;
  }

  // 根元から
  joints
    .filter(joint=> joint.boneIndex === bone)
    .forEach(joint=>{
      // parent * T * R * S
      tmp = mul(
        tmp,
        translate(...joint.offset),
        joint.type === JointType.Revolution ? rotAxis(joint.axis, joint.value) :
        joint.type === JointType.Slide ? translateAxis(joint.axis, joint.value) :
        joint.type === JointType.Static ? rotXYZ(...joint.rotation) : 1,
        scale(...joint.scale));
    });

  return tmp
}

// エフェクタの姿勢行列取得
export function getEffectorWorldMatrix(joints, i) {
  if(i == -1){
    i = joints.length - 1;
  }
  
  const getWorldMatrixRecursive = (index) => {
    if(index < 0){
      return math.identity(4);
    }
    
    const joint = joints[index];

    // 計算済み
    if(!joint.dirty){
      return joint.world;      
    }
    
    // parent * T * R * S
    const local = mul(
      translate(...joint.offset),
      joint.type === JointType.Revolution ? rotAxis(joint.axis, joint.value) :
      joint.type === JointType.Slide ? translateAxis(joint.axis, joint.value) : 
      joint.type === JointType.Static ? rotXYZ(...joint.rotation) : 1,
      scale(...joint.scale));

    joint.world = mul(getWorldMatrixRecursive(joint.parentIndex), local);
    joint.dirty = false;
    return joint.world;
  }

  return getWorldMatrixRecursive(i);
}

// エフェクタの位置取得
export function getEffectorWorldPosition(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorWorldMatrix(joints, i)
  return [tmp._data[0][3], tmp._data[1][3], tmp._data[2][3]];
}

// エフェクタの方向取得
export function getEffectorOrientation(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorWorldMatrix(joints, i)
  return getRotationXYZ(tmp);
}

// ジョイントのパラメータ適用
export function setJointValues(joints, values) {
  for (let [joint, value] of zip(joints, values)) {
    joint.value = value;
    joint.dirty = true;
  }
}

// ヤコビアン計算。偏微分バージョン
export function computeJacobian(joints, values, constrains) {
  // ヤコビアンは（関節角度ベクトル長 × 拘束数*3次元）の行列
  const jac = math.zeros(values.length, constrains.length * 3).toArray();
  
  // 各拘束に対してヤコビアンを求める
  constrains.forEach((constrain, index) => {

    // 拘束対象エフェクタに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    for(let i = constrain.joint; i != -1; i = joints[i].parentIndex){
      const joint = joints[i];

      // 位置拘束
      if(constrain.type === ConstrainType.Position){
          let tmp = math.identity(4);

          // 座標変換をジョイントiで偏微分する
          for (let j = constrain.joint; j != -1; j = joints[j].parentIndex) {
            const tmp_joint = joints[j];            
            const value = values[j];

            const mat = tmp_joint.type === JointType.Revolution ?
              // 回転ジョイント
              (i === j ? // i == j で偏微分
                (angle)=>diffRotAxis(tmp_joint.axis,angle) :
                (angle)=>rotAxis(tmp_joint.axis,angle)) :
              // スライダジョイント
              (i === j ? // i == j で偏微分
                (len)=>diffTranslateAxis(tmp_joint.axis) :
                (len)=>translateAxis(tmp_joint.axis,len))
                      
            tmp = mul(translate(...tmp_joint.offset), mat(value), scale(...tmp_joint.scale), tmp);
          }

          jac[i][index*3 + 0] = tmp._data[0][3];
          jac[i][index*3 + 1] = tmp._data[1][3];
          jac[i][index*3 + 2] = tmp._data[2][3];
          
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation ||
              constrain.type === ConstrainType.OrientationBound){
        const mat = getEffectorWorldMatrix(joints, i)
        const axis = normalize(matrixToArray3(mul(mat, math.matrix([...axisToVec(joint.axis),0]))));

        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          jac[i][index*3 + 0] = axis[0];
          jac[i][index*3 + 1] = axis[1];
          jac[i][index*3 + 2] = axis[2];
        }
      }      
    }
  });

  return math.matrix(jac);
}

// ヤコビアン計算。外積バージョン
export function computeJacobian2(joints, values, constrains) {
  // ヤコビアンは（関節角度ベクトル長 × 拘束数*次元）の行列
  const jac = math.zeros(values.length, constrains.reduce((prev,curr)=>prev+ConstrainDim[curr.type], 0)).toArray();
  
  let offset = 0;
  // 各拘束に対してヤコビアンを求める
  constrains.forEach((constrain, index) => {
    // 拘束対象エフェクタの位置の取得
    const effectorPos = getEffectorWorldPosition(joints, constrain.joint);

    // 拘束対象エフェクタに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    // => 現在のリンクから先端までのベクトルの回転を考える。（速度ヤコビアン）
    for(let i = constrain.joint; i != -1; i = joints[i].parentIndex){
      const joint = joints[i];

      const mat = getEffectorWorldMatrix(joints, i);
      const axis = normalize(matrixToArray3(mul(mat, math.matrix([...axisToVec(joint.axis),0]))));

      // 位置拘束
      if(constrain.type === ConstrainType.Position){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          const currentPos = getEffectorWorldPosition(joints, i)
          const diff = math.subtract(effectorPos, currentPos)
          const cross = math.cross(axis, diff)
    
          jac[i][offset + 0] = cross[0];
          jac[i][offset + 1] = cross[1];
          jac[i][offset + 2] = cross[2];
        }
        // スライダジョイント
        else if(joint.type === JointType.Slide) {
          jac[i][offset + 0] = axis[0];
          jac[i][offset + 1] = axis[1];
          jac[i][offset + 2] = axis[2];
        }
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation ||
              constrain.type === ConstrainType.OrientationBound){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          jac[i][offset + 0] = axis[0];
          jac[i][offset + 1] = axis[1];
          jac[i][offset + 2] = axis[2];
        }
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.RefPose){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          if(constrain.joint === i){
            jac[i][offset] = 1;
          }
        }
      }
    }
    offset += ConstrainDim[constrain.type];
  })

  return math.matrix(jac);
}

// 疑似逆行列計算
export function computePseudoInverse(jac) {
  const jacT = math.transpose(jac);
  // (J^T * J)^-1 * J^T
  return math.multiply(inv(math.multiply(jacT, jac)), jacT);
}

// Singularity-Robust Inverse計算
export function computeSRInverse(jac) {
  const jacT = math.transpose(jac);
  const k = 0.9;
  // (J^T * J + λI)^-1 * J^T
  return math.multiply(inv(math.add(math.multiply(jacT, jac),  mul(k, math.identity(jac.size()[1])))), jacT);
}

// 重み付き疑似逆行列計算
export function computeWeightedPseudoInverse(jac, weight) {
  const jacT = math.transpose(jac);
  const WI = inv(math.matrix(weight));
  // (J^T * W^-1 * J)^-1 * J^T * W^-1
  return mul(inv(mul(jacT, WI, jac)), jacT, WI)
}

// ヤコビアンの一般解の冗長項を計算
export function computeRedundantCoefficients(eta, jac, jacPI) {
  // vRC = eta - eta * J * J^+
  return math.subtract(eta, mul(eta, jac, jacPI));
}

// ヤコビアンを計算して、与えられた関節速度を実現する関節パラメータを計算
export function calcJacobianTask(joints, _values, _diffs, _constrains, diff_ref) {
  let values = math.clone(_values);

  // 優先度でdiffとconstrainsを分解
  const prioritized = [{diffs: [], constrains: []},{diffs: [], constrains: []}];
  zip(_diffs, _constrains).forEach(([diff,constrain])=>{
    const p = constrain.priority > 0 ? 0 : 1;
    prioritized[p].constrains.push(constrain);
    prioritized[p].diffs.push(diff);
  });

  // 高優先度タスク
  const computeHighPriorityTask = (diffs, constrains) =>{
    if(!diffs.length || !constrains.length){
      return [math.zeros(joints.length), math.zeros([joints.length,joints.length])];
    }

    // 加重行列を単位行列で初期化
    // const weight = math.identity(joints.length)     
    // ヤコビアンの計算
    const jac = computeJacobian2(joints, values, constrains)
    // ヤコビアンの擬似逆行列
    const jacPI = computeSRInverse(jac)
    // 目標エフェクタ変位×擬似逆行列
    // Δq = Δp * J^+
    // Δθ_diff = (Δp - Δθref*J) * J^+  (バイアス付き最小ノルム解の場合)
    const dq0 = mul(math.flatten(diffs), jacPI).toArray();    
    // W = (I-JJ^+) 冗長項
    const w = math.subtract(math.identity(joints.length), mul(jac,jacPI));

    return [dq0, w];
  }

  // 低優先度タスク
  const computeLowPriorityTask = (dq0, w, diffs, constrains) => {
    if(!diffs.length || !constrains.length){
      return dq0;
    }

    // ヤコビアンの計算
    const jac_aux = computeJacobian2(joints, values, constrains);
    // S = W*Jaux
    const s = mul(w, jac_aux);
    // S* = SR Invers of S
    const sSI = computeSRInverse(s);
    // y = (paux - Δq0*Jaux)*(S*)
    const y = mul(math.subtract(math.flatten(diffs), mul(dq0,jac_aux)), sSI);

    // Δq = Δq0 + yW
    return add(dq0, mul(y,w)).toArray();
  }
  
  // 優先度順にタスク実行
  const [dq0, w] = computeHighPriorityTask(prioritized[0].diffs, prioritized[0].constrains);
  let dq = computeLowPriorityTask(dq0, w, prioritized[1].diffs, prioritized[1].constrains);

  // 回転ジョイントはバイアス付き解なので戻す
  // Δθ = Δθ_diff + Δθ_ref

  values = add(values, dq);

  // // 冗長変数etaを使って可動範囲を超えた場合に元に戻すように角変位を与える
  // // ↑参考 http://sssiii.seesaa.net/article/383711810.html
  // let eta = math.zeros(joints.length).toArray() // ゼロクリア
  // joints.forEach((joint, i) => {
  //   if(joint.type === JointType.Revolution){
  //     if (values[i] > joint.angle_range[1] * DEG_TO_RAD) { // 最小値より小さければ戻す
  //       eta[i] = (joint.angle_range[1] * DEG_TO_RAD) - values[i];
  //     }else if (values[i] < joint.angle_range[0] * DEG_TO_RAD) { // 最大値より大きければ戻す
  //       eta[i] = (joint.angle_range[0] * DEG_TO_RAD) - values[i];
  //     }
  //   }
  // });
  // // 冗長項の計算   
  // const rc = computeRedundantCoefficients(eta, jac, jacPI).toArray();
  // 冗長項を関節角度ベクトルに加える
  // values = math.add(values, rc);    

  return values;
}

// ヤコビアンIK計算
export function solve_jacobian_ik(joints, constrains, ref_diff, max_iteration = 1, step = 0.05) {
  let min_dist = Number.MAX_VALUE;
  let min_ref_diff = Number.MAX_VALUE;
  let best_values = joints.map(e=>e.value);
  let values = math.clone(best_values);
  const before_values = math.clone(best_values);

  const new_constrains = [
    ...constrains,
    ...ref_diff.map((e,i)=> ({priority: 0, joint: i, value: e, type: ConstrainType.RefPose}))  
  ]

  for (let i of range(max_iteration)) {    
    
    // 目標位置と現在エフェクタ位置の差分の計算
    const enables = [];
    let diffs = new_constrains.map((e,i) => {
      enables[i] = true;

      // 位置拘束は単純に差分
      if(e.type === ConstrainType.Position){
        const p = getEffectorWorldPosition(joints, e.joint);
        return math.subtract(e.pos, p);
      }
      // 向き拘束は回転軸を計算
      else if(e.type === ConstrainType.Orientation){
        const curr = getEffectorWorldMatrix(joints, e.joint);
        const target = rotXYZ(...e.rot);
        const err = getRotationError(target, curr);
        return mul(math.resize(curr,[3,3]),err);
      }
      // 向き範囲制限拘束
      else if(e.type === ConstrainType.OrientationBound){
        const curr = getBoneLocalMatrix(joints, e.bone);
        const target = rotXYZ(...e.base_rot);
        const [ay, az, t, gamma] = getRotationSpherical(curr,target, getSwapMatrix(1,2,0));
        // 範囲を超えた場合のみ戻す
        if(gamma > e.bounds.gamma_max){
          const world = math.resize(getEffectorWorldMatrix(joints, e.joint),[3,3]);
          const err = getRotationError(target, curr);
          return mul(world, err);
        }
      }
      if(e.type === ConstrainType.RefPose){
        return e.value - joints[e.joint].value;
      }
      enables[i] = false;
      return [0,0,0];
    });

    // 現在の関節速度
    const current_diff = ref_diff.map((e,i) => joints[i].value - before_values[i]);
    // 現在の関節速度と目標の関節速度の差分
    const current_diff_diff = ref_diff.map((e,i)=> joints[i].type === JointType.Revolution ? e - current_diff[i] : 0);

    // 差分をまとめる
    const dist_mean = math.mean(diffs.map(diff => math.norm(diff)));
    const ref_mean = math.mean(current_diff_diff.filter((e,i)=>joints[i].type === JointType.Revolution).map(e => Math.abs(e)));
    if (dist_mean < min_dist && ref_mean < min_ref_diff) {
      min_dist = dist_mean;
      min_ref_diff = ref_mean;
      best_values = math.clone(values);
    }

    // 差分が(step / 2)より小さければ計算終了
    if (dist_mean < step / 2 &&
        ref_mean < step / 10) {
      break;
    }

    // 目標エフェクタ変位 = (差分ベクトル / 差分ベクトル長) * step
    diffs = diffs.map(e => math.multiply(e, step / dist_mean));

    // 目標エフェクタ変位にしたがって関節角度ベクトルを更新
    // Δθ = Δp * J^+
    // θ <- θ + Δθ
    values = calcJacobianTask(joints, values, diffs.filter((e,i)=>enables[i]), new_constrains.filter((e,i)=>enables[i]), current_diff_diff.map(e=> e * step * 2));

    setJointValues(joints, values);
  }

  return best_values;
}

// console.log(
//   solve_jacobian_ik([
//     { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [1], parentIndex: -1 },
//     { axis: 2, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [2], parentIndex: 0 },
//     { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [], parentIndex: 1 }],
//     [1, 2, 0]));


// bone, parent-child, current angle, current offset,
// current angle,offset は誰が持つ？
// 