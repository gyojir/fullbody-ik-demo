
import * as math from "mathjs";
import * as ml from "ml-matrix";
import {range, zip} from "./util";

export const ConstrainType = {
  Position: 0,
  Orientation: 1,
}

export const JointType = {
  Revolution: 0,
  Slide: 1,
}


const RAD_TO_DEG = 180.0 / Math.PI;
const DEG_TO_RAD = Math.PI / 180.0;

function inv(mat){
  return math.matrix(ml.inverse(new ml.Matrix(mat.toArray()), true).data.map(e=>[].slice.call(e)));
}

function mul(...matrices) {
  return matrices.reduce((prev, curr) => math.multiply(prev, curr));
}

function getTranslate(mat){
  return math.squeeze(mat).toArray();
}

function getRotationXYZ(mat){
  const m = mat.toArray();
  
  const sq_m12_m22 = Math.sqrt(Math.pow(m[1][2],2) + Math.pow(m[2][2],2));
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

function translate(x,y,z) {
  return math.matrix([
    [1.0, 0.0, 0.0, x],
    [0.0, 1.0, 0.0, y],
    [0.0, 0.0, 1.0, z],
    [0.0, 0.0, 0.0, 1.0]])
}

function translateX(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, dLen],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function translateY(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function translateZ(dLen) {
  return math.matrix(
    [[1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, dLen],
    [0.0, 0.0, 0.0, 1.0]])
}

function translateAxis(axis, len){
  let mat = axis == 0 ? translateX :
            axis == 1 ? translateY :
            axis == 2 ? translateZ : ()=>1;
  return mat(len);
}

function diffTranslateX() {
  return math.matrix([
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

function diffTranslateY() {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

function diffTranslateZ() {
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0]]);
}

function diffTranslateAxis(axis){
  let mat = axis == 0 ? diffTranslateX :
            axis == 1 ? diffTranslateY :
            axis == 2 ? diffTranslateZ : ()=>1;
  return mat();
}

function rotX(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, s, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotY(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, 0.0, s, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotZ(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, -s, 0.0, 0.0],
    [s, c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotAxis(axis, angle){
  let mat = axis == 0 ? rotX :
            axis == 1 ? rotY :
            axis == 2 ? rotZ : ()=>1;
  return mat(angle);
}

function diffRotX(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -s, -c, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffRotY(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [-c, 0.0, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffRotZ(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, -c, 0.0, 0.0],
    [c, -s, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffRotAxis(axis, angle){
  let mat = axis == 0 ? diffRotX :
            axis == 1 ? diffRotY :
            axis == 2 ? diffRotZ : ()=>1;
  return mat(angle);
}


function axisToVec(axis){
  return [Number(axis == 0),Number(axis == 1),Number(axis == 2)];
}

export function _getEffectorMatrix(joints, i) {
  let tmp = math.identity(4)

  if(i == -1){
    i = joints.length - 1;
  }

  for (let joint of joints.slice(0, i + 1)) {
    tmp = mul(tmp, translate(...joint.offset));

　　if(joint.type === JointType.Revolution){  
      tmp = mul(tmp, rotAxis(joint.axis, joint.value));
    }else{
      tmp = mul(tmp, translateAxis(joint.axis, joint.value));
    }
  }
  return tmp
}

export function getEffectorMatrix(joints, i) {
  let tmp = math.identity(4)

  if(i == -1){
    i = joints.length - 1;
  }

  while (i != -1) {
    const joint = joints[i];
    
　　if(joint.type === JointType.Revolution){  
      tmp = mul(rotAxis(joint.axis, joint.value), tmp);
    }else{
      tmp = mul(translateAxis(joint.axis, joint.value), tmp);
    }
    tmp = mul(translate(...joint.offset), tmp);

    i = joint.parentIndex;
  }
  return tmp
}

export function getEffectorPosition(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorMatrix(joints, i)
  return math.flatten(tmp.subset(math.index([0, 1, 2], 3)));
}

export function getEffectorOrientation(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorMatrix(joints, i)
  return getRotationXYZ(tmp);
}

export function setJointValues(joints, values) {
  for (let [joint, value] of zip(joints, values)) {
    joint.value = value
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

          // ジョイントiで偏微分する
          for (let j = constrain.joint; j != -1; j = joints[j].parentIndex) {
            const tmp_joint = joints[j];
            
            const value = values[j];

            const mat = joint.type === JointType.Revolution ?
              // 回転ジョイント
              (i === j ? // i == j で偏微分
                (angle)=>diffRotAxis(tmp_joint.axis,angle) :
                (angle)=>rotAxis(tmp_joint.axis,angle)) :
              // スライダジョイント
              (i === j ? // i == j で偏微分
                (len)=>diffTranslateAxis(tmp_joint.axis) :
                (len)=>translateAxis(tmp_joint.axis,len))
                      
            tmp = mul(translate(...tmp_joint.offset), mat(value), tmp);
          }

          jac[i][index*3 + 0] = tmp.subset(math.index(0, 3));
          jac[i][index*3 + 1] = tmp.subset(math.index(1, 3));
          jac[i][index*3 + 2] = tmp.subset(math.index(2, 3));
          
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation){
        const mat = getEffectorMatrix(joints, i)
        const axis = getTranslate(mul(mat, math.matrix([...axisToVec(joint.axis),0]))).slice(0,3);

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
  // ヤコビアンは（関節角度ベクトル長 × 拘束数*3次元）の行列
  const jac = math.zeros(values.length, constrains.length * 3).toArray();
  
  // 各拘束に対してヤコビアンを求める
  constrains.forEach((constrain, index) => {
    // 拘束対象エフェクタの位置の取得
    const effectorPos = getEffectorPosition(joints, constrain.joint);

    // 拘束対象エフェクタに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    // => 現在のリンクから先端までのベクトルの回転を考える。（速度ヤコビアン）
    for(let i = constrain.joint; i != -1; i = joints[i].parentIndex){
      const joint = joints[i];

      const mat = getEffectorMatrix(joints, i)
      const axis = getTranslate(mul(mat, math.matrix([...axisToVec(joint.axis),0]))).slice(0,3);

      // 位置拘束
      if(constrain.type === ConstrainType.Position){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          const currentPos = getEffectorPosition(joints, i)
          const diff = math.subtract(effectorPos, currentPos)
          const cross = math.cross(axis, diff).toArray()
    
          jac[i][index*3 + 0] = cross[0];
          jac[i][index*3 + 1] = cross[1];
          jac[i][index*3 + 2] = cross[2];
        }
        // スライダジョイント
        else {
          jac[i][index*3 + 0] = axis[0];
          jac[i][index*3 + 1] = axis[1];
          jac[i][index*3 + 2] = axis[2];
        }
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          jac[i][index*3 + 0] = axis[0];
          jac[i][index*3 + 1] = axis[1];
          jac[i][index*3 + 2] = axis[2];
        }
      }
    };
  })

  return math.matrix(jac);
}

export function computePseudoInverse(jac) {
  const jacT = math.transpose(jac);
  // (J^T * J)^-1 * J^T
  return math.multiply(inv(math.multiply(jacT, jac)), jacT);
}

export function computeWeightedPseudoInverse(jac, weight) {
  // inverse(W)
  const WI = inv(math.matrix(weight))
  // J^T * W
  const JtWi = math.multiply(math.transpose(jac), WI);
  // J^T * W^-1 * J
  const JtWiJ = math.multiply(JtWi, jac);
  // (J^T * W^-1 * J)^-1
  const JtWiJi = inv(JtWiJ)
  // (J^T * W^-1 * J)^-1 * J^T
  const JtWiJiJt = math.multiply(JtWiJi, math.transpose(jac))
  // (J^T * W^-1 * J)^-1 * J^T * W^-1
  const WPI = math.multiply(JtWiJiJt, WI)
  return WPI
}

export function computeRedundantCoefficients(eta, jac, jacPI) {
  // vRC = eta - eta * J * J^+
  return math.subtract(eta, mul(eta, jac, jacPI));
}

export function calcNewAnglesAndPositions(joints, _values, diffs, constrains) {
  let values = math.clone(_values);

  // 加重行列を単位行列で初期化
  const weight = math.identity(joints.length)
  // ヤコビアンの計算
  const jac = computeJacobian(joints, values, constrains)
  // ヤコビアンの擬似逆行列
  const jacPI = computePseudoInverse(jac)
  // ヤコビアンの加重擬似逆行列
  const jacWPI = computeWeightedPseudoInverse(jac, weight)
  // 目標エフェクタ変位×加重擬似逆行列
  // Δq = Δp * J^+
  const dq = mul(math.flatten(diffs), jacWPI).toArray();
  values = math.add(values, dq);
  // 冗長変数etaを使って可動範囲を超えた場合に元に戻すように角変位を与える
  // ↑参考 http://sssiii.seesaa.net/article/383711810.html
  let eta = math.zeros(joints.length).toArray() // ゼロクリア
  joints.forEach((joint, i) => {
    if(joint.type === JointType.Revolution){
      if (values[i] > joint.angle_range[1] * DEG_TO_RAD) { // 最小値より小さければ戻す
        eta[i] = (joint.angle_range[1] * DEG_TO_RAD) - values[i];
      }else if (values[i] < joint.angle_range[0] * DEG_TO_RAD) { // 最大値より大きければ戻す
        eta[i] = (joint.angle_range[0] * DEG_TO_RAD) - values[i];
      }
    }
  });
  // 冗長項の計算   
  const rc = computeRedundantCoefficients(eta, jac, jacPI).toArray();
  // 冗長項を関節角度ベクトルに加える
  values = math.add(values, rc);

  return values;
}

export function solve_jacobian_ik(joints, constrains, max_iteration = 1, step = 0.05) {
  let min_dist = Number.MAX_VALUE;
  let best_values = joints.map(e=>e.value)
  let values = math.clone(best_values)

  const constrain_values = constrains.map(e => e.pos);

  for (let i of range(max_iteration)) {
    // リンク構造の全長より遠い位置は到達不可能
    // const length = math.norm(constrain)
    // if (length >= 60.5) {
    //   constrain /= length
    //   constrain *= 60.5
    // }

    // 現在のエフェクタの位置を取得
    const effector_values = constrains.map(e =>
      e.type === ConstrainType.Position ?
        getEffectorPosition(joints, e.joint) : 
        getEffectorOrientation(joints, e.joint));

    // 目標位置とエフェクタ位置の差分の計算
    let diffs = zip(effector_values,constrain_values).map(([p, t]) => math.subtract(t, p));
    const dist_mean = math.mean(diffs.map(diff => math.norm(diff)));
    if (dist_mean < min_dist) {
      min_dist = dist_mean
      best_values = math.clone(values);
    }
    // 差分が(step / 2)より小さければ計算終了
    if (dist_mean < step / 2) {
      break;
    }

    // 目標エフェクタ変位 = (差分ベクトル / 差分ベクトル長) * step
    diffs = math.multiply(diffs, step / dist_mean);

    // 目標エフェクタ変位にしたがって関節角度ベクトルを更新
    const new_value = calcNewAnglesAndPositions(joints, values, diffs, constrains)
    values = new_value;
    setJointValues(joints, values)
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
