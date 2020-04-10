
import * as math from "mathjs";
import * as ml from "ml-matrix";
import {range, zip} from "./util";

export const ConstrainType = {
  Position: 0,
  Orientation: 1,
};

export const JointType = {
  Revolution: 0,
  Slide: 1,
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

export function getTranslate(mat){
  return math.squeeze(mat).toArray();
}

export function getRotationXYZ(mat){
  // rotX(x) * rotY(y) * rotZ(z) =
  //  |  CyCz        -CySz         Sy   |
  //  |  SxSyCz+CxSz -SxSySz+CxCz -SxCy |
  //  | -CxSyCz+SxSz  CxSySz+SxCz  CxCy |

  const m = mat.toArray();
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

export function getRotationRodorigues(axis, rad){
  const norm = math.norm(axis);
  if(Math.abs(rad) < Number.EPSILON ||
     norm < Number.EPSILON){
    return math.identity(4);
  }
  const normalized_axis = math.divide(axis, norm);
  const crossAxis = getCrossMatrix(normalized_axis);
  return expandToMatrix44(add(math.identity(3), mul(crossAxis, Math.sin(rad)), mul(crossAxis,crossAxis, 1-Math.cos(rad))));
}

export function getRotationSpherical(rot, base = math.identity(4)){
  // ベースから見た回転
  const m = mul(math.transpose(base), rot).toArray();

  // x軸を合わせるための回転軸a
  const ay = -m[2][0] / Math.sqrt(2*(1 + m[0][0]));
  const az = m[1][0] / Math.sqrt(2*(1 + m[0][0]));

  // a軸周りにγ回転する座標 と baseから見たrot(m) を比較
  const gamma = Math.asin(Math.sqrt(ay*ay+az*az)) * 2;
  const rot_a = getRotationRodorigues([0,ay,az], gamma);
  const rot_twist = mul(math.transpose(rot_a), m).toArray();

  // y軸がどれだけ回転(x軸回り)したか見る
  const ey = [rot_twist[0][1], rot_twist[1][1], rot_twist[2][1]];
  const twist = Math.atan2(ey[2], ey[1]);
  
  return [ay, az, twist];
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


export function axisToVec(axis){
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

export function computeSRInverse(jac) {
  const jacT = math.transpose(jac);
  const k = 0.9;
  // (J^T * J + λI)^-1 * J^T
  return math.multiply(inv(math.add(math.multiply(jacT, jac),  mul(k, math.identity(jac.size()[1])))), jacT);
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

export function calcNewAnglesAndPositions(joints, _values, _diffs, _constrains) {
  let values = math.clone(_values);

  // 優先度でdiffとconstrainsを分解
  const prioritized_constrains = [[],[]];
  const prioritized_diffs = [[],[]];
  zip(_diffs, _constrains).forEach(([diff,constrain])=>{
    const p = constrain.priority > 0 ? 0 : 1;
    prioritized_constrains[p].push(constrain);
    prioritized_diffs[p].push(diff);
  });

  const computeHighPriorityTask = (diffs, constrains) =>{
    // ヤコビアンの計算
    const jac = computeJacobian2(joints, values, constrains)
    // ヤコビアンの擬似逆行列
    const jacPI = computePseudoInverse(jac)
    // 目標エフェクタ変位×加重擬似逆行列
    // Δq = Δp * J^+
    const dq0 = mul(math.flatten(diffs), jacPI).toArray();    
    // W = (I-JJ^+) 冗長項
    const w = math.subtract(math.identity(joints.length), mul(jac,jacPI));

    return [dq0, w];
  }

  const computeLowPriorityTask = (dq0, w, diffs, constrains) => {        
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
  const [dq0, w] = computeHighPriorityTask(prioritized_diffs[0], prioritized_constrains[0]);
  const dq = computeLowPriorityTask(dq0, w, prioritized_diffs[1], prioritized_constrains[1]);
  values = math.add(values, dq);

  // zip([...prioritized_diffs.entries()], [...prioritized_constrains.entries()]).forEach(([[_,diffs], [__,constrains]])=>{
  //   // 加重行列を単位行列で初期化
  //   const weight = math.identity(joints.length)
  //   // ヤコビアンの計算
  //   const jac = computeJacobian2(joints, values, constrains)
  //   // ヤコビアンの擬似逆行列
  //   const jacPI = computeSRInverse(jac)
  //   // ヤコビアンの加重擬似逆行列
  //   const jacWPI = computeWeightedPseudoInverse(jac, weight)
  //   // 目標エフェクタ変位×加重擬似逆行列
  //   // Δq = Δp * J^+
  //   const dq = mul(math.flatten(diffs), jacWPI).toArray();
  //   values = math.add(values, dq);

  //   // // 冗長変数etaを使って可動範囲を超えた場合に元に戻すように角変位を与える
  //   // // ↑参考 http://sssiii.seesaa.net/article/383711810.html
  //   // let eta = math.zeros(joints.length).toArray() // ゼロクリア
  //   // joints.forEach((joint, i) => {
  //   //   if(joint.type === JointType.Revolution){
  //   //     if (values[i] > joint.angle_range[1] * DEG_TO_RAD) { // 最小値より小さければ戻す
  //   //       eta[i] = (joint.angle_range[1] * DEG_TO_RAD) - values[i];
  //   //     }else if (values[i] < joint.angle_range[0] * DEG_TO_RAD) { // 最大値より大きければ戻す
  //   //       eta[i] = (joint.angle_range[0] * DEG_TO_RAD) - values[i];
  //   //     }
  //   //   }
  //   // });
  //   // // 冗長項の計算   
  //   // const rc = computeRedundantCoefficients(eta, jac, jacPI).toArray();


  //   // 冗長項を関節角度ベクトルに加える
  //   // values = math.add(values, rc);    
  // });


  return values;
}

export function solve_jacobian_ik(joints, constrains, max_iteration = 1, step = 0.05) {
  let min_dist = Number.MAX_VALUE;
  let best_values = joints.map(e=>e.value)
  let values = math.clone(best_values)

  for (let i of range(max_iteration)) {
    // リンク構造の全長より遠い位置は到達不可能
    // const length = math.norm(constrain)
    // if (length >= 60.5) {
    //   constrain /= length
    //   constrain *= 60.5
    // }

    // 目標位置と現在エフェクタ位置の差分の計算
    let diffs = constrains.map(e => {
      if(e.type === ConstrainType.Position){
        const p = getEffectorPosition(joints, e.joint);
        return math.subtract(e.pos, p);
      }
      if(e.type === ConstrainType.Orientation){
        const curr = math.resize(getEffectorMatrix(joints, e.joint), [3,3]);
        const target = math.resize(rotXYZ(...e.pos), [3,3]);
        const err = getRotationError(target, curr);

        return mul(curr,err);
      }
      return [0,0,0];
    });
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