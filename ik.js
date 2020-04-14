
import * as math from "mathjs";
import {range, zip, rotWrap} from "./util";
import {
  add,
  mul,
  inv,
  matrixToArray3,
  normalize,
  translate,
  translateX,
  translateY,
  translateZ,
  translateAxis,
  diffTranslateX,
  diffTranslateY,
  diffTranslateZ,
  diffTranslateAxis,
  rotX,
  rotY,
  rotZ,
  rotXYZ,
  rotAxis,
  diffRotX,
  diffRotY,
  diffRotZ,
  diffRotAxis,
  scale,
  axisToVec,
  cancelScaling,
  getRotationSpherical,
  getRotationError,
  getSwapMatrix,
  getRotationXYZ
} from "./math-util";

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

// ジョイントの姿勢行列取得
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

// ジョイントの姿勢行列取得
export function getJointWorldMatrix(joints, i) {
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

// ジョイントの位置取得
export function getJointWorldPosition(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getJointWorldMatrix(joints, i)
  return [tmp._data[0][3], tmp._data[1][3], tmp._data[2][3]];
}

// ジョイントの方向取得
export function getJointOrientation(joints, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getJointWorldMatrix(joints, i)
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

    // 拘束対象ジョイントに対する 先端からルートまでの各ジョイント に関する偏微分を求める
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
        const mat = getJointWorldMatrix(joints, i)
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
    // 拘束対象ジョイントの位置の取得
    const jointPos = getJointWorldPosition(joints, constrain.joint);

    // 拘束対象ジョイントに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    // => 現在のリンクから先端までのベクトルの回転を考える。（速度ヤコビアン）
    for(let i = constrain.joint; i != -1; i = joints[i].parentIndex){
      const joint = joints[i];

      const mat = getJointWorldMatrix(joints, i);
      const axis = normalize(matrixToArray3(mul(mat, math.matrix([...axisToVec(joint.axis),0]))));

      // 位置拘束
      if(constrain.type === ConstrainType.Position){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          const currentPos = getJointWorldPosition(joints, i)
          const diff = math.subtract(jointPos, currentPos)
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
  const k = 0.1;
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
      return [math.zeros(joints.length).toArray(), math.zeros([joints.length,joints.length])];
    }

    // 加重行列を単位行列で初期化
    // const weight = math.identity(joints.length)     
    // ヤコビアンの計算
    const jac = computeJacobian2(joints, values, constrains)
    // ヤコビアンの擬似逆行列
    const jacPI = computeSRInverse(jac)
    // 目標ジョイント変位×擬似逆行列
    // Δq = Δp * J^+
    // Δθ_diff = (Δp - Δθref*J) * J^+  (バイアス付き最小ノルム解の場合)
    // const dq0 = mul(math.flatten(diffs), jacPI).toArray();
    const dq0 = mul(math.subtract(math.flatten(diffs), mul(diff_ref, jac)), jacPI).toArray();    
    // W = (I-JJ^+) 冗長項
    const w = math.subtract(math.identity(joints.length), mul(jac,jacPI));

    // console.log(math.flatten(diffs), mul(diff_ref, jac).toArray())
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
  values = add(values, dq, diff_ref);

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
export function solve_jacobian_ik(joints, constrains, max_iteration = 1, step = 0.05, ref_diff = math.zeros(joints.length).toArray()) {
  let min_dist = Number.MAX_VALUE;
  let min_ref_diff = Number.MAX_VALUE;
  let best_values = joints.map(e=>e.value);
  let values = math.clone(best_values);
  const before_values = math.clone(best_values);

  // step = 1 / max_iteration;

  for (let i of range(max_iteration)) {    

    // 現在の関節速度
    const current_diff = ref_diff.map((e,i) => joints[i].value - before_values[i]);
    // 現在の関節速度と目標の関節速度の差分
    const current_diff_diff = ref_diff.map((e,i) =>
      joints[i].type === JointType.Revolution ||
      joints[i].type === JointType.Slide ? e - current_diff[i] : 0);
    
    // 目標位置と現在ジョイント位置の差分の計算
    const enables = [];
    let diffs = constrains.map((e,i) => {
      enables[i] = true;

      if(!e.enable) {
        enables[i] = false;
        return [0,0,0];
      }

      // 位置拘束は単純に差分
      if(e.type === ConstrainType.Position){
        const p = getJointWorldPosition(joints, e.joint);
        return math.subtract(e.pos, p);
      }
      // 向き拘束は回転軸を計算
      else if(e.type === ConstrainType.Orientation){
        const curr = getJointWorldMatrix(joints, e.joint);
        const target = rotXYZ(...e.rot);
        const err = getRotationError(target, curr);
        return mul(math.resize(curr,[3,3]),err).toArray();
      }
      // 向き範囲制限拘束
      else if(e.type === ConstrainType.OrientationBound){
        const curr = getBoneLocalMatrix(joints, e.bone);
        const target = rotXYZ(...e.base_rot);
        const [ay, az, t, gamma] = getRotationSpherical(curr,target, getSwapMatrix(1,2,0));
        // 範囲を超えた場合のみ戻す
        if(gamma > e.bounds.gamma_max){
          const world = math.resize(getJointWorldMatrix(joints, e.joint),[3,3]);
          const err = getRotationError(target, curr);
          return mul(world, err).toArray();
        }
      }
      if(e.type === ConstrainType.RefPose){
        return current_diff_diff[e.joint] * step;
      }
      enables[i] = false;
      return [0,0,0];
    });

    // 差分をまとめる
    const dist_mean = diffs.length > 0 ? math.mean(diffs.map(diff => math.norm(diff))) : 0;
    const ref_mean = math.mean(current_diff_diff.map(e => Math.abs(e)));
    if (dist_mean < min_dist && ref_mean < min_ref_diff) {
      min_dist = dist_mean;
      min_ref_diff = ref_mean;
      best_values = math.clone(values);
    }

    // 差分が(step / 2)より小さければ計算終了
    if (dist_mean < step / 2 &&
        ref_mean < 0.001) {
      break;
    }

    // 目標ジョイント変位 = (差分ベクトル / 差分ベクトル長) * step
    diffs = diffs.map(e => math.multiply(normalize(e), step));

    // 目標ジョイント変位にしたがって関節角度ベクトルを更新
    // Δθ = Δp * J^+
    // θ <- θ + Δθ
    values = calcJacobianTask(joints, values, diffs.filter((e,i)=>enables[i]), constrains.filter((e,i)=>enables[i]), current_diff_diff.map(e=> e * step));
    // values = add(values, current_diff_diff.map(e=> e * step));

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