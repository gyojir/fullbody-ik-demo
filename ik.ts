
import * as math from "mathjs";
import {range, zip, rotWrap} from "./util";
import {
  add,
  mul,
  identity,
  zeros,
  inv,
  matrixToArray3,
  normalize,
  translate,
  translateAxis,
  diffTranslateAxis,
  rotXYZ,
  rotAxis,
  diffRotAxis,
  scale,
  axisToVec,
  getRotationSpherical,
  getRotationError,
  getSwapMatrix,
  getRotationXYZ
} from "./math-util";
import { FArray3, Constrain, Joint, JointType, ConstrainType, ConstrainDim } from "./def";

type Diff = number[]

/**
 * ジョイントの姿勢行列取得
 * @param joints 
 * @param bone 
 */
export function getBoneLocalMatrix(joints: Joint[], bone: number): math.Matrix {
  let tmp = identity(4);

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
        joint.type === JointType.Static ? rotXYZ(...joint.rotation || [0,0,0]) : 1,
        scale(...joint.scale));
    });

  return tmp
}

/**
 * ジョイントの姿勢行列取得
 * @param joints 
 * @param i 
 */
export function getJointWorldMatrix(joints: Joint[], i: number): math.Matrix {
  if(i == -1){
    i = joints.length - 1;
  }
  
  const getWorldMatrixRecursive = (index: number): math.Matrix => {
    if(index < 0){
      return identity(4);
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
      joint.type === JointType.Static ? rotXYZ(...joint.rotation || [0,0,0]) : 1,
      scale(...joint.scale));

    joint.world = mul(getWorldMatrixRecursive(joint.parentIndex), local);
    joint.dirty = false;
    return joint.world;
  }

  return getWorldMatrixRecursive(i);
}

/**
 * ジョイントの位置取得
 * @param joints 
 * @param i 
 */
export function getJointWorldPosition(joints: Joint[], i: number): FArray3 {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]
  const tmp = getJointWorldMatrix(joints, i).valueOf() as number[][];
  return [tmp[0][3], tmp[1][3], tmp[2][3]];
}

/**
 * ジョイントの方向取得
 * @param joints 
 * @param i 
 */
export function getJointOrientation(joints: Joint[], i: number): FArray3 {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]
  const tmp = getJointWorldMatrix(joints, i)
  return getRotationXYZ(tmp);
}

/**
 * ジョイントのパラメータ適用
 * @param joints 
 * @param values 
 */
export function setJointValues(joints: Joint[], values: number[]) {
  for (let [joint, value] of zip(joints, values)) {
    joint.value = value;
    joint.dirty = true;
  }
}

/**
 * ヤコビアン計算。偏微分バージョン
 * @param joints 
 * @param values 
 * @param constrains 
 */
export function computeJacobian(joints: Joint[], values: number[], constrains: Constrain[]): math.Matrix {
  // ヤコビアンは（拘束数*次元 × 関節角度ベクトル長）の行列
  const jac = zeros(constrains.reduce((prev,curr)=>prev+ConstrainDim[curr.type], 0), values.length).valueOf() as number[][];
  
  let offset = 0;

  // 各拘束に対してヤコビアンを求める
  constrains.forEach((constrain, index) => {

    // 拘束対象ジョイントに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    for(let i = constrain.joint; i != -1; i = joints[i].parentIndex){
      const joint = joints[i];

      // 位置拘束
      if(constrain.type === ConstrainType.Position){
          let tmp = identity(4);

          // 座標変換をジョイントiで偏微分する
          for (let j = constrain.joint; j != -1; j = joints[j].parentIndex) {
            const tmp_joint = joints[j];            
            const value = values[j];

            const mat =
              // 回転ジョイント
              tmp_joint.type === JointType.Revolution ?
              (i === j ? // i == j で偏微分
                (angle: number)=>diffRotAxis(tmp_joint.axis,angle) :
                (angle: number)=>rotAxis(tmp_joint.axis,angle)) :
              // スライダジョイント
              tmp_joint.type === JointType.Slide ?
              (i === j ? // i == j で偏微分
                (len: number)=>diffTranslateAxis(tmp_joint.axis) :
                (len: number)=>translateAxis(tmp_joint.axis,len))  :
              // スライダジョイント
              tmp_joint.type === JointType.Static ?
                ()=>rotXYZ(...tmp_joint.rotation || [0,0,0]) : () => identity(4);
                      
            tmp = mul(translate(...tmp_joint.offset), mat(value), scale(...tmp_joint.scale), tmp);
          }

          const arr = tmp.valueOf() as number[][];
          jac[offset + 0][i] = arr[0][3];
          jac[offset + 1][i] = arr[1][3];
          jac[offset + 2][i] = arr[2][3];
          
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation ||
              constrain.type === ConstrainType.OrientationBound){
        const mat = getJointWorldMatrix(joints, i)
        const axis = normalize(matrixToArray3(mul(mat, math.matrix([...axisToVec(joint.axis),0]))));

        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          jac[offset + 0][i] = axis[0];
          jac[offset + 1][i] = axis[1];
          jac[offset + 2][i] = axis[2];
        }
      }
      // 参照姿勢拘束
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
  });

  return math.matrix(jac);
}

/**
 * ヤコビアン計算。外積バージョン
 * @param joints 
 * @param values 
 * @param constrains 
 */
export function computeJacobian2(joints: Joint[], values: number[], constrains: Constrain[]): math.Matrix {
  // ヤコビアンは（拘束数*次元 × 関節角度ベクトル長）の行列
  const jac = zeros(constrains.reduce((prev,curr)=>prev+ConstrainDim[curr.type], 0), values.length).valueOf() as number[][];
  
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
          const diff = math.subtract(jointPos, currentPos) as FArray3;
          const cross = math.cross(axis, diff) as FArray3;
    
          jac[offset + 0][i] = cross[0];
          jac[offset + 1][i] = cross[1];
          jac[offset + 2][i] = cross[2];
        }
        // スライダジョイント
        else if(joint.type === JointType.Slide) {
          jac[offset + 0][i] = axis[0];
          jac[offset + 1][i] = axis[1];
          jac[offset + 2][i] = axis[2];
        }
      }
      // 向き拘束
      else if(constrain.type === ConstrainType.Orientation ||
              constrain.type === ConstrainType.OrientationBound){
        // 回転ジョイント
        if(joint.type === JointType.Revolution) {
          jac[offset + 0][i] = axis[0];
          jac[offset + 1][i] = axis[1];
          jac[offset + 2][i] = axis[2];
        }
      }
      // 参照姿勢拘束
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

/**
 * 疑似逆行列計算
 * @param jac 
 */
export function computePseudoInverse(jac: math.Matrix): math.Matrix {
  const jacT = math.transpose(jac);
  // J^T * (J * J^T)^-1
  return mul(jacT, inv(mul(jac, jacT)));
}

/**
 * Singularity-Robust Inverse計算
 * @param jac 
 */
export function computeSRInverse(jac: math.Matrix): math.Matrix {
  const jacT = math.transpose(jac);
  const k = 0.1; // テキトー
  // J^T * (J * J^T + λI)^-1
  return mul(jacT, inv(add(mul(jac, jacT), mul(k, math.identity(jac.size()[0])))));
}

/**
 * 重み付き疑似逆行列計算
 * @param jac 
 * @param weight 
 */
export function computeWeightedPseudoInverse(jac: math.Matrix, weight: math.Matrix): math.Matrix {
  const jacT = math.transpose(jac);
  const WI = inv(math.matrix(weight));
  // W^-1 * J^T * (J * W^-1 * J^T)^-1
  return mul(WI, jacT, inv(mul(jac, WI, jacT)))
}

/**
 * ヤコビアンの一般解の冗長項を計算
 * @param eta 
 * @param jac 
 * @param jacPI 
 */
export function computeRedundantCoefficients(eta: number[], jac: math.Matrix, jacPI: math.Matrix): number[] {
  // vRC = eta - eta * J * J#
  return math.subtract(eta, mul(eta, jac, jacPI)) as number[];
}

/**
 * ヤコビアンを計算して、与えられた関節速度を実現する関節パラメータを計算
 * @param joints 
 * @param _values 
 * @param _diffs 
 * @param _constrains 
 * @param diff_ref 
 */
export function calcJacobianTask(joints: Joint[], _values: number[], _diffs: Diff[], _constrains: Constrain[], diff_ref: number[]) {
  let values = math.clone(_values);

  type Tasks = {
    diffs: Diff[];
    constrains: Constrain[];
  }[];

  // 優先度でdiffとconstrainsを分解
  const prioritized: Tasks = [{diffs: [], constrains: []},{diffs: [], constrains: []}];
  zip(_diffs, _constrains).forEach(([diff,constrain])=>{
    const p = constrain.priority > 0 ? 0 : 1;
    prioritized[p].constrains.push(constrain);
    prioritized[p].diffs.push(diff);
  });

  // 高優先度タスク
  const computeHighPriorityTask = (diffs: Diff[], constrains: Constrain[]): [number[], math.Matrix] =>{
    if(!diffs.length || !constrains.length){
      return [zeros(joints.length).valueOf() as number[], math.identity(joints.length) as math.Matrix];
    }

    // 加重行列を単位行列で初期化
    // const weight = identity(joints.length)     
    // ヤコビアンの計算
    const jac = computeJacobian2(joints, values, constrains)
    // ヤコビアンの擬似逆行列
    const jacPI = computeSRInverse(jac)
    // 目標ジョイント変位×擬似逆行列
    // Δq = J# * Δp
    // Δθ_diff = J# * (Δp - J*Δθref) (バイアス付き最小ノルム解の場合)
    // const dq0 = mul(jacPI, math.flatten(diffs)).valueOf();
    const dq0 = mul(jacPI, math.subtract(math.flatten(diffs), mul(jac, diff_ref))).valueOf() as number[];
    // W = (I-J#J) 冗長項
    const w = math.subtract(math.identity(joints.length), mul(jacPI, jac)) as math.Matrix;

    // console.log(math.flatten(diffs), mul(diff_ref, jac).valueOf())
    return [dq0, w];
  }

  // 低優先度タスク
  const computeLowPriorityTask = (dq0: number[], w: math.Matrix, diffs: Diff[], constrains: Constrain[]): number[] => {
    if(!diffs.length || !constrains.length){
      return dq0;
    }

    // ヤコビアンの計算
    const jac_aux = computeJacobian2(joints, values, constrains);
    // S = Jaux*W
    const s = mul(jac_aux, w);
    // S* = SR Invers of S
    const sSI = computeSRInverse(s);
    // y = (S*)*(paux - Jaux*Δq0)
    const y = mul(sSI, math.subtract(math.flatten(diffs), mul(jac_aux, dq0)));

    // Δq = Δq0 + yW
    return add(dq0, mul(y,w)).valueOf() as number[];
  }
  
  // 優先度順にタスク実行
  const [dq0, w] = computeHighPriorityTask(prioritized[0].diffs, prioritized[0].constrains);
  let dq = computeLowPriorityTask(dq0, w, prioritized[1].diffs, prioritized[1].constrains);

  // 回転ジョイントはバイアス付き解なので戻す
  // Δθ = Δθ_diff + Δθ_ref
  values = add(values, dq, diff_ref);

  // // 冗長変数etaを使って可動範囲を超えた場合に元に戻すように角変位を与える
  // // ↑参考 (三谷純 他 共著，「Computer Graphics Gems JP 2012」，ボーンデジタル，2012)
  // let eta = math.zeros(joints.length).valueOf() // ゼロクリア
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
  // const rc = computeRedundantCoefficients(eta, jac, jacPI).valueOf();
  // 冗長項を関節角度ベクトルに加える
  // values = math.add(values, rc);    

  return values;
}

/**
 * ヤコビアンIK計算
 * @param joints ジョイント配列
 * @param constrains 拘束配列
 * @param max_iteration 最大イテレーション回数
 * @param step ステップサイズ
 * @param ref_diff 参照姿勢速度
 */
export function solveJacobianIk(joints: Joint[], constrains: Constrain[], max_iteration = 1, step = 0.05, ref_diff = zeros(joints.length).valueOf() as number[]): number[] {
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
    const enables: boolean[] = [];
    const diffs: Diff[] = constrains.map((e,i) => {
      enables[i] = true;

      if(!e.enable) {
        enables[i] = false;
        return [0,0,0];
      }

      // 位置拘束は単純に差分
      if(e.type === ConstrainType.Position){
        const p = getJointWorldPosition(joints, e.joint);
        return math.subtract(e.pos || [0,0,0], p) as FArray3;
      }
      // 向き拘束は回転軸を計算
      else if(e.type === ConstrainType.Orientation){
        const curr = getJointWorldMatrix(joints, e.joint);
        const target = rotXYZ(...e.rot || [0,0,0]);
        const err = getRotationError(target, curr);
        return mul(math.resize(curr,[3,3]),err).valueOf() as FArray3;
      }
      // 向き範囲制限拘束
      else if(e.type === ConstrainType.OrientationBound){
        const curr = getBoneLocalMatrix(joints, e.bone);
        const target = rotXYZ(...e.base_rot || [0,0,0]);
        const [ay, az, t, gamma] = getRotationSpherical(curr,target, getSwapMatrix(1,2,0));
        // 範囲を超えた場合のみ戻す
        if(e.bounds && gamma > e.bounds.gamma_max){
          const world = math.resize(getJointWorldMatrix(joints, e.joint),[3,3]);
          const err = getRotationError(target, curr);
          return mul(world, err).valueOf() as FArray3;
        }
      }
      if(e.type === ConstrainType.RefPose){
        return [current_diff_diff[e.joint] * step];
      }
      enables[i] = false;
      return [0,0,0];
    });

    // 差分をまとめる
    const dist_mean = diffs.length > 0 ? math.mean(diffs.map(diff => math.norm(diff) as number)) : 0;
    const ref_mean = math.mean(current_diff_diff.map(e => Math.abs(e)));
    if (dist_mean < min_dist && ref_mean < min_ref_diff) {
      min_dist = dist_mean;
      min_ref_diff = ref_mean;
      best_values = math.clone(values);
    }

    // 差分が(step / 2)より小さければ計算終了
    // 参照姿勢はテキトーに小さくなったら
    if (dist_mean < step / 2 &&
        ref_mean < 0.001) {
      break;
    }

    // 目標ジョイント変位 = (差分ベクトル / 差分ベクトル長) * step
    const step_diffs = diffs.map(e => mul<Diff>(normalize(e), step));
    const step_diff_diff = current_diff_diff.map(e=> e * step);

    // 目標ジョイント変位にしたがって関節角度ベクトルを更新
    // Δθ = J# * Δp
    // θ <- θ + Δθ
    values = calcJacobianTask(joints, values, step_diffs.filter((e,i)=>enables[i]), constrains.filter((e,i)=>enables[i]), step_diff_diff);

    setJointValues(joints, values);
  }

  return best_values;
}