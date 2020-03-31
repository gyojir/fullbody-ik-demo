
import * as math from "mathjs";
import * as ml from "ml-matrix";

const RAD_TO_DEG = 180.0 / Math.PI;
const DEG_TO_RAD = Math.PI / 180.0;

export const range = num => math.range(0,num).toArray();

const zip = (arr1, arr2) => arr1.map((k, i) => [k, arr2[i]]);

function inv(mat){
  return math.matrix(ml.inverse(new ml.Matrix(mat.toArray()), true).data.map(e=>[].slice.call(e)));
}

function mul(...matrices) {
  return matrices.reduce((prev, curr) => math.multiply(prev, curr));
}

function getOffset(mat){
  return math.squeeze(mat).toArray();
}

function translate(x,y,z) {
  return math.matrix([
    [1.0, 0.0, 0.0, x],
    [0.0, 1.0, 0.0, y],
    [0.0, 0.0, 1.0, z],
    [0.0, 0.0, 0.0, 1.0]])
}

function offsetX(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function offsetY(dLen) {
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function offsetZ(dLen) {
  return math.matrix(
    [[1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, dLen],
    [0.0, 0.0, 0.0, 1.0]])
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

function diffX(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -s, -c, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffY(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [-c, 0.0, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffZ(r) {
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, -c, 0.0, 0.0],
    [c, -s, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

export function _getEffectorMatrix(links, i) {
  let tmp = math.identity(4)

  if(i == -1){
    i = links.length - 1;
  }

  for (let link of links.slice(0, i + 1)) {
    let rot = link.axis == 0 ? rotX :
              link.axis == 1 ? rotY :
              link.axis == 2 ? rotZ : ()=>1;

    tmp = mul(tmp, translate(...link.offset), rot(link.angle));
  }
  return tmp
}

export function getEffectorMatrix(links, i) {
  let tmp = math.identity(4)

  if(i == -1){
    i = links.length - 1;
  }

  while (i != -1) {
    const link = links[i];
    let rot = link.axis == 0 ? rotX :
              link.axis == 1 ? rotY :
              link.axis == 2 ? rotZ : ()=>1;

    tmp = mul(translate(...link.offset), rot(link.angle), tmp);

    i = link.parent;
  }
  return tmp
}

export function getEffectorPosition(links, i) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorMatrix(links, i)
  return math.flatten(tmp.subset(math.index([0, 1, 2], 3)));
}

export function reset_angles(self) {
  for (let link of self.link) {
    link.angle = link.default_angle
  }
}

export function set_angles(links, angles) {
  for (let [link, angle] of zip(links, angles)) {
    link.angle = angle
  }
}

// ヤコビアン計算。偏微分バージョン
export function computeJacobian(links, angles, effector_index) {
  // ヤコビアンはsize×3（関節角度ベクトル長×3次元位置ベクトル長）の行列
  const jac = math.zeros(angles.length, 3).toArray();

  const effectors = [effector_index].map(e => e == -1 ? links.length - 1 : e);
  
  // 各エフェクタに対してヤコビアンを求める
  effectors.forEach((effector, index) => {

    // エフェクタに対する 先端からルートまでの各ジョイント に関する偏微分を求める
    for(let i = effector; i != -1; i = links[i].parent){
      let tmp = math.identity(4);
  
      // ジョイントiに関する偏微分を求める
      for (let j = effector; j != -1; j = links[j].parent) {
        const link = links[j];
        const angle = angles[j];

        // i == j で偏微分
        const rots = 
          link.axis == 0 ? [rotX, diffX] :
          link.axis == 1 ? [rotY, diffY] :
          link.axis == 2 ? [rotZ, diffZ] : [()=>1, ()=>1];
        const rot = i === j ? rots[1] : rots[0]
                  
        tmp = mul(translate(...link.offset), rot(angle), tmp);
      }

      jac[i][index*3 + 0] = tmp.subset(math.index(0, 3));
      jac[i][index*3 + 1] = tmp.subset(math.index(1, 3));
      jac[i][index*3 + 2] = tmp.subset(math.index(2, 3));      
    }
  });

  return math.matrix(jac);
}

// ヤコビアン計算。外積バージョン
export function computeJacobian2(links, angles, effector_index) {
  // ヤコビアンはsize×3（関節角度ベクトル長×3次元位置ベクトル長）の行列
  const jac = math.zeros(angles.length, 3).toArray();

  // エフェクタの位置の取得
  const effectorPos = getEffectorPosition(links, effector_index);

  // エフェクタに対する 先端からルートまでの各ジョイント に関する偏微分を求める
  // => 現在のリンクから先端までのベクトルの回転を考える。（速度ヤコビアン）
  for(let i = effector_index; i != -1; i = links[i].parent){
    const link = links[i];
    const currentPos = getEffectorPosition(links, i)
    const diff = math.subtract(effectorPos, currentPos)
    const mat = getEffectorMatrix(links, i)
    const axis = getOffset(mul(mat, math.matrix([[link.axis == 0],[link.axis == 1],[link.axis == 2],[0]]))).slice(0,3);
    const cross = math.cross(axis, diff).toArray()

    jac[i][0] = cross[0];
    jac[i][1] = cross[1];
    jac[i][2] = cross[2];
  };

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

export function updateAngles(links, _angles, dir, effector_index) {
  let angles = math.clone(_angles);

  // 加重行列を単位行列で初期化
  const weight = math.identity(links.length)
  // ヤコビアンの計算
  const jac = computeJacobian2(links, angles, effector_index)
  // ヤコビアンの擬似逆行列
  const jacPI = computePseudoInverse(jac)
  // ヤコビアンの加重擬似逆行列
  const jacWPI = computeWeightedPseudoInverse(jac, weight)
  // 目標エフェクタ変位×加重擬似逆行列
  angles = math.add(angles, mul(dir, jacWPI).toArray());
  // 冗長変数etaには可動範囲を超えた場合に元に戻すような角変位を与えればいい
  // ↑参考 http://sssiii.seesaa.net/article/383711810.html
  let eta = math.zeros(links.length).toArray() // ゼロクリア
  links.forEach((link, i) => {
    if (angles[i] > link.angle_range[1] * DEG_TO_RAD) { // 最小値より小さければ戻す
      eta[i] = (link.angle_range[1] * DEG_TO_RAD) - angles[i];
    }else if (angles[i] < link.angle_range[0] * DEG_TO_RAD) { // 最大値より大きければ戻す
      eta[i] = (link.angle_range[0] * DEG_TO_RAD) - angles[i];
    }
  });
  eta = mul(eta, 1); // 適当に調整
  // 冗長項の計算   
  const rc = computeRedundantCoefficients(eta, jac, jacPI)
  // 冗長項を関節角度ベクトルに加える
  angles = math.add(angles,math.squeeze(rc).toArray());

  angles = angles.map(e=>e%(Math.PI*2));

  return angles;
}

export function solve_jacobian_ik(links, target, effector_index = -1, max_iteration = 1, step = 0.05) {
  let min_dist = Number.MAX_VALUE;
  let best_angles = links.map(e=>e.angle)
  let angles = math.clone(best_angles)

  for (let i of range(max_iteration)) {
    // リンク構造の全長より遠い位置は到達不可能
    // const length = math.norm(target)
    // if (length >= 60.5) {
    //   target /= length
    //   target *= 60.5
    // }

    // 現在のエフェクタの位置を取得
    const pos = getEffectorPosition(links, effector_index)
    // console.log(pos.toArray())
    // console.log(angles);
    // 目標位置とエフェクタ位置の差分の計算
    let diff = math.subtract(target, pos);
    const dist = math.norm(diff)
    if (dist < min_dist) {
      min_dist = dist
      best_angles = math.clone(angles);
    }

    // 差分が(step / 2)より小さければ計算終了
    if (dist < step / 2) {
      break;
    }

    // 目標エフェクタ変位 = (差分ベクトル / 差分ベクトル長) * step
    diff = math.multiply(diff, step / dist);

    // 目標エフェクタ変位にしたがって関節角度ベクトルを更新
    angles = updateAngles(links, angles, diff, effector_index)
    set_angles(links, angles)
  }

  return best_angles;
}

// console.log(
//   solve_jacobian_ik([
//     { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [1], parent: -1 },
//     { axis: 2, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [2], parent: 0 },
//     { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180], child: [], parent: 1 }],
//     [1, 2, 0]));


// bone, parent-child, current angle, current offset,
// current angle,offset は誰が持つ？
// 