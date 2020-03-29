
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
    [0.0, 1.0, 0.0, dLen],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotX(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, s, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotY(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, 0.0, s, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function rotZ(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [c, -s, 0.0, 0.0],
    [s, c, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]])
}

function diffX(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -s, -c, 0.0],
    [0.0, c, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffY(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, 0.0, c, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [-c, 0.0, -s, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

function diffZ(deg) {
  const r = deg * DEG_TO_RAD
  const c = math.cos(r)
  const s = math.sin(r)
  return math.matrix([
    [-s, -c, 0.0, 0.0],
    [c, -s, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]])
}

export function getEffectorMatrix(links, i = -1) {
  let tmp = math.identity(4)
  for (let link of links.slice(0, i)) {
    if (link.axis == 0) // X
      tmp = math.multiply(tmp, rotX(link.angle))
    if (link.axis == 1) // Y
      tmp = math.multiply(tmp, rotY(link.angle))
    if (link.axis == 2) // Z
      tmp = math.multiply(tmp, rotZ(link.angle))
    // 基本的にオフセットはY方向なので計算しない
    tmp = mul(
      tmp,
      offsetY(link.offset[1]),
      offsetX(link.offset[0]),
      offsetZ(link.offset[2])
    );
  }
  return tmp
}

export function getEffectorPosition(links, i = -1) {
  // 先端の座標系から根元の座標系に変換
  // Max0 * Max1 * Mat2 * Mat3 * [0,0,0,1]

  const tmp = getEffectorMatrix(links, i)
  return math.flatten(tmp.subset(math.index([0, 1, 2], 3)));
}

export function getGlobalCoordinate(self, index) {
  tmp = math.identity(4)
  for (let i of range(index)) {
    link = self.link[i]
    if (link.axis == 0) // X
      tmp = math.multiply(tmp, rotX(link.angle))
    if (link.axis == 1) // Y
      tmp = math.multiply(tmp, rotY(link.angle))
    if (link.axis == 2) // Z
      tmp = math.multiply(tmp, rotZ(link.angle))
    // 基本的にオフセットはY方向なので計算しない
    tmp = mul(
      tmp,
      offsetY(link.offset[1]),
      offsetX(link.offset[0]),
      offsetZ(link.offset[2]));
  }

  return tmp
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

export function solve_ccd(self, _angles, target, max_iteration = 5) {
  angles = math.clone(_angles)
  self.set_angles(angles)

  for (let i of range(max_iteration)) {
    for (let j of range(len(self.link)).reverse()) { // 反転
      link = self.link[j]

      // エフェクタの位置の取得
      effectorPos = self.getEffectorPosition()

      // ワールド座標系から注目ノードの局所座標系への変換
      invCoord = math.inv(self.getGlobalCoordinate(j))
      // (1) 基準関節→エフェクタ位置への方向ベクトル(エフェクタのローカル位置)
      localEffectorPos = invCoord * math.matrix(math.append(effectorPos, 1.0)).T // 4x4行列なので4次元ベクトルに拡張
      // (2) 基準関節→目標位置への方向ベクトル(到達目標のローカル位置)
      localTargetPos = invCoord * math.matrix(math.append(target, 1.0)).T

      // math.matrixからmath.arrayに
      basis2Effector = math.squeeze(math.asarray(localEffectorPos.T))
      basis2Target = math.squeeze(math.asarray(localTargetPos.T))

      // 4次元目と、回転軸を削除し、2次元に
      basis2Effector = math.delete(basis2Effector, [3, link.axis])
      basis2Target = math.delete(basis2Target, [3, link.axis])

      // Y のとき軸を入れ替え
      if (link.axis == 1) {
        basis2Effector = basis2Effector.reverse()
        basis2Target = basis2Target.reverse()
      }

      // 正規化
      basis2Effector /= math.norm(basis2Effector)
      basis2Target /= math.norm(basis2Target)

      // 回転角
      // ベクトル (1) を (2) に一致させるための最短回転量（Axis-Angle）
      dot = math.dot(basis2Effector, basis2Target)
      rot = math.arccos(dot)
      // 回転が無いときと、回転が180度あるときはやめる
      if (rot > 1.0e-5 && !math.abs(rot - math.pi) < 1.0e-4) {
        // 回転軸
        cross = math.cross(basis2Effector, basis2Target)
        rot = math.degrees(math.arctan2(cross, dot))
        link.setAngle(link.angle + rot)
        angles[j] = link.angle
      }
    }

    if (math.norm(localEffectorPos - localTargetPos) < 1.0e-4) {
      return angles
    }
  }
  return angles
}

export function computeJacobian(links, angles) {
  // ヤコビアンはsize×3（関節角度ベクトル長×3次元位置ベクトル長）の行列
  const jac = math.zeros(angles.length, 3).toArray();
  let back = math.identity(4)

  zip(links, angles).forEach(([link, angle], i) => {
    let tmp = back.clone();
    for (let [forward_link, forward_angle] of zip(links, angles)) {
      // 自分は偏微分
      if (link === forward_link) {
        if (link.axis == 0) { // X
          tmp = math.multiply(tmp, diffX(forward_angle))
          back = math.multiply(back, rotX(forward_angle))
        }
        if (link.axis == 1) { // Y
          tmp = math.multiply(tmp, diffY(forward_angle))
          back = math.multiply(back, rotY(forward_angle))
        }
        if (link.axis == 2) { // Z
          tmp = math.multiply(tmp, diffZ(forward_angle))
          back = math.multiply(back, rotZ(forward_angle))
        }
        back = math.multiply(back, offsetY(link.offset[1]))
      }
      else {
        if (forward_link.axis == 0) // X
          tmp = math.multiply(tmp, rotX(forward_angle))
        if (forward_link.axis == 1) // Y
          tmp = math.multiply(tmp, rotY(forward_angle))
        if (forward_link.axis == 2) // Z
          tmp = math.multiply(tmp, rotZ(forward_angle))
      }

      // 基本的にオフセットはY方向なので計算しない
      tmp = math.multiply(tmp, offsetY(forward_link.offset[1]))
      //import pdb; pdb.set_trace()
    }

    jac[i][0] = tmp.subset(math.index(0, 3));
    jac[i][1] = tmp.subset(math.index(1, 3));
    jac[i][2] = tmp.subset(math.index(2, 3));

    //print tmp
    //import pdb; pdb.set_trace()
  });

  return math.matrix(jac);
}

export function computeJacobian2(links, angles) {
  // ヤコビアンはsize×3（関節角度ベクトル長×3次元位置ベクトル長）の行列
  const jac = math.zeros(angles.length, 3).toArray();

  // エフェクタの位置の取得
  const effectorPos = getEffectorPosition(links);

  zip(links, angles).forEach(([link, angle], i) => {
    // 現在のリンクから先端までのベクトルの回転を考える。（速度ヤコビアン）
    const currentPos = getEffectorPosition(links, i)
    const diff = math.subtract(effectorPos, currentPos)
    const mat = getEffectorMatrix(links, i)
    const axis = getOffset(mul(mat, math.matrix([[link.axis == 0],[link.axis == 1],[link.axis == 2],[0]]))).slice(0,3);
    const cross = math.cross(axis, diff).toArray()

    jac[i][0] = cross[0];
    jac[i][1] = cross[1];
    jac[i][2] = cross[2];
  });

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

export function updateAngles(links, _angles, dir, step) {
  let angles = math.clone(_angles);

  // 加重行列を単位行列で初期化
  const weight = math.identity(links.length)
  // ヤコビアンの計算
  const jac = computeJacobian2(links, angles)
  // ヤコビアンの擬似逆行列
  const jacPI = computePseudoInverse(jac)
  // ヤコビアンの加重擬似逆行列
  const jacWPI = computeWeightedPseudoInverse(jac, weight)
  // 目標エフェクタ変位×加重擬似逆行列
  angles = math.add(angles, mul(dir, jacWPI, RAD_TO_DEG).toArray());
  // 冗長変数etaには可動範囲を超えた場合に元に戻すような角変位を与えればいい
  // ↑参考 http://sssiii.seesaa.net/article/383711810.html
  let eta = math.zeros(links.length) // ゼロクリア
  links.forEach((link, i) => {
    if (angles[i] > link.angle_range[1]) // 最小値より小さければ戻す
      eta[i] = link.angle_range[0] - angles[i]
    if (angles[i] < link.angle_range[0]) // 最大値より大きければ戻す
      eta[i] = link.angle_range[0] - angles[i]
  });
  eta = math.multiply(eta, DEG_TO_RAD);
  // 冗長項の計算   
  const rc = computeRedundantCoefficients(eta, jac, jacPI)
  // 冗長項を関節角度ベクトルに加える
  angles = math.add(angles, math.multiply(math.squeeze(rc), RAD_TO_DEG).toArray());

  return angles;
}

export function solve_jacobian_ik(links, target, max_iteration = 5, step = 0.5) {
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
    const pos = getEffectorPosition(links)
    // 目標位置とエフェクタ位置の差分の計算
    let diff = math.subtract(target, pos);
    // 差分が(step / 2)より小さければ計算終了
    const dist = math.norm(diff)
    if (dist < min_dist) {
      min_dist = dist
      best_angles = math.clone(angles);
    }

    //if (dist < step / 2)
    if (dist < 1.0) {
      return best_angles
    }

    // 目標エフェクタ変位 = (差分ベクトル / 差分ベクトル長) * step
    diff = math.multiply(diff, step / dist);

    // 目標エフェクタ変位にしたがって関節角度ベクトルを更新
    angles = updateAngles(links, angles, diff, step)
    set_angles(links, angles)
  }
}

console.log(
  solve_jacobian_ik([
    { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180] },
    { axis: 2, angle: 0, offset: [0, 1, 0], angle_range: [0, 180] },
    { axis: 0, angle: 0, offset: [0, 1, 0], angle_range: [0, 180] }],
    [1, 2, 0]));
