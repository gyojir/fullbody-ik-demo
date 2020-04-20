
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';

export type FArray3 = [number, number, number];
export type FArray4 = [number, number, number, number];

export type Bone = {
  offset: FArray3;
  rotation: FArray3;
  scale: FArray3;
  static?: boolean;
  slide?: boolean;
  object?: THREE.Object3D;
  animation_object?: THREE.Object3D;
  parentIndex?: number;
  children: Bone[];
}

export type Joint = {
  boneIndex: number;
  type: JointType;
  axis: number;
  value: number;
  offset: FArray3;
  scale: FArray3;
  rotation: FArray3;
  parentIndex: number;
  dirty: boolean;
  world: math.Matrix; 
};

export type Constrain = {
  priority: number;
  bone: number;
  joint: number;
  pos?: FArray3;
  rot?: FArray3;
  base_rot?: FArray3;
  bounds?: {
    gamma_max: number;
  };
  object?: THREE.Object3D;
  control?: TransformControls;
  type: any;
  enable: boolean;
}

export enum ConstrainType{
  Position,
  Orientation,
  OrientationBound,
  RefPose
};

export const ConstrainDim = [
  3,
  3,
  3,
  1
]

export enum JointType {
  Revolution,
  Slide,
  Static,
};