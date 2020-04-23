import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonUtils } from 'three/examples/jsm/utils/SkeletonUtils.js';
import * as math from "mathjs";
import { getJointOrientation, getJointWorldMatrix, solveJacobianIk, getJointWorldPosition } from './ik';
import { mul, rotXYZ, getRotationXYZ, identity, cancelScaling, cancelTranslate }from './math-util';
import { existFilter, rotWrap, SliderAngleFloat3, initImGui, endImGui, beginImGui } from "./util";
import { Joint, ConstrainType, JointType, FArray3, Bone, Constrain, ConstrainName, Priority, PriorityName } from './def';
import * as ImGui from 'imgui-js/imgui.js';

const modelfile = require('../models/Soldier.glb');

let camera : THREE.PerspectiveCamera | undefined;
let scene : THREE.Scene | undefined;
let renderer : THREE.WebGLRenderer | undefined;
let model : THREE.Object3D | undefined;
let animation_compute_model : THREE.Object3D | undefined;
let skeleton : THREE.SkeletonHelper | undefined;
let animation_compute_skeleton : THREE.SkeletonHelper | undefined;
let mixer : THREE.AnimationMixer | undefined;
var actions : THREE.AnimationAction[] = [];
let clock : THREE.Clock | undefined;

const canvas = document.getElementById("canvas") as HTMLCanvasElement;

const settings = {
  animation: true,
  debugDisp: false,
};

const bones : Bone[] = [];
const constrains: Constrain[] = [
  {priority: Priority.High, bone: 15, joint: -1, pos: [0.5,1.5,0], object: undefined, type: ConstrainType.Position, enable: true},
  {priority: Priority.High, bone: 15, joint: -1, rot: [-0.5,-0.2,-0.2], object: undefined, type: ConstrainType.Orientation, enable: true},
  {priority: Priority.High, bone: 11, joint: -1, pos: [-0.5,1.5,0], object: undefined, type: ConstrainType.Position, enable: true},
  {priority: Priority.Low, bone: 6, joint: -1, base_rot: [0,0,0], bounds: { gamma_max: Math.PI/4}, object: undefined, type: ConstrainType.OrientationBound, enable: true},
];

/*
const constrains: Constrain[] = [
  {priority: 1, bone: 2, joint: -1, pos: [1,1,0], object: undefined, type: ConstrainType.Position, enable: true},
  {priority: 0, bone: 2, joint: -1, rot: [1,0,0], object: undefined, type: ConstrainType.Orientation, enable: true},
  {priority: 0, bone: 2, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: undefined, type: ConstrainType.OrientationBound, enable: true},
  {priority: 0, bone: 0, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: undefined, type: ConstrainType.OrientationBound, enable: true},
  {priority: 1, bone: 4, joint: -1, pos: [-1,1,0], object: undefined, type: ConstrainType.Position, enable: true}
];

const bones: Bone[] = [];
const root: Bone = {
  offset: [0,0,0],
  rotation: [0,0,0],
  scale: [1,1,1],
  slide: false, // ik計算用スライドジョイントフラグ
  parentIndex: -1,
  children: [{
      offset: [0.5,1,0],
      rotation: [0,0,0],
      scale: [1,1,1],
      parentIndex: -1,
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        parentIndex: -1,
        children: []
      }]
    },
    {
      offset: [-0.5,1,0],
      rotation: [0,0,0],
      scale: [1,1,1],
      parentIndex: -1,
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        parentIndex: -1,
        children: []
      }]
    },
    {
      offset: [0.5,0,0],
      rotation: [Math.PI,0,0],
      scale: [1,1,1],
      parentIndex: -1,
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        parentIndex: -1,
        children: []
      }]
    },
    {
      offset: [-0.5,0,0],
      rotation: [Math.PI,0,0],
      scale: [1,1,1],
      parentIndex: -1,
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        parentIndex: -1,
        children: []
      }]
    }
  ]
}
*/

/**
 * ボーンに対応するジョイントを取得
 * @param joints ジョイント
 * @param boneIndex 取得したいボーン
 * @returns ジョイントインデックス
 */
function convertBoneToJointIndex(joints: Joint[], boneIndex: number): number {
  // ボーンに対応する一番最後のジョイントを選ぶ
  for(const [index, joint] of [...joints.entries()].reverse()) {
    if(joint.boneIndex === boneIndex){
      return index;
    }
  }
  return -1;
}

/**
 * 回転と移動を分解してジョイントに変換
 * @param bones ボーン
 * @returns ジョイント
 */
function convertBonesToJoints(bones: Bone[]): Joint[] {
  const joints: Joint[] = [];
  const indices: number[] = [];
  bones.forEach((bone,i)=>{
    let parent = (()=> { let j = 0; return ()=> j++ === 0 && indices[bone.parentIndex] !== undefined ? indices[bone.parentIndex] : joints.length - 1; })();

    if(bone.static) {
      joints.push({ boneIndex: i, type: JointType.Static, axis: 0, value: 0, offset: bone.offset, scale: bone.scale, rotation: bone.rotation, parentIndex: parent(), dirty: true, world: identity(4) });      
    }
    else {
      // value = 関節変位 q
      // スライダジョイントを挿入
      if(bone.slide){
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 0, value: bone.offset[0], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: identity(4) });
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 1, value: bone.offset[1], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: identity(4) });
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 2, value: bone.offset[2], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: identity(4) });
      }
  
      // XYZ回転
      let offset: FArray3 = bone.slide ? [0,0,0] : bone.offset;
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 0, value: bone.rotation[0], offset: offset, scale: [1,1,1],  parentIndex: parent(), dirty: true, world: identity(4) });
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 1, value: bone.rotation[1], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: identity(4) });
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 2, value: bone.rotation[2], offset: [0,0,0], scale: bone.scale, parentIndex: parent(), dirty: true, world: identity(4) });
    }
    indices[i] = joints.length - 1;
  });
  return joints;
}

/**
 * ボーンにジョイントの値を適用
 * @param joints ジョイント
 * @param bones ボーン(出力)
 */
function applyJointsToBones(joints: Joint[], bones: Bone[]): void {
  joints.forEach((joint,i)=>{
    if(joint.type === JointType.Revolution){
      bones[joint.boneIndex].rotation[joint.axis] = joint.value;
    }
    else if(joint.type === JointType.Slide){
      bones[joint.boneIndex].offset[joint.axis] = joint.value;
    }
  });
}

/**
 * ベクトルを配列に変換
 * @param vec ベクトル
 * @returns 配列
 */
function convertVector3ToArray(vec: THREE.Vector3 | THREE.Euler): FArray3 {
  return [vec.x, vec.y, vec.z];
}

/**
 * アニメのウェイト設定
 * @param action アクション
 * @param weight ウェイト 0~1
 */
function setAnimWeight(action: THREE.AnimationAction, weight: number) {
  action.enabled = true;
  action.setEffectiveTimeScale( 1 );
  action.setEffectiveWeight( weight );
  return weight;
}

/**
 * シーン初期化
 */
function initScene() {
  // -----------------------------------------
  // Renderer
  // -----------------------------------------
  renderer = new THREE.WebGLRenderer({ antialias: true, canvas: canvas });
  renderer.localClippingEnabled = true;
  renderer.shadowMap.enabled = true;
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x263238);
  window.addEventListener('resize', onWindowResize, false);
  document.body.appendChild(renderer.domElement);

  clock = new THREE.Clock();
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(36, window.innerWidth / window.innerHeight, 1, 10000);
  camera.position.set(-5, 5, -5);
  // camera.setViewOffset( window.innerWidth, window.innerHeight * 2, 0, window.innerHeight * 0.3, window.innerWidth, window.innerHeight);
  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const dirLight = new THREE.DirectionalLight(0xffffff, 1);
  dirLight.position.set(5, 10, 7.5);
  dirLight.castShadow = true;
  dirLight.shadow.camera.right = 2;
  dirLight.shadow.camera.left = - 2;
  dirLight.shadow.camera.top = 2;
  dirLight.shadow.camera.bottom = - 2;
  dirLight.shadow.mapSize.width = 1024;
  dirLight.shadow.mapSize.height = 1024;
  scene.add(dirLight);

  // helper
  const axis = new THREE.AxesHelper(1000);   
  scene.add(axis);

  // Controls
  const orbit = new OrbitControls(camera, renderer.domElement);
  orbit.minDistance = 2;
  orbit.maxDistance = 20;
  orbit.update();

  // -----------------------------------------
  // 拘束デバッグモデル
  // -----------------------------------------
  constrains.forEach(constrain => {
    if(scene === undefined ||
       camera === undefined ||
       renderer === undefined){
      return;
    }

    const geometry =
      constrain.type === ConstrainType.Position ? new THREE.SphereGeometry(0.05) :
      constrain.type === ConstrainType.Orientation ? new THREE.CylinderGeometry(0, 0.05, 0.3) :
      constrain.type === ConstrainType.OrientationBound ? new THREE.CylinderGeometry(0.1, 0, 0.2) : undefined;
    if(geometry === undefined){
      return;
    }
    const material = new THREE.MeshStandardMaterial({color: 0xFFFFFF, wireframe: true, depthTest: false});    
    constrain.object = new THREE.Mesh(geometry, material);
    if(constrain.type === ConstrainType.Position){
      constrain.object.position.set(...constrain.pos || [0,0,0]);
    }else if(constrain.type === ConstrainType.Orientation){
      constrain.object.rotation.set(...constrain.rot || [0,0,0]);
    }else if(constrain.type === ConstrainType.OrientationBound){
      constrain.object.rotation.set(...constrain.base_rot || [0,0,0]);
    }
    constrain.object.renderOrder = 999; // always display
    scene.add(constrain.object);
    
    // 掴んで移動
    const control = new TransformControls( camera, renderer.domElement );
    constrain.control = control;
    control.size =
      constrain.type === ConstrainType.Orientation ||
      constrain.type === ConstrainType.OrientationBound ? 0.2 : 0.5;
    control.setMode(
      constrain.type === ConstrainType.Position ? "translate" :
      constrain.type === ConstrainType.Orientation ? "rotate" :
      constrain.type === ConstrainType.OrientationBound ? "rotate" : "translate");
    control.attach(constrain.object);
    control.addEventListener( 'change', (event) =>{
      if(constrain.object === undefined){
        return;
      }
      if(constrain.type === ConstrainType.Position){
        constrain.pos = convertVector3ToArray(constrain.object.position);
      }else if(constrain.type === ConstrainType.Orientation){
        constrain.rot = convertVector3ToArray(constrain.object.rotation);
      }else if(constrain.type === ConstrainType.OrientationBound){
        const joints = convertBonesToJoints(bones);
        const parentBone = bones[constrain.bone].parentIndex;
        const parent = parentBone != -1 ? getJointWorldMatrix(joints, convertBoneToJointIndex(joints,parentBone)) : identity(4);
        const rotInv = math.transpose(cancelTranslate(cancelScaling(parent)));
        constrain.base_rot = getRotationXYZ(mul(rotInv, rotXYZ(...convertVector3ToArray(constrain.object.rotation))));
      }
    });
    control.addEventListener( 'dragging-changed', (event) => {
      orbit.enabled = ! event.value;
    });
    scene.add(control);
  })

  // -----------------------------------------
  // モデルロード
  // -----------------------------------------
  var loader = new GLTFLoader();
  loader.load(modelfile, function ( gltf ) {
    if(scene === undefined){
      return;
    }

    model = gltf.scene;
    scene.add( model );

    // ボーンを摘出
    model.traverse( function ( object ) {      
      if(!(/Hand.+/).test(object.name) &&
         ((object as any).isBone ||
          (object.parent && (object.parent as any).isBone)  ||
          object.children.reduce((prev, curr) => prev || (curr as any).isBone, false))){
        const bone: Bone = {
          object: object,
          offset: convertVector3ToArray(object.position),
          rotation: convertVector3ToArray(object.rotation),
          scale: convertVector3ToArray(object.scale),
          parentIndex: bones.findIndex(e=> e.object?.id === object.parent?.id),
          children: []
        }
        bones.push(bone);    
      }
    });
    bones[0].static = true;
    // bones[0].slide = true;

    // アニメーション用ダミーモデル
    animation_compute_model = SkeletonUtils.clone(model) as THREE.Object3D;
    animation_compute_model.visible = false;
    scene.add( animation_compute_model );

    bones.forEach(bone=>{
      bone.animation_object = animation_compute_model?.getObjectByName(bone.object?.name || "");
    });

    // アニメーション
    mixer = new THREE.AnimationMixer(animation_compute_model);
    actions = gltf.animations.map(anim=> mixer?.clipAction(anim)).filter(existFilter);
    actions.forEach((action,i) => {
      setAnimWeight( action, i == 1 ? 1.0 : 0.0 );
      action.play();
    });    

    // helper
    skeleton = new THREE.SkeletonHelper( model );
    skeleton.visible = true;
    scene.add( skeleton );

    animation_compute_skeleton = new THREE.SkeletonHelper( animation_compute_model );
    animation_compute_skeleton.visible = true;
    scene.add( animation_compute_skeleton );
  } );

  // -----------------------------------------
  // ボーン組み立て
  // -----------------------------------------
  /*
  const buildBone = (tree: Bone, parentObject: THREE.Object3D, parentIndex: number)=>{
    const height = tree.children.length > 0 ? 1 : 0.2;
    const geometry = new THREE.CylinderGeometry( 0, 0.05, height);
    geometry.vertices.forEach(e=>e.y+=height/2);
    const material = new THREE.MeshStandardMaterial({color: 0xFFC107});
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...tree.offset);
    mesh.rotation.set(...tree.rotation);
    parentObject.add(mesh);

    tree.object = mesh;
    tree.parentIndex = parentIndex;
    bones.push(tree);

    const index = bones.length - 1;
    tree.children.forEach(e=> buildBone(e, mesh, index));
  }
  const group = new THREE.Group();
  buildBone(root, group, -1);
  scene.add(group);
  */
}

/**
 * ik更新
 * @param delta デルタ時間
 */
function updateIk(delta: number): void {
  if(!bones.length){
    return;
  }

  // model→bones
  bones.forEach(bone =>{
    bone.offset = convertVector3ToArray(bone.object?.position || new THREE.Vector3());
    bone.rotation = convertVector3ToArray(bone.object?.rotation || new THREE.Vector3());
    bone.scale = convertVector3ToArray(bone.object?.scale || new THREE.Vector3());
  });
  
  // ik計算しやすい形に変換
  const joints = convertBonesToJoints(bones);
  // 拘束にジョイント番号を入れておく
  const converted_constrains = [
    ...constrains.map(e=> ({...e,　joint: convertBoneToJointIndex(joints, e.bone)})),
    // ...ref_diff.map((e,i)=> ({priority: 0, joint: i, value: e, type: ConstrainType.RefPose})).filter((e,i)=>i<30) // 拘束による参照姿勢追随
  ];
  
  // スケルトンアニメーション更新
  settings.animation && mixer && mixer.update(delta);
  // 回転ジョイントに関しては、参照ポーズとの差分に対して解を求める。
  // Δθref = θref - θ
  const ref_diff = joints.map((joint,i)=> {
    const bone = bones[joint.boneIndex];
    const obj = bone.animation_object;
    if(obj === undefined){
      return 0;
    }
    const pos = convertVector3ToArray(obj.position);
    const rot = convertVector3ToArray(obj.rotation);
    return (
      joint.type === JointType.Revolution ? (
        !bone.slide && joint.axis === 0 && (bone.offset = joint.offset = pos),
        rotWrap(rot[joint.axis] - joint.value)) : // 角度は近道で移動
      joint.type === JointType.Slide ? (
        pos[joint.axis] - joint.value) :          // 差分
      joint.type === JointType.Static ? (
        bone.offset = joint.offset = pos,
        bone.rotation = joint.rotation =  rot,
        0 ) : 0)
  });

  // ik計算
  solveJacobianIk(joints, converted_constrains, 8, 1/8, ref_diff);

  // joints -> bones
  applyJointsToBones(joints, bones);
  
  // bones->model
  bones.forEach((e,i)=>{
    e.object?.position.set(...e.offset);
    e.object?.rotation.set(...e.rotation);
    e.object?.scale.set(...e.scale);
  });
}


/**
 * ImGuiによるデバッグ表示
 * @param time 時間
 */
function drawImgui(time: number): void {
  ImGui.Begin("Debug Window");
  ImGui.Dummy(new ImGui.ImVec2(400,0));
  
  // 稼働ルート
  if (ImGui.TreeNodeEx("flags##1", ImGui.ImGuiTreeNodeFlags.DefaultOpen))
  {
    if(bones.length){
      ImGui.Checkbox(`slide root`, (value = bones[0].slide || false) => {
        bones[0].slide = value;
        bones[0].static = !value;
        return  value;
      })
    }
    // フラグ
    ImGui.Checkbox(`enable animation`, (value = settings.animation) => settings.animation = value);
    ImGui.Checkbox(`enable debug display`, (value = settings.debugDisp) => settings.debugDisp = value)
    {
      constrains.forEach(e=> e.object && (e.object.visible = settings.debugDisp));
      skeleton && (skeleton.visible = settings.debugDisp);
      animation_compute_skeleton && (animation_compute_skeleton.visible = settings.debugDisp);
    }

    ImGui.TreePop();
  }

  // アニメーション
  if (ImGui.TreeNodeEx("animations##1", ImGui.ImGuiTreeNodeFlags.DefaultOpen))
  {
    actions.forEach((action,i) => {
      ImGui.SliderFloat(`${action.getClip().name} weight`,  (value = action.getEffectiveWeight()) => setAnimWeight(action, value), 0, 1);
    })

    if(mixer !== undefined){
      const _mixer = mixer;
      ImGui.SliderFloat(`timeScale`,  (scale = _mixer.timeScale) => _mixer.timeScale = scale, 0, 5);
    }
  
    ImGui.TreePop();
  }
  
  // ボーンデバッグ表示
  if (ImGui.TreeNode("bones##1"))
  {
    bones.forEach((e,i)=>{
      ImGui.SliderFloat3(`pos[${i}] ${e.object?.name}`, e.offset, -5, 5)
      SliderAngleFloat3(`rot[${i}] ${e.object?.name}`, e.rotation, -180, 180)
      ImGui.SliderFloat3(`scale[${i}] ${e.object?.name}`, e.scale, 0.001, 1)

      // bones->model
      e.object?.position.set(...e.offset);
      e.object?.rotation.set(...e.rotation);
      e.object?.scale.set(...e.scale);
    });
    ImGui.TreePop();
  }
  ImGui.Separator();
  
  // 拘束デバッグ表示
  if (ImGui.TreeNodeEx("constrains##1", ImGui.ImGuiTreeNodeFlags.DefaultOpen))
  {
    // ik計算しやすい形に変換
    const joints = convertBonesToJoints(bones);

    constrains.forEach((constrain,i) => {
      const joint = convertBoneToJointIndex(joints, constrain.bone);
      const pos = getJointWorldPosition(joints, joint);

      ImGui.PushID(i);
      ImGui.Text(`${ConstrainName[constrain.type]}`);

      if(ImGui.Checkbox(`enable`, (value = constrain.enable) => constrain.enable = value)){
        constrain.object && (constrain.object.visible = constrain.enable);
        constrain.control && (constrain.control.enabled = constrain.enable);
      }

      ImGui.Combo(`priority`,  (value = constrain.priority) => constrain.priority = value, PriorityName, 2);

      if(constrain.type === ConstrainType.Position){
        ImGui.SliderFloat3(`constrain pos`, constrain.pos || [0,0,0], -2, 2)
        ImGui.SliderFloat3(`current`, pos, -100, 100);
        if(constrain.object){
          constrain.object.position.set(...constrain.pos || [0,0,0]);
        }
      }
      else if(constrain.type === ConstrainType.Orientation){
        const rot = getJointOrientation(joints, joint);
        SliderAngleFloat3(`constrain rot`, constrain.rot || [0,0,0], -180, 180)
        SliderAngleFloat3(`current`, rot, -180, 180);
        if(constrain.object){
          constrain.object.rotation.set(...constrain.rot || [0,0,0]);  
          constrain.object.position.set(...pos);
        }
      }
      else if(constrain.type === ConstrainType.OrientationBound){
        if(bones[constrain.bone]){
          const parentBone = bones[constrain.bone].parentIndex;
          const parent = parentBone != -1 ? getJointWorldMatrix(joints, convertBoneToJointIndex(joints,parentBone)) : math.identity(4);
          const rot = getJointOrientation(joints, joint);
          SliderAngleFloat3(`constrain rot bound`, constrain.base_rot || [0,0,0], -180, 180)
          SliderAngleFloat3(`current`, rot, -180, 180);
          if(constrain.object){
            constrain.object.rotation.set(...getRotationXYZ(mul(parent, rotXYZ(...constrain.base_rot || [0,0,0]))));  
            constrain.object.position.set(...pos);
          }          
        }
      }
      ImGui.Separator();
      ImGui.PopID();
    });

    ImGui.TreePop();
  }

  ImGui.End(); 
}

/**
 * ウィンドウリサイズ
 */
function onWindowResize() {
  if(camera === undefined ||
     renderer === undefined){
    return;
  }
  camera.aspect = window.innerWidth / window.innerHeight;
  // camera.setViewOffset( window.innerWidth, window.innerHeight * 2, 0, window.innerHeight * 0.3, window.innerWidth, window.innerHeight);
  camera.updateProjectionMatrix();
  
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth, window.innerHeight);
}

/**
 * 更新
 * @param time 
 */
function loop(time: number) {
  const delta = clock?.getDelta() || 0;
  beginImGui(time);

  // ik更新
  updateIk(delta);

  // デバッグ描画
  drawImgui(time);

  // シーン描画
  if(scene !== undefined &&
     renderer !== undefined &&
     camera !== undefined){
      renderer.render(scene, camera);
      renderer.state.reset();
  }

  endImGui();
  requestAnimationFrame(loop);  
}

// 実行
window.onload = async ()=>{
  await initImGui(canvas);

  initScene();
  loop(0);
}