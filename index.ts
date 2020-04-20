
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonUtils } from 'three/examples/jsm/utils/SkeletonUtils.js';
// import Stats from 'three/examples/jsm/libs/stats.module.js';
import * as math from "mathjs";
// import * as ImGui from "imgui-js/imgui";
// import * as ImGui_Impl from "imgui-js/example/imgui_impl"
import { getJointOrientation, getJointWorldMatrix, solve_jacobian_ik, getJointWorldPosition } from './ik';
import { mul, rotXYZ, getRotationXYZ }from './math-util';
import { range, zip, rotWrap } from "./util";
import { Joint, ConstrainType, JointType, FArray3, Bone, Constrain } from './def';
const ImGui_Impl = require("imgui-js/dist/imgui_impl.umd");
const modelfile = require('./models/Soldier.glb');

const ImGui = ImGui_Impl.ImGui;

let camera : THREE.PerspectiveCamera | undefined;
let scene : THREE.Scene | undefined;
let renderer : THREE.WebGLRenderer | undefined;
let model : THREE.Object3D | undefined;
let animation_compute_model : THREE.Object3D | undefined;
let skeleton : THREE.SkeletonHelper | undefined;
let mixer : THREE.AnimationMixer | undefined;
var actions : THREE.AnimationAction[] = [];
let clock : THREE.Clock | undefined;

const canvas = document.getElementById("canvas") as HTMLCanvasElement;

/*
let bones : Bone[] = [];
const constrains: Constrain[] = [
  {priority: 1, bone: 15, joint: -1, pos: [0.5,1,0], object: undefined, type: ConstrainType.Position, enable: true},
  {priority: 1, bone: 15, joint: -1, rot: [0.5,1,0], object: undefined, type: ConstrainType.Orientation, enable: true},
  {priority: 1, bone: 11, joint: -1, pos: [-0.5,1,0], object: undefined, type: ConstrainType.Position, enable: true},
];
*/

const constrains: Constrain[] = [
  {priority: 1, bone: 2, joint: -1, pos: [1,1,0], object: undefined, type: ConstrainType.Position, enable: true},
  {priority: 0, bone: 2, joint: -1, rot: [1,0,0], object: undefined, type: ConstrainType.Orientation, enable: true},
  {priority: 0, bone: 2, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: undefined, type: ConstrainType.OrientationBound, enable: true},
  {priority: 0, bone: 0, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: undefined, type: ConstrainType.OrientationBound, enable: true},
  {priority: 1, bone: 4, joint: -1, pos: [-1,1,0], object: undefined, type: ConstrainType.Position, enable: true}
];

let bones: Bone[] = [];
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
  const joints: any = [];
  const indices: number[] = [];
  bones.forEach((bone,i)=>{
    let parent = (()=> { let j = 0; return ()=> j++ === 0 && indices[bone.parentIndex] !== undefined ? indices[bone.parentIndex] : joints.length - 1; })();

    if(bone.static) {
      joints.push({ boneIndex: i, type: JointType.Static, axis: 0, value: 0, offset: bone.offset, scale: bone.scale, rotation: bone.rotation, parentIndex: parent(), dirty: true, world: math.identity(4) });      
    }
    else {
      // value = 関節変位 q
      // スライダジョイントを挿入
      if(bone.slide){
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 0, value: bone.offset[0], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: math.identity(4) });
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 1, value: bone.offset[1], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: math.identity(4) });
        joints.push({ boneIndex: i, type: JointType.Slide, axis: 2, value: bone.offset[2], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: math.identity(4) });
      }
  
      // XYZ回転
      let offset = bone.slide ? [0,0,0] : bone.offset;
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 0, value: bone.rotation[0], offset: offset, scale: [1,1,1],  parentIndex: parent(), dirty: true, world: math.identity(4) });
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 1, value: bone.rotation[1], offset: [0,0,0], scale: [1,1,1], parentIndex: parent(), dirty: true, world: math.identity(4) });
      joints.push({ boneIndex: i, type: JointType.Revolution, axis: 2, value: bone.rotation[2], offset: [0,0,0], scale: bone.scale, parentIndex: parent(), dirty: true, world: math.identity(4) });
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
 * ImGuiの角度表示関数改良版
 * @param label 
 * @param v_rad 
 * @param v_degrees_min 
 * @param v_degrees_max 
 */
export function SliderAngleFloat3(label: string, v_rad: number[], v_degrees_min = -360.0, v_degrees_max = +360.0): boolean {
  let _v_rad = math.clone(v_rad);
  _v_rad = math.multiply(_v_rad, 180 / Math.PI);
  const ret = ImGui.SliderFloat3(label, _v_rad, v_degrees_min, v_degrees_max, "%.1f deg");
  _v_rad = math.multiply(_v_rad, Math.PI / 180);
  v_rad.forEach((e,i)=> v_rad[i] = _v_rad[i]);
  return ret;
}

/**
 * ImGuiによるデバッグ表示
 * @param delta デルタ時間
 */
function draw_imgui(delta: number): void {   
  // ------------------------------------------
  {
    ImGui.Begin("Debug Window2");
    ImGui.Dummy(new ImGui.ImVec2(400,0));
    
    // アニメーション
    actions.forEach((action,i) => {
      ImGui.SliderFloat(`${action.getClip().name} weight`,  (value = action.getEffectiveWeight()) => setAnimWeight(action, value), 0, 1);
    })

    // model→bones
    bones.forEach(bone =>{
      bone.offset = convertVector3ToArray(bone.object?.position || new THREE.Vector3());
      bone.rotation = convertVector3ToArray(bone.object?.rotation || new THREE.Vector3());
      bone.scale = convertVector3ToArray(bone.object?.scale || new THREE.Vector3());
    });
    
    // ik計算しやすい形に変換
    const joints = convertBonesToJoints(bones);
    
    // 回転ジョイントに関しては、参照ポーズとの差分に対して解を求める。
    // Δθref = θref - θ
    // スケルトンアニメーション
    if(mixer !== undefined){
      const _mixer = mixer;
      ImGui.SliderFloat(`timeScale`,  (scale = _mixer.timeScale) => _mixer.timeScale = scale, 0, 5);
      mixer.update(delta);
    }
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

    const converted_constrains = [
      ...constrains.map(e=> ({
        ...e,
        joint: convertBoneToJointIndex(joints, e.bone)
      })),
      // ...ref_diff.map((e,i)=> ({priority: 0, joint: i, value: e, type: ConstrainType.RefPose})).filter((e,i)=>i<30)
    ];

    solve_jacobian_ik(joints, converted_constrains, 10, 0.1, ref_diff);
    applyJointsToBones(joints, bones);

    // デバッグ表示
    if (ImGui.TreeNode("bones##1"))
    {
      bones.forEach((e,i)=>{
        ImGui.SliderFloat3(`pos[${i}] ${e.object?.name}`, e.offset, -5, 5)
        SliderAngleFloat3(`rot[${i}] ${e.object?.name}`, e.rotation, -180, 180)
        ImGui.SliderFloat3(`scale[${i}] ${e.object?.name}`, e.scale, 0.001, 1)
      });
    }
    ImGui.Separator();

    // bones->model
    bones.forEach((e,i)=>{
      e.object?.position.set(...e.offset);
      e.object?.rotation.set(...e.rotation);
      e.object?.scale.set(...e.scale);
    })
    
    // デバッグ表示
    if (ImGui.TreeNodeEx("constrains##1", ImGui.ImGuiTreeNodeFlags.DefaultOpen))
    {
      converted_constrains.forEach((constrain,i) => {
        const origin_constrain = constrains[i];
        const pos = getJointWorldPosition(joints, constrain.joint);

        if(ImGui.Checkbox(`enable[${i}]`, (value = origin_constrain.enable) => origin_constrain.enable = value)){
          origin_constrain.object && (origin_constrain.object.visible = origin_constrain.enable);
          origin_constrain.control && (origin_constrain.control.enabled = origin_constrain.enable);
        }

        if(constrain.type === ConstrainType.Position){
          ImGui.SliderFloat3(`pos constrain[${i}]`, constrain.pos, -2, 2)
          ImGui.SliderFloat3(`effector_pos[${i}]`, pos, -100, 100);
          if(constrain.object){
            constrain.object.position.set(...constrain.pos || [0,0,0]);
          }
        }
        else if(constrain.type === ConstrainType.Orientation){
          const rot = getJointOrientation(joints, constrain.joint);
          SliderAngleFloat3(`rot constrain[${i}]`, constrain.rot || [0,0,0], -180, 180)
          SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
          if(constrain.object){
            constrain.object.rotation.set(...constrain.rot || [0,0,0]);  
            constrain.object.position.set(...pos);
          }
        }
        else if(constrain.type === ConstrainType.OrientationBound){
          const parentBone = bones[constrain.bone].parentIndex;
          const parent = parentBone != -1 ? getJointWorldMatrix(joints, convertBoneToJointIndex(joints,parentBone)) : math.identity(4);
          const rot = getJointOrientation(joints, constrain.joint);
          SliderAngleFloat3(`rot bound constrain[${i}]`, constrain.base_rot || [0,0,0], -180, 180)
          SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
          if(constrain.object){
            constrain.object.rotation.set(...getRotationXYZ(mul(parent, rotXYZ(...constrain.base_rot || [0,0,0]))));  
            constrain.object.position.set(...pos);
          }
        }
        ImGui.Separator();
      })
    }

    ImGui.End(); 
  }

}

/**
 * ImGui初期化
 */
async function init_imgui() {
  await ImGui.default();
  ImGui.IMGUI_CHECKVERSION();
  ImGui.CreateContext();
  ImGui_Impl.Init(canvas);

  canvas.addEventListener( 'mousedown', event=>{
    if(!ImGui.IsWindowHovered()){
      event.stopImmediatePropagation();
    }
  }, false );
}

/**
 * シーン初期化
 */
function init_three() {
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
  camera.position.set(2, 2, 2);
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
  // 拘束
  // -----------------------------------------
  constrains.forEach(constrain => {
    if(scene === undefined ||
       camera === undefined ||
       renderer === undefined){
      return;
    }

    const geometry =
      constrain.type === ConstrainType.Position ? new THREE.SphereGeometry(0.1) :
      constrain.type === ConstrainType.Orientation ? new THREE.CylinderGeometry(0, 0.05, 0.3) :
      constrain.type === ConstrainType.OrientationBound ? new THREE.CylinderGeometry(0.1, 0, 0.2) : undefined;
    const material = new THREE.MeshStandardMaterial({color: 0xFFFFFF, wireframe: true});    
    if(geometry === undefined){
      return;
    }

    constrain.object = new THREE.Mesh(geometry, material);
    if(constrain.type === ConstrainType.Position){
      constrain.object.position.set(...constrain.pos || [0,0,0]);
    }else if(constrain.type === ConstrainType.Orientation){
      constrain.object.rotation.set(...constrain.rot || [0,0,0]);
    }else if(constrain.type === ConstrainType.OrientationBound){
      constrain.object.rotation.set(...constrain.base_rot || [0,0,0]);
    }
    scene.add(constrain.object);
    
    // 掴んで移動
    const control = new TransformControls( camera, renderer.domElement );
    constrain.control = control;
    control.size = 0.3;
    control.setMode(
      constrain.type === ConstrainType.Position ? "translate" :
      constrain.type === ConstrainType.Orientation ? "rotate" :
      constrain.type === ConstrainType.OrientationBound ? "rotate" : "translate");
    control.attach(constrain.object);
    control.addEventListener( 'change', (event) =>{
      if(constrain.type === ConstrainType.Position){
        constrain.pos = convertVector3ToArray(constrain.object?.position || new THREE.Vector3());
      }else if(constrain.type === ConstrainType.Orientation){
        constrain.rot = convertVector3ToArray(constrain.object?.rotation || new THREE.Vector3());
      }else if(constrain.type === ConstrainType.OrientationBound){
        constrain.base_rot = convertVector3ToArray(constrain.object?.rotation || new THREE.Vector3());
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
  /*
  var loader = new GLTFLoader();
  loader.load(modelfile, function ( gltf ) {
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
          parentIndex: bones.findIndex(e=> e.object.id === object.parent.id),
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
      bone.animation_object = animation_compute_model.getObjectByName(bone.object.name);
    });

    // アニメーション
    mixer = new THREE.AnimationMixer(animation_compute_model);
    actions = gltf.animations.map(anim=> mixer.clipAction(anim));
    actions.forEach((action,i) => {
      setAnimWeight( action, i == 1 ? 1.0 : 0.0 );
      action.play();
    });    

    // helper
    skeleton = new THREE.SkeletonHelper( model );
    skeleton.visible = true;
    scene.add( skeleton );

    let skeleton_2 = new THREE.SkeletonHelper( animation_compute_model );
    skeleton_2.visible = true;
    scene.add( skeleton_2 );
  } );
  */

  // -----------------------------------------
  // 組み立て
  // -----------------------------------------
  const build = (tree: Bone, parentObject: THREE.Object3D, parentIndex: number)=>{
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
    tree.children.forEach(e=> build(e, mesh, index));
  }
  const group = new THREE.Group();
  build(root, group, -1);
  scene.add(group);
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
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

/**
 * 更新
 * @param time 
 */
function animate(time: number) {
  if(scene === undefined ||
     clock === undefined ||
     renderer === undefined ||
     camera === undefined){
    requestAnimationFrame(animate);
    return;
  }

  const delta = clock.getDelta();
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();

  // デバッグ描画
  draw_imgui(delta);

  // シーン描画
  renderer.render(scene, camera);
  renderer.state.reset();

  ImGui.EndFrame();
  ImGui.Render();
  ImGui_Impl.RenderDrawData(ImGui.GetDrawData());

  requestAnimationFrame(animate);
}

// 実行
(async ()=>{
  await init_imgui();

  init_three();
  animate(0);
})()
