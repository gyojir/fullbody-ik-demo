
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { SkeletonUtils } from 'three/examples/jsm/utils/SkeletonUtils.js';
// import Stats from 'three/examples/jsm/libs/stats.module.js';
import * as math from "mathjs";
import * as ImGui_Impl from "imgui-js/dist/imgui_impl.umd";
import * as ik from './ik';
import { mul, solve_jacobian_ik, getEffectorWorldPosition, getEffectorOrientation, ConstrainType, JointType, getEffectorWorldMatrix, rotXYZ, getRotationXYZ, getRotationError, getRotationFromAxis, normalize } from './ik';
import { range, zip, rotWrap as rotWrap } from "./util";
import modelfile from './models/Soldier.glb';

const ImGui = ImGui_Impl.ImGui;

let camera, scene, renderer;
let model, animation_compute_model, skeleton, mixer;
var actions;
let clock;

var step = 1.0;

const canvas = document.getElementById("canvas");

let model_bones = [];

const model_constrains = [
  {priority: 1, bone: 33, joint: -1, pos: [0.5,1,0], object: null, type: ConstrainType.Position},
  // {priority: 1, bone: 11, joint: -1, pos: [-0.5,1,0], object: null, type: ConstrainType.Position},
];

const constrains = [
  {priority: 1, bone: 2, joint: -1, pos: [1,1,0], object: null, type: ConstrainType.Position},
  {priority: 0, bone: 2, joint: -1, rot: [1,0,0], object: null, type: ConstrainType.Orientation},
  {priority: 0, bone: 2, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: null, type: ConstrainType.OrientationBound},
  {priority: 0, bone: 0, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: null, type: ConstrainType.OrientationBound},
  {priority: 0, bone: 4, joint: -1, pos: [-1,1,0], object: null, type: ConstrainType.Position}
];

let bones = [];
const root = {
  offset: [0,0,0],
  rotation: [0,0,0],
  scale: [1,1,1],
  slide: true, // ik計算用スライドジョイントフラグ
  children: [{
      offset: [0.5,1,0],
      rotation: [0,0,0],
      scale: [1,1,1],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        children: []
      }]
    },
    {
      offset: [-0.5,1,0],
      rotation: [0,0,0],
      scale: [1,1,1],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        children: []
      }]
    },
    {
      offset: [0.5,0,0],
      rotation: [Math.PI,0,0],
      scale: [1,1,1],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        children: []
      }]
    },
    {
      offset: [-0.5,0,0],
      rotation: [Math.PI,0,0],
      scale: [1,1,1],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        scale: [1,1,1],
        children: []
      }]
    }
  ]
}

function convertBoneToJointIndex(joints, boneIndex){
  // ボーンに対応する一番最後のジョイントを選ぶ
  for(const [index, joint] of [...joints.entries()].reverse()) {
    if(joint.boneIndex === boneIndex){
      return index;
    }
  }
  return -1;
}

// 回転と移動を分解する
function convertBonesToJoints(bones){
  const joints = [];
  const indices = [];
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

function convertJointsToBones(joints, bones){
  joints.forEach((joint,i)=>{
    if(joint.type === JointType.Revolution){
      bones[joint.boneIndex].rotation[joint.axis] = joint.value;
    }
    else if(joint.type === JointType.Slide){
      bones[joint.boneIndex].offset[joint.axis] = joint.value;
    }
  });
}

function convertVector3ToArray(vec){
  return [vec.x, vec.y, vec.z];
}

export function SliderAngleFloat3(label, v_rad, v_degrees_min = -360.0, v_degrees_max = +360.0) {
  let _v_rad = math.clone(v_rad);
  _v_rad = math.multiply(_v_rad, 180 / Math.PI);
  const ret = ImGui.SliderFloat3(label, _v_rad, v_degrees_min, v_degrees_max, "%.1f deg");
  _v_rad = math.multiply(_v_rad, Math.PI / 180);
  v_rad.forEach((e,i)=> v_rad[i] = _v_rad[i]);
  return ret;
}

function draw_imgui(delta) {   

  /*
  {
    ImGui.Begin("Debug Window");
    ImGui.Dummy(new ImGui.ImVec2(400,0));
    
    // 全ジョイント表示
    bones.forEach((bone,i)=>{
      ImGui.PushID(i);
      ImGui.SliderFloat3(`pos[${i}]`, bone.offset, -10, 10);
      SliderAngleFloat3(`rot[${i}]`, bone.rotation);
      if(bone.object){
        bone.object.position.set(...bone.offset);
        bone.object.rotation.set(...bone.rotation);
      }
      ImGui.PopID();
      ImGui.Separator();
    })

    // ik計算
    {
      // ik計算しやすい形に変換
      const joints = convertBonesToJoints(bones);
      const converted_constrains = constrains.map(e=> ({
        ...e,
        joint: convertBoneToJointIndex(joints, e.bone)
      }));

      solve_jacobian_ik(joints, converted_constrains, 1);
      convertJointsToBones(joints, bones);
      
      // デバッグ表示
      converted_constrains.forEach((constrain,i) => {
        const pos = getEffectorWorldPosition(joints, constrain.joint);

        if(constrain.type === ConstrainType.Position){
          ImGui.SliderFloat3(`pos constrain[${i}]`, constrain.pos, -2, 2)
          ImGui.SliderFloat3(`effector_pos[${i}]`, pos, -100, 100);
          if(constrain.object){
            constrain.object.position.set(...constrain.pos);
          }
        }
        else if(constrain.type === ConstrainType.Orientation){
          const rot = getEffectorOrientation(joints, constrain.joint);
          SliderAngleFloat3(`rot constrain[${i}]`, constrain.rot, -180, 180)
          SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
          if(constrain.object){
            constrain.object.rotation.set(...constrain.rot);  
            constrain.object.position.set(...pos);
          }
        }
        else if(constrain.type === ConstrainType.OrientationBound){
          const parentBone = bones[constrain.bone].parentIndex;
          const parent = parentBone != -1 ? getEffectorWorldMatrix(joints, convertBoneToJointIndex(joints,parentBone)) : math.identity(4);
          const rot = getEffectorOrientation(joints, constrain.joint);
          SliderAngleFloat3(`rot bound constrain[${i}]`, constrain.base_rot, -180, 180)
          SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
          if(constrain.object){
            constrain.object.rotation.set(...getRotationXYZ(mul(parent, rotXYZ(...constrain.base_rot))));  
            constrain.object.position.set(...pos);
          }
        }
        ImGui.Separator();
      })
    }

    ImGui.End();
  }
  */

  // ------------------------------------------
  if(actions && model){
    ImGui.Begin("Debug Window2");
    ImGui.Dummy(new ImGui.ImVec2(400,0));
    
    // アニメーション
    actions.forEach((action,i) => {
      ImGui.SliderFloat(`${action._clip.name} weight`,  (value = action.getEffectiveWeight()) => setWeight(action, value), 0, 1);
    })

    // model→bones
    model_bones.forEach(bone =>{
      bone.offset = convertVector3ToArray(bone.object.position);
      bone.rotation = convertVector3ToArray(bone.object.rotation);
      bone.scale = convertVector3ToArray(bone.object.scale);
    });
    
    // ik計算しやすい形に変換
    const joints = convertBonesToJoints(model_bones);
    
    // 回転ジョイントに関しては、参照ポーズとの差分に対して解を求める。
    // Δθref = θref - θ
    // スケルトンアニメーション
    ImGui.SliderFloat(`timeScale`,  (scale = mixer.timeScale) => mixer.timeScale = scale, 0, 5);
    mixer.update(delta);
    const bone_anim = model_bones.map(bone => convertVector3ToArray(bone.animation_object.rotation));
    const anim_diff = joints.map(joint=>
      joint.type === JointType.Revolution ? bone_anim[joint.boneIndex][joint.axis] : 0);
    const ref_diff = joints.map((joint,i)=>
      joint.type === JointType.Revolution ? rotWrap(anim_diff[i] - joint.value) : 0);

    const converted_constrains = [
      ...model_constrains.map(e=> ({
        ...e,
        joint: convertBoneToJointIndex(joints, e.bone)
      })),
      // ...ref_diff.map((e,i)=> ({priority: 0, joint: i, value: e, type: ConstrainType.RefPose})).filter((e,i)=>i<30)
    ];

    solve_jacobian_ik(joints, converted_constrains, ref_diff, 10 , 0.1);
    convertJointsToBones(joints, model_bones);

    // デバッグ表示
    if (ImGui.TreeNode("bones##1"))
    {
      model_bones.forEach((e,i)=>{
        ImGui.SliderFloat3(`pos[${i}] ${e.object.name}`, e.offset, -5, 5)
        SliderAngleFloat3(`rot[${i}] ${e.object.name}`, e.rotation, -180, 180)
        ImGui.SliderFloat3(`scale[${i}] ${e.object.name}`, e.scale, 0.001, 1)
      });
    }
    ImGui.Separator();

    // bones->model
    model_bones.forEach((e,i)=>{
      e.object.position.set(...e.offset);
      e.object.rotation.set(...e.rotation);
      e.object.scale.set(...e.scale);
    })
    
    // デバッグ表示
    converted_constrains.forEach((constrain,i) => {
      const pos = getEffectorWorldPosition(joints, constrain.joint);

      if(constrain.type === ConstrainType.Position){
        ImGui.SliderFloat3(`pos constrain[${i}]`, constrain.pos, -2, 2)
        ImGui.SliderFloat3(`effector_pos[${i}]`, pos, -100, 100);
        if(constrain.object){
          constrain.object.position.set(...constrain.pos);
        }
      }
      else if(constrain.type === ConstrainType.Orientation){
        const rot = getEffectorOrientation(joints, constrain.joint);
        SliderAngleFloat3(`rot constrain[${i}]`, constrain.rot, -180, 180)
        SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
        if(constrain.object){
          constrain.object.rotation.set(...constrain.rot);  
          constrain.object.position.set(...pos);
        }
      }
      else if(constrain.type === ConstrainType.OrientationBound){
        const parentBone = bones[constrain.bone].parentIndex;
        const parent = parentBone != -1 ? getEffectorWorldMatrix(joints, convertBoneToJointIndex(joints,parentBone)) : math.identity(4);
        const rot = getEffectorOrientation(joints, constrain.joint);
        SliderAngleFloat3(`rot bound constrain[${i}]`, constrain.base_rot, -180, 180)
        SliderAngleFloat3(`effector_rot[${i}]`, rot, -180, 180);
        if(constrain.object){
          constrain.object.rotation.set(...getRotationXYZ(mul(parent, rotXYZ(...constrain.base_rot))));  
          constrain.object.position.set(...pos);
        }
      }
      ImGui.Separator();
    })

    ImGui.End(); 
  }

}

function init_three() {
  // Renderer
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

  const axis = new THREE.AxisHelper(1000);   
  scene.add(axis);

  // Controls
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.minDistance = 2;
  controls.maxDistance = 20;
  controls.update();

  // 拘束
  [...constrains, ...model_constrains].forEach(constrain => {
    const geometry =
      constrain.type === ConstrainType.Position ? new THREE.SphereGeometry(0.1) :
      constrain.type === ConstrainType.Orientation ? new THREE.CylinderGeometry(0, 0.05, 0.3) :
      constrain.type === ConstrainType.OrientationBound ? new THREE.CylinderGeometry(0.1, 0, 0.2) : null;
    const material = new THREE.MeshStandardMaterial({color: 0xFFFFFF, wireframe: true});
    constrain.object = new THREE.Mesh(geometry, material);
    if(constrain.type === ConstrainType.Position){
      constrain.object.position.set(...constrain.pos);
    }else if(constrain.type === ConstrainType.Orientation){
      constrain.object.rotation.set(...constrain.rot);
    }else if(constrain.type === ConstrainType.Orientation){
      constrain.object.rotation.set(...constrain.base_rot);
    }
    scene.add(constrain.object);
  })

  
  var loader = new GLTFLoader();
  loader.load(modelfile, function ( gltf ) {
    model = gltf.scene;
    scene.add( model );

    model.traverse( function ( object ) {      
      if(object.isBone ||
         (object.parent && object.parent.isBone)  ||
         object.children.reduce((prev, curr) => prev || curr.isBone, false)){
        const bone = {
          id: object.id,
          name: object.name,
          object: object,
          offset: convertVector3ToArray(object.position),
          rotation: convertVector3ToArray(object.rotation),
          scale: convertVector3ToArray(object.scale),
          parentIndex: model_bones.findIndex(e=> e.id === object.parent.id),
        }
        model_bones.push(bone);    
      }
    });
    model_bones[0].static = true;
    // model_bones[0].slide = true;

    skeleton = new THREE.SkeletonHelper( model );
    skeleton.visible = true;
    scene.add( skeleton );

    animation_compute_model = SkeletonUtils.clone(model);
    animation_compute_model.visible = false;
    scene.add( animation_compute_model );
    let skeleton_2 = new THREE.SkeletonHelper( animation_compute_model );
    skeleton_2.visible = true;
    scene.add( skeleton_2 );


    model_bones.forEach(bone=>{
      bone.animation_object = animation_compute_model.getObjectByName(bone.object.name);
    });

    mixer = new THREE.AnimationMixer(animation_compute_model);
    var animations = gltf.animations;
    actions = animations.map(anim=> mixer.clipAction(anim));

    actions.forEach((action,i) => {
      setWeight( action, i == 1 ? 1.0 : 0.0 );
      action.play();
    });
  } );

  // -----------------------------------------
  // 組み立て
  // -----------------------------------------
  const build = (tree, parent)=>{
    const height = tree.children.length > 0 ? 1 : 0.2;
    const geometry = new THREE.CylinderGeometry( 0, 0.05, height);
    geometry.vertices.forEach(e=>e.y+=height/2);
    const material = new THREE.MeshStandardMaterial({color: 0xFFC107});
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...tree.offset);
    mesh.rotation.set(...tree.rotation);
    parent.object.add(mesh);

    tree.object = mesh;
    tree.index = bones.length;
    tree.parentIndex = parent.index;
    bones.push(tree);

    tree.children.forEach(e=> build(e, tree));
  }
  const group = new THREE.Group();
  build(root, {object: group, index: -1});
  scene.add(group);
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate(time) {
  const delta = clock.getDelta();
  requestAnimationFrame(animate);
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();

  draw_imgui(delta);

  //
  renderer.render(scene, camera);
  renderer.state.reset();

  ImGui.EndFrame();
  ImGui.Render();
  ImGui_Impl.RenderDrawData(ImGui.GetDrawData());
}

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

function setWeight( action, weight ) {
  action.enabled = true;
  action.setEffectiveTimeScale( 1 );
  action.setEffectiveWeight( weight );
  return weight;
}

(async ()=>{
  await init_imgui();

  init_three();
  animate();
})()
