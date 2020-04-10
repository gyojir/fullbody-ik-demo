
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
// import Stats from 'three/examples/jsm/libs/stats.module.js';
import * as math from "mathjs";
import * as ImGui_Impl from "imgui-js/dist/imgui_impl.umd";
import { mul, solve_jacobian_ik, getEffectorWorldPosition, getEffectorOrientation, ConstrainType, JointType, getEffectorWorldMatrix, rotXYZ, getRotationXYZ } from './ik';
import { range, zip } from "./util";

const ImGui = ImGui_Impl.ImGui;

let camera, scene, renderer;
let clock;

const canvas = document.getElementById("canvas");

const constrains = [
  {priority: 0, bone: 2, joint: -1, pos: [1,1,0], object: null, type: ConstrainType.Position},
  // {priority: 0, bone: 2, joint: -1, rot: [1,0,0], object: null, type: ConstrainType.Orientation},
  {priority: 1, bone: 0, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: null, type: ConstrainType.OrientationBound},
  {priority: 1, bone: 2, joint: -1, bounds: {gamma_max: Math.PI/4}, base_rot: [0,0,0], object: null, type: ConstrainType.OrientationBound},
  // {priority: 0, bone: 4, joint: -1, pos: [-1,1,0], object: null, type: ConstrainType.Position}
];

let bones = [];
const root = {
  offset: [0,0,0],
  rotation: [0,0,0],
  // slide: true, // ik計算用スライドジョイントフラグ
  children: [{
      offset: [0.5,1,0],
      rotation: [0,0,0],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        children: []
      }]
    },
    {
      offset: [-0.5,1,0],
      rotation: [0,0,0],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        children: []
      }]
    },
    {
      offset: [0.5,0,0],
      rotation: [Math.PI,0,0],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        children: []
      }]
    },
    {
      offset: [-0.5,0,0],
      rotation: [Math.PI,0,0],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
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
    let parent = (()=> { let j = 0; return ()=> j++ === 0 ? indices[bone.parentIndex] || -1 : joints.length - 1; })();

    // value = 関節変位 q
    // スライダジョイントを挿入
    if(bone.slide){
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 0, value: bone.offset[0], offset: [0,0,0], parentIndex: parent() });
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 1, value: bone.offset[1], offset: [0,0,0], parentIndex: parent() });
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 2, value: bone.offset[2], offset: [0,0,0], parentIndex: parent() });
    }

    // XYZ回転
    let offset = bone.slide ? [0,0,0] : bone.offset;
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 0, value: bone.rotation[0], offset: offset, angle_range: [0, 180], parentIndex: parent() });
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 1, value: bone.rotation[1], offset: [0,0,0], angle_range: [0, 180], parentIndex: parent() });
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 2, value: bone.rotation[2], offset: [0,0,0], angle_range: [0, 180], parentIndex: parent() });
    indices[i] = joints.length - 1;
  });
  return joints;
}

function convertJointsToBones(joints, bones){
  joints.forEach((joint,i)=>{
    if(joint.type === JointType.Slide){
      bones[joint.boneIndex].offset[joint.axis] = joint.value;
    }else{
      bones[joint.boneIndex].rotation[joint.axis] = joint.value;
    }
  });
}


export function SliderAngleFloat3(label, v_rad, v_degrees_min = -360.0, v_degrees_max = +360.0) {
  let _v_rad = math.clone(v_rad);
  _v_rad = math.multiply(_v_rad, 180 / Math.PI);
  const ret = ImGui.SliderFloat3(label, _v_rad, v_degrees_min, v_degrees_max, "%.1f deg");
  _v_rad = math.multiply(_v_rad, Math.PI / 180);
  v_rad.forEach((e,i)=> v_rad[i] = _v_rad[i]);
  return ret;
}

function draw_imgui(time) {   
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();
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
      const pos = getEffectorWorldPosition(joints, constrain.joint).toArray();

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
  ImGui.EndFrame();
  ImGui.Render();
  ImGui_Impl.RenderDrawData(ImGui.GetDrawData());
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
  constrains.forEach(constrain => {
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

  // -----------------------------------------
  // 組み立て
  // -----------------------------------------
  const build = (bone, parent)=>{
    const height = bone.children.length > 0 ? 1 : 0.2;
    const geometry = new THREE.CylinderGeometry( 0, 0.05, height);
    geometry.vertices.forEach(e=>e.y+=height/2);
    const material = new THREE.MeshStandardMaterial({color: 0xFFC107});
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...bone.offset);
    mesh.rotation.set(...bone.rotation);
    parent.object.add(mesh);

    bone.object = mesh;
    bone.index = bones.length;
    bone.parentIndex = parent.index;
    bones.push(bone);

    bone.children.forEach(e=> build(e, bone));
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

  //
  renderer.render(scene, camera);
  renderer.state.reset();

  draw_imgui(time);
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

(async ()=>{
  await init_imgui();

  init_three();
  animate();
})()
