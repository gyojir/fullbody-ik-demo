
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
// import Stats from 'three/examples/jsm/libs/stats.module.js';
import * as ImGui_Impl from "imgui-js/dist/imgui_impl.umd";
import * as ik from './ik';
import { ConstrainType, JointType } from './ik';
import { range, zip } from "./util";

const ImGui = ImGui_Impl.ImGui;

let camera, scene, renderer;
let clock;

const canvas = document.getElementById("canvas");

const targets = [
  {bone: 2, pos: [1,1,0], object: null, type: ConstrainType.Position},
  {bone: 4, pos: [-1,1,0], object: null, type: ConstrainType.Position}
];

let bones = [];
const root = {
  offset: [0,0,0],
  rotation: [0,0,0],
  slide: true, // ik計算用スライドジョイントフラグ
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
  for(const [index, joint] of joints.entries()) {
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
    // value = 関節変位 q
    if(bone.slide){
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 0, value: bone.offset[0], offset: [0,0,0], parentIndex: indices[bone.parentIndex] || -1 });
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 1, value: bone.offset[1], offset: [0,0,0], parentIndex: joints.length - 1 });
      joints.push({ boneIndex: i, type: JointType.Slide, axis: 2, value: bone.offset[2], offset: [0,0,0], parentIndex: joints.length - 1 });
    }
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 0, value: bone.rotation[0], offset: bone.slide ? [0,0,0] : bone.offset, angle_range: [0, 180], parentIndex: bone.slide ? joints.length - 1 : indices[bone.parentIndex] || -1 });
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 1, value: bone.rotation[1], offset: [0,0,0], angle_range: [0, 180], parentIndex: joints.length - 1 });
    joints.push({ boneIndex: i, type: JointType.Revolution, axis: 2, value: bone.rotation[2], offset: [0,0,0], angle_range: [0, 180], parentIndex: joints.length - 1 });
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

function draw_imgui(time) {   
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();
  ImGui.Begin("My Window");
  ImGui.Dummy(new ImGui.ImVec2(400,0));
  
  // 全ジョイント表示
  bones.forEach((bone,i)=>{
    ImGui.PushID(i);
    ImGui.SliderFloat3(`pos[${i}]`, bone.offset, -10, 10);
    ImGui.SliderAngle3(`rot[${i}]`, bone.rotation);
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
    const targetIndices = targets.map(e=> convertBoneToJointIndex(joints, e.bone));

    ik.solve_jacobian_ik(joints, targets.map(e=>e.pos), targetIndices, 1);
    convertJointsToBones(joints, bones);
    
    targets.forEach((target,i) => {
      if(ImGui.SliderFloat3(`target[${i}]`, target.pos, -2, 2) && target.object){
        target.object.position.set(...target.pos);
      }
      const eff_pos = ik.getEffectorPosition(joints, targetIndices[i]).toArray();
      ImGui.SliderFloat3(`effector_pos[${i}]`, eff_pos, -100, 100);
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

  // ターゲット
  targets.forEach(target => {
    const geometry = new THREE.SphereGeometry(0.1);
    const material = new THREE.MeshStandardMaterial({color: 0xFFFFFF});
    target.object = new THREE.Mesh(geometry, material);
    target.object.position.set(...target.pos);
    scene.add(target.object);
  })

  // -----------------------------------------
  // 組み立て
  // -----------------------------------------
  const build = (bone, parent)=>{
    const height = bone.children.length > 0 ? 1 : 0.1;
    const geometry = new THREE.CylinderGeometry( 0, 0.1, height);
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
