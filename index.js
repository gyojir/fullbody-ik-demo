
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
// import Stats from 'three/examples/jsm/libs/stats.module.js';
import * as ImGui_Impl from "imgui-js/dist/imgui_impl.umd";
import * as ik from './ik';

const ImGui = ImGui_Impl.ImGui;

let camera, scene, renderer;
let clock;

const canvas = document.getElementById("canvas");

const target = [1,1,1]
let targetObject = null;

let bones = [];
const root = {
  offset: [0,0,0],
  rotation: [0,0,0],
  children: [{
      offset: [0.5,1,0],
      rotation: [1,0,0],
      children: [{
        offset: [0,1,0],
        rotation: [0,0,0],
        children: []
      }]
    },
    {
      offset: [-0.5,1,0],
      rotation: [0,0,0],
      children: []
    },
    {
      offset: [0.5,0,0],
      rotation: [Math.PI,0,0],
      children: []
    },
    {
      offset: [-0.5,0,0],
      rotation: [Math.PI,0,0],
      children: []
    }]
}

function convertBonesToLinks(bones){
  const links = [];
  bones.forEach((bone,i)=>{
    links.push(...ik.range(3).map(axis=>({ boneIndex: i, axis: axis, angle: bone.rotation[axis], offset: axis == 0 ? bone.offset : [0,0,0], angle_range: [0, 180] })));
  });
  return links;
}

function draw_imgui(time) {   
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();
  ImGui.Begin("My Window");
  ImGui.Dummy(new ImGui.ImVec2(400,0));

  bones.forEach((link,i)=>{
    ImGui.PushID(i);
    ImGui.SliderFloat3("pos"+i, link.offset, -10, 10);
    ImGui.SliderAngle3("rot"+i, link.rotation);
    if(link.object){
      link.object.position.set(link.offset[0], link.offset[1], link.offset[2]);
      link.object.rotation.set(link.rotation[0], link.rotation[1], link.rotation[2]);
    }
    ImGui.PopID();
  })

  if(ImGui.SliderFloat3("target", target, -10, 10)){
    if(targetObject){
      targetObject.position.set(target[0], target[1], target[2]);
    }
    const links = convertBonesToLinks(bones);
    ik.solve_jacobian_ik(links, target);
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
  {
    const geometry = new THREE.SphereGeometry(0.1);
    const material = new THREE.MeshStandardMaterial({color: 0xFFFFFF});
    targetObject = new THREE.Mesh(geometry, material);
    targetObject.position.set(target[0], target[1], target[2]);
    scene.add(targetObject);
  }

  // -----------------------------------------
  // 組み立て
  // -----------------------------------------
  const build = (link, parent)=>{
    const geometry = new THREE.CylinderGeometry( 0, 0.1, 1);
    geometry.vertices.forEach(e=>e.y+=0.5);
    const material = new THREE.MeshStandardMaterial({color: 0xFFC107});
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(link.offset[0], link.offset[1], link.offset[2]);
    mesh.rotation.set(link.rotation[0], link.rotation[1], link.rotation[2]);
    parent.object.add(mesh);

    link.object = mesh;
    link.index = bones.length;
    link.parentIndex = parent.index;
    bones.push(link);

    link.children.forEach(e=> build(e, link));
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
