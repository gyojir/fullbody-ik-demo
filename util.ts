import * as math from "mathjs";
import * as ImGui from './node_modules/imgui-js/imgui.js';
import * as ImGui_Impl from './node_modules/imgui-js/example/imgui_impl.js';

export const existFilter = <T>(x: T|undefined|null): x is T => x !== null || x !== undefined;

export const range = (num: number) => [...Array(num).keys()];

export const zip = <T,U>(arr1: T[], arr2: U[]): [T,U][] => arr1.map((k, i) => [k, arr2[i]]);

export const clamp = (a: number, max: number, min: number) => Math.min(Math.max(a, min), max);

// 回転の絶対値を 0~π に抑える
export const rotWrap = (rot: number) => {
  return (rot > Math.PI) ? rot - Math.PI * 2 :
         (rot < Math.PI * -1) ? rot + Math.PI * 2 : rot;
};

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
 * ImGui初期化
 */
export async function initImGui(canvas: HTMLCanvasElement) {
  await ImGui.default();
  ImGui.IMGUI_CHECKVERSION();
  ImGui.CreateContext();
  ImGui_Impl.Init(canvas);

  canvas.addEventListener( 'mousedown', event=>{
    // 逆っぽいけど
    if(!ImGui.IsWindowHovered()){
      event.stopImmediatePropagation();
    }
  }, false );
  canvas.addEventListener( 'touchstart', event=>{
    if(!ImGui.IsWindowHovered()){
      event.stopImmediatePropagation();
    }
  }, false );
}

/**
 * ImGuiフレーム表示開始
 * @param time 時間
 */
export function beginImGui(time: number){
  ImGui_Impl.NewFrame(time);
  ImGui.NewFrame();
}

/**
 * ImGuiフレーム表示終了
 */
export function endImGui(){
  ImGui.EndFrame();
  ImGui.Render();
  ImGui_Impl.RenderDrawData(ImGui.GetDrawData());
}