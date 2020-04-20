export const range = (num: number) => [...Array(num).keys()];

export const zip = <T,U>(arr1: T[], arr2: U[]): [T,U][] => arr1.map((k, i) => [k, arr2[i]]);

export const clamp = (a: number, max: number, min: number) => Math.min(Math.max(a, min), max);

// 回転の絶対値を 0~π に抑える
export const rotWrap = (rot: number) => {
  return (rot > Math.PI) ? rot - Math.PI * 2 :
         (rot < Math.PI * -1) ? rot + Math.PI * 2 : rot;
};