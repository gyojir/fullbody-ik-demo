export const range = num => [...Array(num).keys()];

export const zip = (arr1, arr2) => arr1.map((k, i) => [k, arr2[i]]);

export const clamp = (a, max, min) => Math.min(Math.max(a, min), max);

export const rotWrap = (rot) => {
  return (rot > Math.PI) ? rot - Math.PI * 2 :
         (rot < Math.PI * -1) ? rot + Math.PI * 2 : rot;
};