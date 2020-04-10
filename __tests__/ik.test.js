import {toBeDeepCloseTo,toMatchCloseTo} from 'jest-matcher-deep-close-to';
import * as ik from '../ik';
import * as math from "mathjs";

expect.extend({toBeDeepCloseTo, toMatchCloseTo});

test('getRotationFromAxis', ()=>{
    expect(ik.getRotationFromAxis([0,0,1], Math.PI/2).toArray()).toBeDeepCloseTo([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]], 3);
})

test('getRotationSpherical', ()=>{
    const ret = ik.getRotationSpherical(ik.mul(ik.rotZ(Math.PI/4),ik.rotX(Math.PI/2)), ik.mul(ik.rotZ(Math.PI/4),ik.rotX(Math.PI/4)));
    console.log(ret);
    // expect(.toArray()).;
})
test('swapMatrix', ()=>{
    expect(ik.getSwapMatrix(0,1,2).toArray()).toBeDeepCloseTo([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], 3);
    expect(ik.getSwapMatrix(2,0,1).toArray()).toBeDeepCloseTo([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]], 3);
})