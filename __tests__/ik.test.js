import {toBeDeepCloseTo,toMatchCloseTo} from 'jest-matcher-deep-close-to';
import * as ik from '../ik';
import * as math from "mathjs";

expect.extend({toBeDeepCloseTo, toMatchCloseTo});

test('getRotationRodorigues', ()=>{
    expect(ik.getRotationRodorigues([0,0,1], Math.PI/2).toArray()).toBeDeepCloseTo([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]], 3);
})

test('getRotationSpherical', ()=>{
    const ret = ik.getRotationSpherical(ik.mul(ik.rotZ(Math.PI/4),ik.rotX(Math.PI/2)), ik.mul(ik.rotZ(Math.PI/4),ik.rotX(Math.PI/4)));
    console.log(ret);
    // expect(.toArray()).;
})