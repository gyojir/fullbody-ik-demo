import {toBeDeepCloseTo,toMatchCloseTo} from 'jest-matcher-deep-close-to';
import * as mutil from '../math-util';
import * as math from "mathjs";

expect.extend({toBeDeepCloseTo, toMatchCloseTo});

test('getRotationFromAxis', ()=>{
    expect(mutil.getRotationFromAxis([0,0,1], Math.PI/2).toArray()).toBeDeepCloseTo([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]], 3);
})

test('getRotationSpherical', ()=>{
    const ret = mutil.getRotationSpherical(mutil.mul(mutil.rotZ(Math.PI/4),mutil.rotX(Math.PI/2)), mutil.mul(mutil.rotZ(Math.PI/4),mutil.rotX(Math.PI/4)));
    // expect(.toArray()).;
})
test('swapMatrix', ()=>{
    expect(mutil.getSwapMatrix(0,1,2).toArray()).toBeDeepCloseTo([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], 3);
    expect(mutil.getSwapMatrix(2,0,1).toArray()).toBeDeepCloseTo([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]], 3);
})
test('world matrix', ()=>{
    expect(mutil.getScale(mutil.mul(mutil.rotX(1), mutil.scale(2,2,2)))).toBeDeepCloseTo([2,2,2], 3);
    expect(mutil.getRotationXYZ(mutil.mul(mutil.rotX(Math.PI/4), mutil.rotY(Math.PI/5),mutil.rotZ(Math.PI/6),mutil.scale(2,2,2)))).toBeDeepCloseTo([Math.PI/4,Math.PI/5,Math.PI/6], 3);
})
test('getRotationError', ()=>{
    const a = [Math.PI/4, Math.PI/5, Math.PI/6];
    const b = [Math.PI/6, Math.PI/5, Math.PI/2];
    const A = mutil.mul(mutil.rotXYZ(...a));
    const B = mutil.mul(mutil.rotXYZ(...b));
    const diff = mutil.mul(math.transpose(B), A);
    const e = mutil.getRotationError(A, B);
    const rot = mutil.getRotationFromAxis(mutil.normalize(e), Math.asin(math.norm(e)) * 2);
    console.log(A.toArray());
    console.log(b, e);
    console.log(mutil.rotXYZ(...math.add(b, e)).toArray());
});