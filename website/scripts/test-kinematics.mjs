// Test of the Control page's forward kinematics (src/control/kinematics.ts) against src/kinematics.jl: the
// flange poses in tools/firmware-tests/fk_reference.h (made by tools/firmware-tests/fk_reference.jl), which the
// firmware's FK test (test_twist_check.cpp) uses too.
//
//   node scripts/test-kinematics.mjs        (Node 22.18+ runs the TypeScript file directly)
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { parseChain } from '../src/control/kinematics.ts';

const here = (p) => fileURLToPath(new URL(p, import.meta.url));
const chain = parseChain(readFileSync(here('../public/robot/mycobot_280_arduino.urdf'), 'utf8'));
const ref = readFileSync(here('../../tools/firmware-tests/fk_reference.h'), 'utf8');
const table = (name) =>
  [...ref.match(new RegExp(`${name}\\[\\]\\[\\d\\] = \\{(.*)\\};`))[1].matchAll(/\{([^}]*)\}/g)].map((m) => m[1].split(',').map((x) => parseFloat(x)));
const Q = table('FK_REF_Q'), P = table('FK_REF_P'), R = table('FK_REF_R');
let worstP = 0, worstR = 0;
Q.forEach((q, i) => {
  const { pose } = chain.fk(q);
  pose.p.forEach((x, k) => (worstP = Math.max(worstP, Math.abs(x - P[i][k]))));
  pose.R.forEach((x, k) => (worstR = Math.max(worstR, Math.abs(x - R[i][k]))));
});
const ok = chain.dof === 6 && Q.length >= 5 && worstP < 1e-3 && worstR < 1e-6;
console.log(`${ok ? 'ok  ' : 'FAIL'} FK = src/kinematics.jl on ${Q.length} poses: ${worstP.toExponential(1)} mm, ${worstR.toExponential(1)}`);
process.exit(ok ? 0 : 1);
