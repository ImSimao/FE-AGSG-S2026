// Small seeded PRNG (mulberry32) so a track can be reproduced exactly:
// same seed -> same sign layout and same starting direction.

let state = 0x9e3779b9;

export function setSeed(seed) {
  state = (Number.isFinite(seed) ? seed : Date.now()) >>> 0;
  return state;
}

export function getSeed() {
  return state;
}

export function newSeed() {
  return setSeed((Math.random() * 0xffffffff) >>> 0);
}

export function rngNext() {
  state = (state + 0x6d2b79f5) >>> 0;
  let t = state;
  t = Math.imul(t ^ (t >>> 15), t | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}

export function rngChance(p) {
  return rngNext() < p;
}
