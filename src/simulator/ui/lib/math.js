export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

export function rotateDir(dir, angleRad) {
  const cos = Math.cos(angleRad);
  const sin = Math.sin(angleRad);
  return { x: dir.x * cos - dir.y * sin, y: dir.x * sin + dir.y * cos };
}

