// Default traffic sign positions (cm) converted to meters; duplicates removed.
const rawSignCoordsCm = [
  // Bottom Lane
  [100, 60],
  [150, 60],
  [200, 60],
  // Left Lane
  [40, 100],
  [40, 150],
  [40, 200],
  [60, 100],
  [60, 150],
  [60, 200],
  // Top Lane
  [100, 240],
  [150, 240],
  [200, 240],
  [100, 260],
  [150, 260],
  [200, 260],
  // Right Lane
  [240, 100],
  [240, 150],
  [240, 200],
  [260, 100],
  [260, 150],
  [260, 200],
];

export const uniqueSignCoords = Array.from(new Set(rawSignCoordsCm.map(([x, y]) => `${x},${y}`)))
  .map((key) => key.split(",").map((n) => Number(n)));

export const trafficSignLanes = {
  bottom: uniqueSignCoords
    .filter(([_, y]) => y === 60)
    .map(([x, y]) => ({ x: x / 100, y: y / 100 })),
  left: uniqueSignCoords
    .filter(([x, _]) => x === 40 || x === 60)
    .map(([x, y]) => ({ x: x / 100, y: y / 100 })),
  top: uniqueSignCoords
    .filter(([_, y]) => y === 240 || y === 260)
    .map(([x, y]) => ({ x: x / 100, y: y / 100 })),
  right: uniqueSignCoords
    .filter(([x, _]) => x === 240 || x === 260)
    .map(([x, y]) => ({ x: x / 100, y: y / 100 })),
};

export const trafficSigns = uniqueSignCoords.map(([x, y]) => ({
  x: x / 100,
  y: y / 100,
  dx: 0, // displacement from the seat, from being pushed (rule 13.14)
  dy: 0,
  state: 0, // 0=empty,1=green,2=red
  modified: false, // Surprise Rule: opposite visible colour + 5/15 cm height
  heightCm: 10,
  lane: (() => {
    if (y === 60) return "bottom";
    if (y === 240 || y === 260) return "top";
    if (x === 40 || x === 60) return "left";
    if (x === 240 || x === 260) return "right";
    return "unknown";
  })(),
}));

// Lane bounds (meters) for side-checking rules
export const laneBounds = {
  left: { x0: 0, x1: 1, y0: 1, y1: 2 },
  bottom: { x0: 1, x1: 2, y0: 0, y1: 1 },
  top: { x0: 1, x1: 2, y0: 2, y1: 3 },
  right: { x0: 2, x1: 3, y0: 1, y1: 2 },
};

