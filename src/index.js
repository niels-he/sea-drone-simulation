import Matter from 'matter-js';
import MatterAttractors from 'matter-attractors';
import decomp from 'poly-decomp';

const {
  Engine,
  Render,
  World,
  Bodies,
  Body,
  Runner,
  Events,
  Vector,
  use
} = Matter;

use(MatterAttractors);

const DEFAULT_WIND = { x: 0.0, y: 0 };

const BOAT_SILHOUETTE = [20, 20];
const BOTTLE_SILHOUETTE = [2, 2, 2, 2];

const STRETCH = 60;

const MAX_FORCE = 0.025;

const shapeFromSilhouette = (samples, stretch) => [
  ...samples.map((value, i) => ({
    x: i * stretch,
    y: value
  })),
  ...samples.reverse().map((value, i) => ({
    x: (samples.length - 1 - i) * stretch,
    y: -value
  }))
];

const normalizeAngle = angle => {
  const degrees = ((angle * 180) / Math.PI) % 360;
  const normalized = degrees < 0 ? Math.abs(degrees) : 360 - degrees;
  return (450 - normalized) % 360;
};

const boatShape = shapeFromSilhouette(BOAT_SILHOUETTE, STRETCH);
const bottleShape = shapeFromSilhouette(BOTTLE_SILHOUETTE, 5);

/**
 * Create a new simulation.
 * @param {function} loop - Function representing the main control loop
 * @param {DOMElement} element - DOM element to render the simulation canvas to. Default: document.body
 * @param {Object} wind - a vector {x:number, y:number}
 * @param {number} width - Width of the canvas
 * @param {number} height - Height of the canvas
 * @param {number} numberOfBottles - Number of bottles in the simulation
 * @param {number} detectorAngle - Expected detection angle of the waste detector <code>-detectorAngle  <= x <= detectorAngle</code>. Default: 45°
 * @param {number} detectorRange - Expected range of the detector. Default: 80
 * @returns {*}
 */
const createSimulation = ({
  loop,
  element = document.body,
  wind = DEFAULT_WIND,
  width = 1000,
  height = 1000,
  numberOfBottles = 3,
  detectorAngle = 45,
  detectorRange = 80
} = {}) => {
  const EngineWithCustomTimeScale = Engine.timing.timeScale(5);
  const runner = Runner.create();
  const world = World.create({ gravity: { x: 0, y: 0, scale: 0 } });
  const engine = EngineWithCustomTimeScale.create({ world });
  const render = Render.create({
    element,
    engine,
    options: {
      width,
      height,
      showVelocity: true
      //showAngleIndicator: true
    }
  });

  const bodyProps = {
    frictionAir: 0.01,
    friction: 0.01,
    plugin: {
      attractors: [
        (a, b) => {
          Body.applyForce(a, a.position, wind);
        }
      ]
    }
  };

  const boat = Bodies.fromVertices(
    width / 2 - (BOAT_SILHOUETTE.length / 2) * STRETCH,
    height / 2,
    boatShape,
    {
      ...bodyProps,
      density: 5.0
    }
  );

  let bottles = [];
  for (let i = 0; i < numberOfBottles; i++) {
    bottles.push(
      Bodies.fromVertices(
        20 + (width - 30) * Math.random(),
        20 + (height - 30) * Math.random(),
        bottleShape,
        {
          ...bodyProps,
          density: 0.3,
          angle: Math.random() * 2 * Math.PI
        }
      )
    );
  }

  const top = Bodies.rectangle(width / 2, 5, width, 10, { isStatic: true });
  const bottom = Bodies.rectangle(width / 2, height - 5, width, 10, {
    isStatic: true
  });
  const left = Bodies.rectangle(5, height / 2, 10, height, { isStatic: true });
  const right = Bodies.rectangle(width - 5, height / 2, 10, height, {
    isStatic: true
  });

  World.add(engine.world, [boat, ...bottles, bottom, top, left, right]);

  EngineWithCustomTimeScale.run(engine);
  Render.run(render);

  let velocityLeft = 0;
  let velocityRight = 0;

  const control = {
    setVelocityLeft(value) {
      if (value < -1.0 || value > 1.0) throw 'Invalid velocity value.';
      velocityLeft = value * 4;
    },
    setVelocityRight(value) {
      if (value < -1.0 || value > 1.0) throw 'Invalid velocity value.';
      velocityRight = value * 4;
    }
  };

  const position = {
    getPosition() {
      const { x, y } = boat.position;
      return { longitude: x, latitude: y };
    },
    getVelocityLeft() {
      const { x, y } = boat.velocityLeft;
      return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    },
    getVelocityRight() {
      const { x, y } = boat.velocityRight;
      return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    },
    getHeading() {
      return normalizeAngle(boat.angle);
    }
  };

  const detector = {
    getDetectedWaste: () =>
      bottles
        .map(bottle => {
          const distanceVector = Vector.sub(boat.position, bottle.position);
          const distance = Vector.magnitude(distanceVector);

          const wasteAngle = normalizeAngle(
            Vector.angle(boat.position, bottle.position)
          );
          const boatAngle = normalizeAngle(boat.angle);

          const angle = wasteAngle - boatAngle;
          return { angle, distance };
        })
        .filter(
          ({ angle, distance }) =>
            distance < detectorRange && Math.abs(angle) < detectorAngle
        )
        .map(({ angle, distance }) => ({
          direction: angle,
          proximity: distance < detectorRange / 2 ? 0 : 1
        }))
  };

  const map = {
    getFence() {
      return [
        { longitude: 10, latitude: 10 },
        { longitude: width - 10, latitude: height - 10 }
      ];
    }
  };

  let weight = 0;
  const container = {
    getWeight: () => weight
  };

  Events.on(runner, 'tick', () => {
    bottles = bottles.filter(bottle => {
      const { collided } = Matter.SAT.collides(bottle, boat);

      if (collided) {
        weight++;
        World.remove(world, bottle);
        return false;
      }

      return true;
    });

    // call control loop
    if (typeof loop === 'function') {
      loop({
        control,
        position,
        map,
        detector,
        container
      });
    }

    // transform changes to controller variables to forces and apply forces
    const { x: x1, y: y1 } = boat.vertices[0];
    const { x: x2, y: y2 } = boat.vertices[boat.vertices.length - 1];

    const propellerPositionOne = {
      x: x1, // > x2 ? x2 + (x1 - x2) / 2 : x1 + (x2 - x1) / 2,
      y: y1
    };
    const propellerPositionTwo = {
      x: x2, // x1 > x2 ? x2 + (x1 - x2) / 2 : x1 + (x2 - x1) / 2,
      y: y2
    };

    const forceLeft = MAX_FORCE * velocityLeft;
    const forceRight = MAX_FORCE * velocityRight;

    Body.applyForce(boat, propellerPositionOne, {
      x: Math.cos(boat.angle) * forceLeft,
      y: Math.sin(boat.angle) * forceLeft
    });

    Body.applyForce(boat, propellerPositionTwo, {
      x: Math.cos(boat.angle) * forceRight,
      y: Math.sin(boat.angle) * forceRight
    });
  });

  return {
    start() {
      Runner.start(runner, engine);
    },
    stop() {
      Runner.stop(runner);
    }
  };
};

export { createSimulation };

if (typeof global === 'object') {
  global.createSimulation = createSimulation;
  global.decomp = decomp; // actually a peer dep
} else if (typeof window === 'object') {
  window.createSimulation = createSimulation;
  window.decomp = decomp; // actually a peer dep
}
