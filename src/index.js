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
    use
} = Matter;

use(MatterAttractors);

const DEFAULT_WIND = {x: 0, y: 0};
const SILHOUETTE = [2, 4, 5, 5, 3, 0];
const STRETCH = 7;

const MAX_FORCE = 0.025;

const shape = [
    ...SILHOUETTE
        .map((value, i) => ({
            x: i * STRETCH,
            y: value
        })),
    ...SILHOUETTE
        .reverse()
        .map((value, i) => ({
            x: (SILHOUETTE.length - 1 - i) * STRETCH,
            y: -value
        }))
];

const createSimulation = ({element = document.body, wind = DEFAULT_WIND, loop, width = 1000, height = 1000} = {}) => {

    const runner = Runner.create();
    const world = World.create({gravity: {x: 0, y: 0, scale: 0}});
    const engine = Engine.create({world});
    const render = Render.create({
        element,
        engine,
        options: {
            width,
            height,
            showVelocity: true,
            //showAngleIndicator: true
        }
    });

    const boat = Bodies.fromVertices(width / 2 - SILHOUETTE.length / 2 * STRETCH, height / 2, shape, {
        density: 3.0,
        frictionAir: .01,
        plugin: {
            attractors: [
                (a, b) => {
                    Body.applyForce(a, a.position, wind);
                }
            ]
        }
    });

    const top = Bodies.rectangle(width/2, 5, width, 10, {isStatic: true});
    const bottom = Bodies.rectangle(width/2, height - 5, width, 10, {isStatic: true});
    const left = Bodies.rectangle(5, height / 2, 10, height, {isStatic: true});
    const right = Bodies.rectangle(width - 5, height / 2, 10, height, {isStatic: true});

    World.add(engine.world, [boat, bottom, top, left, right]);

    Engine.run(engine);
    Render.run(render);

    let rudder = 0, velocity = 0;
    const control = {
        setVelocity(value){
            if (value < -1.0 || value > 1.0) throw 'Invalid velocity value.';
            velocity = value;
        },
        setRudder(value){
            if (value < -1.0 || value > 1.0) throw 'Invalid rudder value.';
            rudder = value;
        }
    };

    const position = {
        getPosition(){
            const {x,y} = boat.position;
            return {longitude:x,latitude:y};
        },
        getVelocity(){
            const {x,y} = boat.velocity;
            return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
        },
        getHeading(){
            const degrees =  (boat.angle * 180 / Math.PI) % 360;
            const normalized = degrees < 0 ? Math.abs(degrees) :  360-degrees;
            return (450 - normalized) % 360;

        }
    };

    const map = {
        getFence(){
            return [
                {longitude: 10, latitude: 10},
                {longitude: width-10, latitude: height -10}];
        }
    };


    Events.on(runner, 'tick', () => {
        // call control loop
        if(typeof loop === 'function'){
            loop({control, position, map});
        }

        // transform changes to controller variables to forces and apply forces
        const {x: x1, y: y1} = boat.vertices[0];
        const {x: x2, y: y2} = boat.vertices[boat.vertices.length-1];
        const propellerPosition = {
            x: x1 > x2? x2 + (x1-x2)/2: x1+ (x2-x1)/2,
            y: y1 > y2? y2 + (y1-y2)/2: y1+ (y2-y1)/2
        }

        const angle = boat.angle + ((Math.PI / 2) * rudder);
        const force = MAX_FORCE * velocity;
        Body.applyForce(boat, propellerPosition, {
            x: Math.cos(angle) * force,
            y: Math.sin(angle) * force
        });

        Body.applyForce(boat, propellerPosition /*boat.position*/, {
            x: Math.cos(boat.angle + (rudder < 0 ? 1 : -1) * Math.PI / 2) * force * 0.8 * Math.abs(rudder),
            y: Math.sin(boat.angle + (rudder < 0 ? 1 : -1) * Math.PI / 2) * force * 0.8 * Math.abs(rudder)
        });
    })

    return {
        start() {
            Runner.start(runner, engine);
        },
        stop() {
            Runner.stop(runner);
        }
    }
}

export {createSimulation};

if(typeof global === 'object'){
    global.createSimulation = createSimulation;
    global.decomp = decomp; // actually a peer dep
} else if(typeof window === 'object'){
    window.createSimulation = createSimulation;
    window.decomp = decomp;// actually a peer dep
}


