function sph2cart(rho, theta, phi) {
    return new THREE.Vector3(
        rho * Math.cos(theta) * Math.sin(phi),
        rho * Math.sin(theta) * Math.sin(phi),
        rho * Math.cos(phi)
    );
}

function equator(rho, t) {
    return new THREE.Vector3(
        rho * Math.cos(2 * Math.PI * t),
        rho * Math.sin(2 * Math.PI * t),
        0
    );
}

// construction of the key points on the sphere
const keyPoints = new Array(6);
const keyBalls = new Array(6);
const redmat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
var phi = 1;
for (let i = 0; i < 6; i++) {
    const theta = (i+1) * (2 * Math.PI) / 6;
    const keyPoint = sph2cart(5, theta, phi);
    keyPoints[i] = keyPoint.clone();
    const x = keyPoint.x;
    const y = keyPoint.y;
    const z = keyPoint.z;
    const geom = new THREE.SphereGeometry(0.25).translate(x, y, z);
    keyBalls[i] = new THREE.Mesh(geom, redmat);
    phi = Math.PI - phi;
}
const startingPoint = keyPoints[0].clone();

// construction of the key rotors; the first key rotor is the identity
//   quaternion and rotor i sends the key point 1 to the key point i
const keyRotors = new Array(6);
var rotor = new THREE.Quaternion(0, 0, 0, 1);
keyRotors[0] = rotor.clone();
const startingRotor = rotor.clone();
for(let i = 0; i < 5; i++){
    const point1 = keyPoints[i].clone();
    const point2 = keyPoints[i+1].clone();
    keyRotors[i+1] = new THREE.Quaternion().setFromUnitVectors(point1.divideScalar(5), point2.divideScalar(5));
    keyRotors[i+1].multiply(keyRotors[i]);
}

const yellowTimes = new Array(121);
for(let i = 0; i < 121; i++){
    yellowTimes[i] = 0.05 * i;
}

function Spline(t, c, b){
    const tcb = [t, c, b];
    const kb = KochanekBartels(keyRotors, seq_len(7), tcb);
    return kb;
}


function YellowPoints(t, c, b){
    const rotors = yellowTimes.map(function(time){return Spline(t, c, b)(time);});
    rotors.shift();
    const out = Array(rotors.length);
    for(let i = 0; i < rotors.length; i++){
        let p = startingPoint.clone();
        p.applyQuaternion(rotors[i]);
        out[i] = p;
    }
    return out;
}


const yellowmat = new THREE.MeshBasicMaterial({ color: 0xffff00 });

function addYellowBalls(group, t, c, b){
    const yellowPoints = YellowPoints(t,c,b);
    for(pt of yellowPoints){
        const geom = new THREE.SphereGeometry(0.15).translate(pt.x, pt.y, pt.z);
        group.add(new THREE.Mesh(geom, yellowmat));
    }
}


function CatmullRom(t, c, b){
    const yellowPoints = YellowPoints(t,c,b);
    return new THREE.CatmullRomCurve3(yellowPoints, true); 
}

function Hopf0(u, v, catmull) {
    // both u and v run from zero to one in Three.js
    const s = u;
    const phi = 2 * Math.PI * v;
    const p = catmull.getPoint(s);
    const p1 = p.x / 5;
    const p2 = p.y / 5;
    const p3 = p.z / 5;
    const cosphi = Math.cos(phi);
    const sinphi = Math.sin(phi);
    const yden = Math.sqrt(2 * (1 + p3));
    const x4 = (1 + p3) * cosphi / yden;
    const x2 = (p1 * sinphi - p2 * cosphi) / yden;
    const x3 = (p1 * cosphi + p2 * sinphi) / yden;
    const x1 = (1 + p3) * sinphi / yden;
    return new THREE.Vector3(2*x1 / (1 - x4), 2*x2 / (1 - x4), 2*x3 / (1 - x4));
}
function Hopf(catmull) {
    return function f(u, v, vector) {
        var out = Hopf0(u, v, catmull);
        vector.set(out.x, out.y, out.z);
    }
}