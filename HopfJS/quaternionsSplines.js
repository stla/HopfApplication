function bisect_right(a, x) {
    var lo = 0;
    var hi = a.length;
    do {
        const mid = (lo + hi) >>> 1;
        if (a[mid] - x <= 0) lo = mid + 1;
        else hi = mid;
    } while (lo < hi);
    return lo;
}


function quatmultiply(q, scalar) {
    return new THREE.Quaternion(scalar * q.x, scalar * q.y, scalar * q.z, scalar * q.w);
}


function quatdivide(q, scalar) {
    return new THREE.Quaternion(q.x / scalar, q.y / scalar, q.z / scalar, q.w / scalar);
}


function quatlog(q) {
    const qnorm = q.length();
    const w = q.w;
    let v = q.clone();
    v.w = 0;
    const vnorm = v.length();
    if (vnorm == 0) {
        return new THREE.Quaternion(0, 0, 0, Math.log(qnorm));
    }
    const a = Math.acos(w / qnorm) / vnorm;
    return new THREE.Quaternion(a * q.x, a * q.y, a * q.z, Math.log(qnorm));
}


function quatexp(q) {
    const w = q.w;
    let v = q.clone();
    v.w = 0;
    const vnorm = v.length();
    if (vnorm == 0) {
        return new THREE.Quaternion(0, 0, 0, Math.exp(w));
    }
    const b = Math.exp(w) * Math.sin(vnorm) / vnorm;
    return new THREE.Quaternion(b * q.x, b * q.y, b * q.z, Math.exp(w) * Math.cos(vnorm));
}


function quatpower(q, alpha) {
    return quatexp(quatmultiply(quatlog(q), alpha));
}


function _select_segment_and_normalize_t(segments, keyTimes, t) {
    const lastTime = keyTimes[keyTimes.length - 1];
    let idx;
    if (t < keyTimes[0]) {
        throw "`t` is too small."
    } else if (t < lastTime) {
        idx = bisect_right(keyTimes, t) - 1;
    } else if (t == lastTime) {
        idx = keyTimes.length - 2;
    } else {
        throw "`t` is too big.";
    }
    const t0 = keyTimes[idx];
    const t1 = keyTimes[idx+1];
    const delta_t = t1 - t0;
    return {
        "segment": segments[idx],
        "time": (t - t0) / delta_t,
        "difftime": delta_t
    };
}


function _reduce_de_casteljau(segment, t) {
    let l = segment.length;
    if (l < 2) {
        throw "Segment must have at least two quaternions.";
    }
    while (l > 2) {
        let newsegment = new Array(l - 1);
        for (let i = 0; i < l - 1; i++) {
            let q = new THREE.Quaternion();
            q.slerpQuaternions(segment[i], segment[i + 1], t);
            newsegment[i] = q.clone();
        }
        segment = newsegment.map(function(qq){return qq.clone();});
        l = segment.length;
    }
    return segment;
}


function seq_len(n) {
    let seq = new Array(n);
    for (let i = 0; i < n; i++) {
        seq[i] = i;
    }
    return seq;
}


function DeCasteljau(segments, keyTimes) {
    const n_segments = segments.length;
    if (typeof (keyTimes) == "undefined") {
        keyTimes = seq_len(n_segments + 1);
    } else if (keyTimes.length != n_segments + 1) {
        throw "Number of key times must be one more than number of segments.";
    }
    const evaluate = function (t) {
        const x = _select_segment_and_normalize_t(segments, keyTimes, t);
        const segment = x["segment"];
        const s = x["time"];
        const quats = _reduce_de_casteljau(segment, s);
        let q = new THREE.Quaternion();
        q.slerpQuaternions(quats[0], quats[1], s);
        return q;
    }
    return evaluate;
}



function _calculate_control_quaternions(quaternions, times, tcb) {
    const q_1 = quaternions[0].clone();
    const q0 = quaternions[1].clone();
    const q00 = quaternions[1].clone();
    const q000 = quaternions[1].clone();
    const q1 = quaternions[2].clone();
    const t_1 = times[0];
    const t0 = times[1];
    const t1 = times[2];
    const T = tcb[0];
    const C = tcb[1];
    const B = tcb[2];
    const a = (1 - T) * (1 + C) * (1 + B);
    const b = (1 - T) * (1 - C) * (1 - B);
    const c = (1 - T) * (1 - C) * (1 + B);
    const d = (1 - T) * (1 + C) * (1 - B);
    const q_in = q0.multiply(q_1.invert());
    const q_out = q1.multiply(q00.invert());
    let v_in = quatdivide(quatlog(q_in), t0 - t_1);
    let v_out = quatdivide(quatlog(q_out), t1 - t0);
    // const V0 = function (weight_in, weight_out) {
    //     const va = quatmultiply(v_in, weight_in * (t1 - t0));
    //     const vb = quatmultiply(v_out, weight_out * (t0 - t_1));
    //     const vavb = new THREE.Quaternion(va.x + vb.x, va.y + vb.y, va.z + vb.z, va.w + vb.w); 
    //     return quatdivide(vavb, t1 - t_1);
    // }
    // let out = new Array(2);
    // const qcd = quatmultiply(V0(c, d), -(t0 - t_1) / 3);
    // out[0] = quatexp(qcd).multiply(q00);
    // const qab = quatmultiply(V0(a, b), (t1 - t0) / 3);
    // out[1] = quatexp(qab).multiply(q00);
    v_in = new THREE.Vector3(v_in.x, v_in.y, v_in.z);
    v_out = new THREE.Vector3(v_out.x, v_out.y, v_out.z);
    const V0 = function (weight_in, weight_out) {
        const vin = v_in.clone();
        const vout = v_out.clone();
        const va = vin.multiplyScalar(weight_in * (t1 - t0));
        const vb = vout.multiplyScalar(weight_out * (t0 - t_1));
        return (va.add(vb)).divideScalar(t1 - t_1);
    }
    let out = new Array(2);
    const v0cd = V0(c, d).multiplyScalar((t0 - t_1) / 3);
    const qcd = new THREE.Quaternion(-v0cd.x, -v0cd.y, -v0cd.z, 0);
    out[0] = quatexp(qcd).multiply(q000);
    const v0ab = V0(a, b).multiplyScalar((t1 - t0) / 3);
    const qab = new THREE.Quaternion(v0ab.x, v0ab.y, v0ab.z, 0);
    out[1] = quatexp(qab).multiply(q000);
    return out;
}


function _check_endcondition(endcondition, rotors, times) {
    const n_rotors = rotors.length;
    const n_times = times.length;
    let triples_times;
    let triples_rotors;
    if (endcondition == "closed") {
        let prefix = rotors[n_rotors - 2].clone();
        if (prefix.dot(rotors[0]) < 0) {
            prefix = quatmultiply(prefix, -1);
        }
        let suffix = rotors[1].clone();
        if (rotors[n_rotors - 1].dot(suffix) < 0) {
            suffix = quatmultiply(suffix, -1);
        }
        rotors = [prefix].concat(rotors).concat([suffix]);
        times =
            [times[0] - (times[n_times - 1] - times[n_times - 2])]
                .concat(times)
                .concat([times[n_times - 1] + (times[1] - times[0])]);
        triples_times = seq_len(n_times).map(function (i) { return [times[i], times[i + 1], times[i + 2]] });
        triples_rotors = seq_len(n_rotors).map(function (i) { return [rotors[i], rotors[i + 1], rotors[i + 2]] });
    } else {
        triples_times = seq_len(n_times).map(function (i) { return [times[i], times[i + 1], times[i + 2]] });
        triples_rotors = seq_len(n_rotors - 2).map(function (i) { return [rotors[i], rotors[i + 1], rotors[i + 2]] });
    }
    return {
        "times": triples_times,
        "rotors": triples_rotors
    };
}


// function _natural_control_quaternion(outer, inner_control, inner) {
//     return quatpower(
//         inner_control.multiply(outer.invert()),
//         0.5
//     ).multiply(outer);
// }


function _canonicalized(quaternions) {
    const n_quaternions = quaternions.length;
    let out = new Array(n_quaternions);
    let p = new THREE.Quaternion(0, 0, 0, 1);
    for (let i = 0; i < n_quaternions; i++) {
        let q = quaternions[i].clone();
        if (p.dot(q) < 0) {
            q = quatmultiply(q, -1);
        }
        out[i] = q.clone();
        p = q.clone();
    }
    return out;
}

function _check_keyRotors(keyRotors, closed) {
    if (keyRotors.length < 2) {
        throw "At least two key rotors are required.";
    }
    if (closed) {
        keyRotors = keyRotors.concat([keyRotors[0]]);
    }
    return _canonicalized(keyRotors);
}


function _check_keyTimes(keyTimes, n_quaternions) {
    if (typeof (keyTimes) === "undefined") {
        return seq_len(n_quaternions);
    }
    if (n_quaternions != keyTimes.length) {
        throw "Numbers of key times and key rotors do not match.";
    }
    let i = 1;
    while (i < keyTimes.length) {
        if (keyTimes[i] <= keyTimes[i - 1]) {
            throw "`keyTimes` must be an increasing vector of numbers.";
        }
        i += 1;
    }
    return keyTimes;
}

function KochanekBartels(keyRotors, keyTimes, tcb) {
    keyRotors = keyRotors.map(function(q){return q.clone();});
    keyRotors = _check_keyRotors(keyRotors, true);
    const n_keyRotors = keyRotors.length;
    keyTimes = _check_keyTimes(keyTimes, n_keyRotors)
    const triples = _check_endcondition("closed", keyRotors, keyTimes);
    const triples_rotors = triples["rotors"];
    const triples_times = triples["times"];
    let control_points = new Array(4*triples_rotors.length);
    for (let i = 0; i < triples_rotors.length; i++) {
        const qs = triples_rotors[i];
        const qb_qa = _calculate_control_quaternions(qs, triples_times[i], tcb);
        const q_before = qb_qa[0];
        const q_after = qb_qa[1];
        control_points[4*i] = q_before;
        control_points[4*i+1] = qs[1];
        control_points[4*i+2] = qs[1];
        control_points[4*i+3] = q_after;
    }
    const n_control_points = control_points.length;
    //    if(closed){
    //stopifnot(4*length(keyTimes) == n_control_points)
    control_points = control_points.slice(2, n_control_points - 2);
    // }else if(n_control_points == 0L){
    //   # two quaternions -> slerp
    //   stopifnot(n_keyRotors == 2L)
    //   stopifnot(length(keyTimes) == 2L)
    //   q0 <- keyRotors[1L]
    //   q1 <- keyRotors[2L]
    //   offset <- (q1 * onion_inverse(q0))^(1/3)
    //   control_points <- c(q0, offset*q0, onion_inverse(offset)*q1, q1)
    // }else{ # natural
    //   control_points <- c(
    //     keyRotors[1L],
    //     .natural_control_quaternion(
    //       keyRotors[1L], control_points[1L], control_points[2L]
    //     ),
    //     control_points,
    //     .natural_control_quaternion(
    //       keyRotors[n_keyRotors],
    //       control_points[n_control_points],
    //       control_points[n_control_points-1L]
    //     ),
    //     keyRotors[n_keyRotors]
    //   )
    // }
    const k = ~~(control_points.length / 4);
    let indices = new Array(k);
    for (let i = 0; i < k; i++) {
        indices[i] = 4 * i;
    }
    const segments = indices.map(function (i) { return control_points.slice(i, i + 4); });
    return DeCasteljau(segments, keyTimes);
}

