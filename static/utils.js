function hslToHex(h, s, l) {
    l /= 100;
    const a = s * Math.min(l, 1 - l) / 100;
    const f = n => {
        const k = (n + h / 30) % 12;
        const color = l - a * Math.max(Math.min(k - 3, 9 - k, 1), -1);
        return Math.round(255 * color).toString(16).padStart(2, '0');   // convert to Hex and prefix "0" if needed
    };
    return `#${f(0)}${f(8)}${f(4)}`;
}


function get_distinguishable_colors(n_of_colors) {
    if (n_of_colors == 0) {
        return [];
    }
    let step = 360 / n_of_colors;
    let res = [];
    for (let i = 0; i < n_of_colors; ++i) {
        res.push(hslToHex(step * i, 100, 50));
    }
    return res;
}

function get_rotation(x1, y1, x2, y2, angle) {
    c = Math.cos(angle);
    s = Math.sin(angle);
    let x1new = x1 * c - y1 * s;
    let y1new = x1 * s + y1 * c;
    let x2new = x2 * c - y2 * s;
    let y2new = x2 * s + y2 * c;

    let center_x = (x2new + x1new) / 2;
    let center_y = (y2new + y1new) / 2;

    console.log(x1new, y1new, x2new, y2new);

    let movement_x = 50 - center_x;
    let movement_y = 50 - center_y;
    return {x1: x1new + movement_x, y1: y1new + movement_y, x2: x2new + movement_x, y2: y2new + movement_y};
}


function get_available_services() {
    let req = new Request('/get_services', {
        method: 'GET'
    });
    fetch(req).then(response => {
        if (response.status != 200) {
            alert("Error while receiving valid services");
            throw new Error();
        } else {
            return response.json();
        }
    }).then(data => {
        console.log(data);
        update_falling_list_values(data.valid_services.concat([""]));
    }).catch(er => {
        alert("Error while transforming valid services data")
    });
}

function start_following_paths() {
    let request = [];
    for (let i = 0; i < uavs_num; ++i) {
        let selection = document.getElementById(`path-${i}-target`);
        request.push([i, selection.value]);
    }
    let req = new Request('/load_paths', {
        method: 'POST',
        body: JSON.stringify({'uav_topic': request})
    });
    console.log("Sending " + req.body);

    fetch(req).then(response => {
        if (response.status != 200) {
            alert("Error. Invalid response from server");
            throw new Error();
        }
        return response.json();
    }).then(data => {
        console.log(data);
        alert(data.map(el => String(el)).join("\n"));

    }).catch(er => {
        console.log(er);
    });
}