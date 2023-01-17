const FLY_ZONE_COLOR = '#32CD32';
const NO_FLY_ZONE_COLOR = '#FF1433';
var drawingManager;
var all_overlays = [];
var all_paths = [];
var selectedShape;
var colors = [NO_FLY_ZONE_COLOR, FLY_ZONE_COLOR];
var selectedColor;
var colorButtons = {};
var map;
var uavs_num = 0;

function clearSelection() {
    if (selectedShape) {
        if (selectedShape.type == google.maps.drawing.OverlayType.MARKER) {
            return;
        }
        selectedShape.setEditable(false);
        selectedShape = null;
    }
}

function setSelection(shape) {
    clearSelection();
    selectedShape = shape;
    if (shape.type == google.maps.drawing.OverlayType.MARKER) {
        return;
    }
    shape.setEditable(true);
    selectColor(shape.get('fillColor') || shape.get('strokeColor'));
}

function deleteSelectedShape() {
    if (selectedShape) {
        selectedShape.setMap(null);
        all_overlays = all_overlays.filter(el => {
            return el.overlay !== selectedShape
        });
    }
}

function deleteAllShape() {
    for (var i = 0; i < all_overlays.length; i++) {
        all_overlays[i].overlay.setMap(null);
    }
    all_overlays = [];
}

function selectColor(color) {
    selectedColor = color;
    for (var i = 0; i < colors.length; ++i) {
        var currColor = colors[i];
        colorButtons[currColor].style.border = currColor == color ? '2px solid #789' : '2px solid #fff';
    }

    // Retrieves the current options from the drawing manager and replaces the
    // stroke or fill color as appropriate.
    var polylineOptions = drawingManager.get('polylineOptions');
    polylineOptions.strokeColor = color;
    drawingManager.set('polylineOptions', polylineOptions);

    var rectangleOptions = drawingManager.get('rectangleOptions');
    rectangleOptions.fillColor = color;
    drawingManager.set('rectangleOptions', rectangleOptions);

    var circleOptions = drawingManager.get('circleOptions');
    circleOptions.fillColor = color;
    drawingManager.set('circleOptions', circleOptions);

    var polygonOptions = drawingManager.get('polygonOptions');
    polygonOptions.fillColor = color;
    drawingManager.set('polygonOptions', polygonOptions);
}

function setSelectedShapeColor(color) {
    if (selectedShape) {
        if (selectedShape.type == google.maps.drawing.OverlayType.MARKER) {
            return;
        }
        if (selectedShape.type == google.maps.drawing.OverlayType.POLYLINE) {
            selectedShape.set('strokeColor', color);
        } else {
            selectedShape.set('fillColor', color);
        }
    }
}

function makeColorButton(color) {
    var button = document.createElement('span');
    button.className = 'color-button';
    button.style.backgroundColor = color;
    google.maps.event.addDomListener(button, 'click', function () {
        selectColor(color);
        setSelectedShapeColor(color);
    });

    return button;
}

function buildColorPalette() {
    var colorPalette = document.getElementById('color-palette');
    for (var i = 0; i < colors.length; ++i) {
        var currColor = colors[i];
        var colorButton = makeColorButton(currColor);
        colorPalette.appendChild(colorButton);
        colorButtons[currColor] = colorButton;
    }
    selectColor(colors[0]);
}

function initialize() {
    map = new google.maps.Map(document.getElementById('map'), {
        zoom: 15,
        center: new google.maps.LatLng(50.111874, 14.418484),
        // disableDefaultUI: true,
        zoomControl: true
    });

    var polyOptions = {
        strokeWeight: 0,
        fillOpacity: 0.2,
        editable: true
    };
    // Creates a drawing manager attached to the map that allows the user to draw
    // markers, lines, and shapes.
    drawingManager = new google.maps.drawing.DrawingManager({
        drawingMode: google.maps.drawing.OverlayType.POLYGON,
        drawingControlOptions: {
            drawingModes: [google.maps.drawing.OverlayType.POLYGON,
                google.maps.drawing.OverlayType.MARKER]
        },

        markerOptions: {
            draggable: true
        },
        polylineOptions: {
            editable: false
        },
        rectangleOptions: polyOptions,
        circleOptions: polyOptions,
        polygonOptions: polyOptions,
        map: map
    });


    // drawingManager.

    google.maps.event.addListener(drawingManager, 'overlaycomplete', function (e) {
        all_overlays.push(e);
        if (true) {
            // Switch back to non-drawing mode after drawing a shape.
            drawingManager.setDrawingMode(null);

            // Add an event listener that selects the newly-drawn shape when the user
            // mouses down on it.
            var newShape = e.overlay;
            newShape.type = e.type;
            google.maps.event.addListener(newShape, 'click', function () {
                setSelection(newShape);
            });
            setSelection(newShape);
        }
    });


    // Clear the current selection when the drawing mode is changed, or when the
    // map is clicked.
    google.maps.event.addListener(drawingManager, 'drawingmode_changed', clearSelection);
    google.maps.event.addListener(map, 'click', clearSelection);
    google.maps.event.addDomListener(document.getElementById('delete-button'), 'click', deleteSelectedShape);
    google.maps.event.addDomListener(document.getElementById('delete-all-button'), 'click', deleteAllShape);

    buildColorPalette();
    update_rotation_value();
}

google.maps.event.addDomListener(window, 'load', initialize);

// Own initialization



function update_rotation_value() {
    document.getElementById("rotation-value").innerHTML = document.getElementById("init-rotation").value;
    let rot = get_rotation(50, 25, 50, 75, parseFloat(document.getElementById("init-rotation").value));
    document.getElementById("rotation-line").innerHTML = `<line x1="${rot.x1}" y1="${rot.y1}" x2="${rot.x2}" y2="${rot.y2}" style="stroke:rgb(255,0,0);stroke-width:2" />`
}

function get_polygon_points(polygon) {
    let res = [];
    let points = polygon.getPath().getArray();
    for (let i = 0; i < points.length; ++i) {
        res.push([points[i].lat(), points[i].lng()]);
    }
    return res;
}

function clear_paths_from_map() {
    all_paths.forEach(el => {
        el.setMap(null)
    });
    all_paths = [];
}

function add_one_path(path, color) {
    let path_tr = path.map(point => {
        return {lat: point[1], lng: point[0]}
    });
    let coorindates = new google.maps.Polyline({
        path: path_tr,
        geodesic: true,
        strokeColor: color,
        strokeOpacity: 1.0,
        strokeWeight: 2,
    });
    coorindates.setMap(map);
    all_paths.push(coorindates);
}

function make_one_polygon(path, color) {
    let path_tr = path.map(point => {
        return {lat: point[0], lng: point[1]}
    });
    let polygon = new google.maps.Polygon({
        path: path_tr,
        editable: true,
        fillColor: color,
        fillOpacity: 0.2,
        type: 'polygon'
    });
    polygon.setMap(map);
    return polygon;
}

function make_marker(marker_pos) {
    let marker = new google.maps.Marker({
        // position: {lat: () => {return marker_pos[0]}, lng: () => {return marker_pos[1]}},
        position: {lat: marker_pos[0], lng: marker_pos[1]},
        draggable: true,
        type: google.maps.drawing.OverlayType.MARKER
    });
    marker.setMap(map);
    return marker;
}

function draw_paths_on_map(paths) {
    let colors = get_distinguishable_colors(paths.length);
    for (let i = 0; i < paths.length; ++i) {
        add_one_path(paths[i], colors[i]);
    }
}

var cons;

function update_paths_metrics(data) {
    cons = data.energies;
    let colors = get_distinguishable_colors(data.energies.length);
    let list = document.getElementById("power-consumptions");
    list.innerHTML = "";
    for (let i = 0; i < data.energies.length; ++i) {
        list.innerHTML += `<li> <div class="color-marker" style="background-color: ${colors[i]}"></div> ${data.energies[i].toFixed(2)}, ${data.times[i].toFixed(2)}, ${data.lengths[i].toFixed(1)}</li>`
    }
}

function get_start_point() {
    let markers_found = 0;
    let init_point;
    for (let i = 0; i < all_overlays.length; ++i) {
        let overlay = all_overlays[i].overlay;
        console.log(overlay);
        if (overlay.type == google.maps.drawing.OverlayType.MARKER) {
            if (markers_found != 0) {
                alert("There may be only one marker identifying the starting point...");
                return undefined;
            }
            markers_found++;
            init_point = [overlay.position.lat(), overlay.position.lng()];
        }
    }
    if (!markers_found) {
        return undefined;
    }
    return init_point;
}

function get_fly_zone() {
    let fly_zone;
    let fly_zone_found = false;

    for (let i = 0; i < all_overlays.length; ++i) {
        let overlay = all_overlays[i].overlay;
        console.log(overlay);
        if (overlay.type === google.maps.drawing.OverlayType.POLYGON) {
            if (overlay.fillColor === FLY_ZONE_COLOR) {
                if (fly_zone_found) {
                    alert("There may not be more than 1 fly zone");
                    return undefined;
                }
                fly_zone_found = true;
                fly_zone = get_polygon_points(overlay);
            }
        }
    }
    return fly_zone;
}

function get_no_fly_zones() {
    let no_fly_zones = [];
    for (let i = 0; i < all_overlays.length; ++i) {
        let overlay = all_overlays[i].overlay;
        if (overlay.type === google.maps.drawing.OverlayType.POLYGON) {
            if (overlay.fillColor !== FLY_ZONE_COLOR) {
                no_fly_zones.push(get_polygon_points(overlay));
            }
        }
    }
    return no_fly_zones;
}

function generate_trajectories(e) {
    e.preventDefault();

    clear_paths_from_map();
    let init_point = get_start_point();
    let no_fly_zones = get_no_fly_zones();
    let fly_zone = get_fly_zone();

    if (fly_zone === undefined) {
        alert("There should be at least one fly zone");
        return;
    }

    if (init_point === undefined) {
        alert("UAVs initial position is not placed on the map");
        return;
    }


    fly_zone.push(fly_zone[0]);
    for (let i = 0; i < no_fly_zones.length; ++i) {
        no_fly_zones[i].push(no_fly_zones[i][0]);
    }


    let form_data = new FormData(e.target);
    let value = Object.fromEntries(form_data.entries());
    value["fly-zone"] = fly_zone;
    value["no-fly-zones"] = no_fly_zones;
    value["start-point"] = init_point;


    console.log(value);

    console.log("Values: ");
    console.log(Object.values(value));

    if (Object.keys(value).indexOf("override-drone-spec") > -1) {
        value["override-drone-spec"] = true;
    } else {
        value["override-drone-spec"] = false;
    }

    if (Object.values(value).indexOf("override-battery-model") > -1) {
        value["override-battery-model"] = true;
    } else {
        value["override-battery-model"] = false;
    }

    console.log("Sendong", value);
    let req = new Request("/generate_trajectories", {
        method: 'POST',
        body: JSON.stringify(value),
    });
    fetch(req).then(response => {
        if (response.status != 200) {
            response.text().then(text => {
                alert(text)
            });
            throw new Error();
        } else {
            return response.json();
        }
    }).then(data => {
        console.log(data);
        if (data.success != true) {
            alert(`Error while generating paths: ${data.message}`);
            return;
        }
        uavs_num = data.path.length;
        draw_paths_on_map(data.path);
        update_paths_metrics(data);

    }).catch(er => {
        alert(er)
    })
    return;
}

function save_results(e) {
    e.preventDefault();

    let form_data = new FormData(e.target);
    let value = Object.fromEntries(form_data.entries());
    console.log(value);

    let fly_zone = get_fly_zone();
    let no_fly_zones = get_no_fly_zones();
    let start_point = get_start_point();

    value["fly-zone"] = fly_zone;
    value["no-fly-zones"] = no_fly_zones;
    value["start-point"] = start_point;
    value["paths"] = [];




    for (let path of all_paths) {
        value["paths"].push(get_polygon_points(path));
    }

    let req = new Request("/save_results", {
        method: 'POST',
        body: JSON.stringify(value),
    });
    fetch(req).then(response => {
        if (response.status != 200) {
            response.text().then(text => {
                alert(text)
            });
            throw new Error();
        } else {
            alert("Successfully saved the data");
        }
    });
}

function load_data(e) {
    e.preventDefault();
    let form_data = new FormData(e.target);
    let value = Object.fromEntries(form_data.entries());

    console.log("loading data");
    let req = new Request('/load_polygon', {
        method: 'POST',
        body: JSON.stringify(value)
    })
    fetch(req).then(response => {
        if (response.status != 200) {
            response.text().then(text => {
                alert(text)
            });
            throw new Error();
        } else {
            return response.json();
        }
    }).then(data => {
        deleteAllShape();
        clear_paths_from_map();
        console.log("LOADED DATA: ", data);

        let start_point_marker = make_marker(data['start-point']);
        all_overlays.push(Object({type: google.maps.drawing.OverlayType.MARKER, overlay: start_point_marker}));

        let fly_zone_pol = make_one_polygon(data['fly-zone'], FLY_ZONE_COLOR);
        all_overlays.push(Object({type: 'polygon', overlay: fly_zone_pol}));
        for (let no_fly_zone of data['no-fly-zones']) {
            let pol = Object({type: 'polygon', overlay: make_one_polygon(no_fly_zone, NO_FLY_ZONE_COLOR)});
            all_overlays.push(pol);
        }

        for (let key of Object.keys(data)) {
            if (key === "fly-zone" || key === "no-fly-zones" || key === "start-point") {
                continue;
            }
            console.log("Loading ", key);
            let element = document.getElementById(key);
            if (element.type === "checkbox") {
                element.checked = key === "true";
            } else {
                document.getElementById(key).value = data[key];
            }
        }

        console.log(data);
    })

}

function update_falling_list_values(values) {
    let targets_selection = document.getElementById("target-selection");
    let targets_list = document.getElementById("targets-list");

    let options = "";
    values.forEach(value => {
        options += `<option value=${value}>${value}</option>`;
    })

    let colors = get_distinguishable_colors(uavs_num);
    targets_list.innerHTML = "";
    for (let i = 0; i < uavs_num; ++i) {
        targets_list.innerHTML += `<div><div class="color-marker" style="background-color: ${colors[i]}"></div> <select id="path-${i}-target">${options}</select></div>`;
    }

}

