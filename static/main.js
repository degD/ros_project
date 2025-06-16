
const configData = JSON.parse(document.body.dataset.token);
const serviceAreaCenter = configData["service_area_center"];
const serviceAreaRadius = configData["service_area_radius"];
const cameraFov = configData["camera_fov"];
const homePoint = configData["home_point"];
let height = configData["init_height"];
let period = configData["period"];
const MAPTILER_KEY = document.body.dataset.maptileKey;

var map = L.map('map').setView(configData["service_area_center"], 17);

// Less detail, but much higher max zoom
L.tileLayer(`https://api.maptiler.com/maps/streets/{z}/{x}/{y}.png?key=${MAPTILER_KEY}`, {
    maxZoom: 22,
    attribution: '&copy; <a href="https://www.maptiler.com/copyright/">MapTiler</a>'
}).addTo(map);

// Better details, free OSM, but less max zoom
// L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
//     maxZoom: 19,
//     attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
// }).addTo(map);


const HEIGHT_MIN = 10;
const HEIGHT_MAX = 100;


function projectedAreaFromHeight(height, hfov, vfov) {
    let areaWidth = 2 * height * Math.tan(hfov * Math.PI / 360);
    let areaHeight = 2 * height * Math.tan(vfov * Math.PI / 360);
    console.log("Tile size in meters:", areaWidth, areaHeight);

    return { width: areaWidth, height: areaHeight };
}


// Service area
var circle = L.circle(serviceAreaCenter, {
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 0.1,
    radius: serviceAreaRadius
}).addTo(map);

// Service area center
L.circleMarker(serviceAreaCenter, {
    radius: 6,
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 1
}).addTo(map);

// Drone position relative to map
let droneMarker = null;
function updateDroneMarker() {
    fetch('/get_drone_position')
        .then(response => response.json())
        .then(pos => {
            // 1. Project the service area center to meters
            const centerMeters = latlngToMeters({lat: serviceAreaCenter[0], lng: serviceAreaCenter[1]});
            // 2. Add the relative x, y offsets
            const droneMeters = {
                x: centerMeters.x + pos.x,
                y: centerMeters.y + pos.y
            };
            // 3. Convert back to lat/lng
            const latlng = metersToLatlng(droneMeters);

            if (droneMarker) {
                droneMarker.setLatLng(latlng);
            } else {
                droneMarker = L.circleMarker(latlng, {
                    radius: 3,
                    color: 'blue',
                    fillColor: '#00f',
                    fillOpacity: 0.8
                }).addTo(map);
            }
        });
}
setInterval(updateDroneMarker, 2000);

// Initialize drone marker at center
droneMarker = L.circleMarker(serviceAreaCenter, {
    radius: 3,
    color: 'blue',
    fillColor: '#00f',
    fillOpacity: 0.8
}).addTo(map);


// Add drone altitude overlay
const altitudeControl = L.control({position: 'bottomright'});

altitudeControl.onAdd = function (map) {
    const div = L.DomUtil.create('div', 'info altitude');
    div.style.background = 'white';
    div.style.padding = '8px';
    div.style.borderRadius = '6px';
    div.style.boxShadow = '0 2px 6px rgba(0,0,0,0.2)';
    div.innerHTML = `<b>Drone Altitude:</b> <span id="drone-altitude">-</span> m`;
    return div;
};

altitudeControl.addTo(map);

function updateDroneAltitude() {
    fetch('/get_drone_position')
        .then(response => response.json())
        .then(pos => {
            document.getElementById("drone-altitude").textContent =
                (typeof pos.z === "number") ? pos.z.toFixed(2) : "-";
        });
}

setInterval(updateDroneAltitude, 2000);
updateDroneAltitude();


let tileLayerGroup = L.layerGroup().addTo(map);
let area = null;               // Final completed polygon
let tempPreview = null;        // Temporary polygon while drawing
let drawStatus = 0;            // 0 = not drawing, 1 = drawing
let currentPoints = [];

const button = document.getElementById("draw-area-button");

button.addEventListener("click", () => {
    if (drawStatus === 0) {
        // Start drawing
        currentPoints = [];
        if (area) {
            map.removeLayer(area);
            area = null;
        }
        if (tempPreview) {
            map.removeLayer(tempPreview);
            tempPreview = null;
        }
        tileLayerGroup.clearLayers();

        drawStatus = 1;
        button.textContent = "Complete";
    } else {
        if (currentPoints.length >= 3) {
            if (tempPreview) {
                map.removeLayer(tempPreview);
                tempPreview = null;
            }
            area = L.polygon(currentPoints, { color: 'blue' }).addTo(map);

            const tile = projectedAreaFromHeight(height, cameraFov[0], cameraFov[1]);
            const areaGroup = L.featureGroup([area]); // wrap single polygon in a group
            tilePolygonArea(areaGroup, tile.width, tile.height);
        }
        drawStatus = 0;
        button.textContent = "Draw Area";
    }
});

map.on("click", function (e) {
    if (drawStatus === 1) {
        currentPoints.push([e.latlng.lat, e.latlng.lng]);

        // Remove old preview before adding new one
        if (tempPreview) {
            map.removeLayer(tempPreview);
        }
        tempPreview = L.polygon(currentPoints, {
            color: 'blue',
            dashArray: '5, 5',
            fillOpacity: 0.1
        }).addTo(map);
    }
});



const crs = new L.Proj.CRS('EPSG:3857',
    '+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs',
    { resolutions: [8192, 4096, 2048, 1024, 512, 256, 128], origin: [0, 0] }
);

function latlngToMeters(latlng) {
    return crs.project(L.latLng(latlng));
}

function metersToLatlng(point) {
    return crs.unproject(L.point(point.x, point.y));
}

function tilePolygonArea(polygonGroup, tileWidth, tileHeight) {
    if (!tileWidth || !tileHeight || isNaN(tileWidth) || isNaN(tileHeight)) {
        console.error("Invalid tile size:", tileWidth, tileHeight);
        return;
    }

    // Get the polygon from group and convert to Turf.js polygon
    const polygon = polygonGroup.getLayers()[0];
    if (!polygon) {
        console.warn("No polygon found in group");
        return;
    }

    const geojson = polygon.toGeoJSON();
    const turfPoly = turf.polygon(geojson.geometry.coordinates);

    // Convert latlngs to projected coordinates
    const rawLatlngs = polygon.getLatLngs();
    const latlngs = Array.isArray(rawLatlngs[0]) ? rawLatlngs[0] : rawLatlngs;
    const meters = latlngs.map(latlngToMeters);

    // Compute bounding box
    let minX = Math.min(...meters.map(p => p.x));
    let maxX = Math.max(...meters.map(p => p.x));
    let minY = Math.min(...meters.map(p => p.y));
    let maxY = Math.max(...meters.map(p => p.y));

    let tileCount = 0;

    for (let x = minX; x < maxX; x += tileWidth) {
        for (let y = minY; y < maxY; y += tileHeight) {
            const tileBounds = [
                { x: x, y: y },
                { x: x + tileWidth, y: y },
                { x: x + tileWidth, y: y + tileHeight },
                { x: x, y: y + tileHeight }
            ];

            const latlngBounds = tileBounds.map(metersToLatlng);
            if (latlngBounds.includes(null)) continue;

            // Convert to Turf polygon with closure
            const ring = latlngBounds.map(ll => [ll.lng, ll.lat]);
            ring.push(ring[0]);  // Close the ring

            const turfTile = turf.polygon([ring]);

            if (!turf.booleanDisjoint(turfTile, turfPoly)) {
                const tile = L.polygon(latlngBounds, {
                    color: 'green',
                    weight: 1,
                    fillOpacity: 0.1
                });
                tileLayerGroup.addLayer(tile);

                tileCount++;
            }
        }
    }

    console.log("Total tiles added:", tileCount);
}


document.addEventListener("DOMContentLoaded", () => {
    
    document.getElementById("height-input").value = height;
    document.getElementById("height-current").textContent = height;

    document.getElementById("period-total").textContent = period;
    document.getElementById("period-current").textContent = Math.floor(periodCounter / (3600 * 1000));
});


document.getElementById("set-height-button").addEventListener("click", () => {
    
    const newHeight = document.getElementById("height-input").value;
    if (HEIGHT_MIN <= newHeight && newHeight <= HEIGHT_MAX) {
        document.getElementById("height-current").textContent = newHeight;
        height = newHeight;
    }
});


let interval = null;
let counterInterval = null;
let periodCounter = 0
const startBtn = document.getElementById("start-button");
const stopBtn = document.getElementById("stop-button");
startBtn.addEventListener("click", () => {

    console.log("Start sending data...");

    // Initially send navData to '/navigation' endpoint
    const centers = [];
    tileLayerGroup.eachLayer(layer => {
        let center;
        center = layer.getBounds().getCenter();
        centers.push(latlngToMeters(center));
    });
    centers.push(latlngToMeters({ "lat": homePoint[0], "lng": homePoint[1] }));
    const navData = {
        "home": latlngToMeters(homePoint),
        "height": Number(height),
        "tiles": centers,
    };
    fetch('/navigation', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(navData)
    }).then(response => {
        if (!response.ok) {
            console.error('Failed to send navigation data');
        }
    }).catch(error => {
        console.error('Error sending navigation data:', error);
    });

    interval = setInterval(() => {
        const centers = [];
        tileLayerGroup.eachLayer(layer => {
            let center;
            center = layer.getBounds().getCenter();
            centers.push(latlngToMeters(center));
        });
        centers.push(latlngToMeters({ "lat": homePoint[0], "lng": homePoint[1] }));
        const navData = {
            "home": latlngToMeters(homePoint),
            "height": Number(height),
            "tiles": centers,
        };

        // Send navData to '/navigation' endpoint
        fetch('/navigation', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(navData)
        }).then(response => {
            if (!response.ok) {
                console.error('Failed to send navigation data');
            }
        }).catch(error => {
            console.error('Error sending navigation data:', error);
        });
        
        console.log("Sent period navigation.");
    }, 
    period * 3600 * 1000);

    counterInterval = setInterval(() => {
        document.getElementById("period-current").textContent = Math.floor(periodCounter / period);
        periodCounter++;
    }, 1000);

    startBtn.disabled = true;
    stopBtn.disabled = false;

    // Polling for completion of navigation
    startPollingStatus()
});


stopBtn.addEventListener("click", () => {

    clearInterval(interval);
    clearInterval(counterInterval);
    clearInterval(pollInterval); // Stop polling status
    startBtn.disabled = false;
    stopBtn.disabled = true;
});


// Poll navigation status every second
let pollInterval = null;

function startPollingStatus() {
    pollInterval = setInterval(() => {
        fetch('/navigation_status')
            .then(response => response.text().then(text => ({ status: response.status, text })))
            .then(({ status, text }) => {
                if (status === 200 && text.includes("complete")) {
                    alert("Navigation complete!");
                    clearInterval(pollInterval);

                    // Trigger download of plates.csv
                    const link = document.createElement('a');
                    link.href = '/plates.txt';
                    link.download = 'plates.txt';
                    document.body.appendChild(link);
                    link.click();
                    document.body.removeChild(link);
                } else {
                    console.log("Navigation in progress...");
                }
            })
            .catch(error => {
                console.error("Error polling navigation status:", error);
            });
    }, 1000);
}


function drawCarRectangles() {
    // Fetch both car locations and plates
    Promise.all([
        fetch('/get_car_locations').then(r => r.json()),
        fetch('/get_car_plates').then(r => r.json())
    ]).then(([carLocations, platesList]) => {
        // Build a map from index to plate
        const plateMap = new Map();
        platesList.forEach(([idx, plate]) => {
            plateMap.set(Number(idx), plate);
        });

        // Remove old rectangles if needed
        if (window.carRectsGroup) {
            map.removeLayer(window.carRectsGroup);
        }
        window.carRectsGroup = L.layerGroup().addTo(map);

        carLocations.forEach((car, index) => {
            // Rectangle center in meters (relative to service area center)
            const centerMeters = latlngToMeters({lat: serviceAreaCenter[0], lng: serviceAreaCenter[1]});
            const carCenter = {
                x: centerMeters.x + car.x_global,
                y: centerMeters.y + car.y_global
            };

            // Rectangle parameters
            const w = car.h_global * 2;
            const h = car.w_global * 2;
            const angle = car.angle; // radians

            // Compute rectangle corners (relative to center)
            const dx = w / 2;
            const dy = h / 2;

            // Rotation matrix
            const cosA = Math.cos(angle);
            const sinA = Math.sin(angle);

            // Rectangle corners (clockwise)
            const corners = [
                {x: -dx, y: -dy},
                {x: dx, y: -dy},
                {x: dx, y: dy},
                {x: -dx, y: dy}
            ].map(corner => {
                // Rotate and translate
                const x_rot = corner.x * cosA - corner.y * sinA + carCenter.x;
                const y_rot = corner.x * sinA + corner.y * cosA + carCenter.y;
                return metersToLatlng({x: x_rot, y: y_rot});
            });

            // Determine color based on plate status
            let color = "orange";
            if (plateMap.has(index)) {
                const plate = plateMap.get(index);
                if (plate === "FAILED") color = "red";
                else if (plate === "DISABLED") color = "blue";
                else color = "green";
            }

            // Draw rectangle
            L.polygon(corners, {
                color: color,
                weight: 1,
                fillOpacity: 0.2
            }).addTo(window.carRectsGroup);
        });
    });
}
setInterval(drawCarRectangles, 2000);
drawCarRectangles();


// Add car color legend overlay
const legend = L.control({position: 'topright'});

legend.onAdd = function (map) {
    const div = L.DomUtil.create('div', 'info legend');
    div.style.background = 'white';
    div.style.padding = '8px';
    div.style.borderRadius = '6px';
    div.style.boxShadow = '0 2px 6px rgba(0,0,0,0.2)';
    div.innerHTML = `
        <b>Car Plate Status</b><br>
        <i style="background: orange; width: 12px; height: 12px; display: inline-block; border-radius:2px; margin-right:4px;"></i> Not read<br>
        <i style="background: red; width: 12px; height: 12px; display: inline-block; border-radius:2px; margin-right:4px;"></i> FAILED<br>
        <i style="background: blue; width: 12px; height: 12px; display: inline-block; border-radius:2px; margin-right:4px;"></i> DISABLED<br>
        <i style="background: green; width: 12px; height: 12px; display: inline-block; border-radius:2px; margin-right:4px;"></i> Success
    `;
    return div;
};

legend.addTo(map);