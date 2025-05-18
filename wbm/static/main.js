var map = L.map('map').setView([51.505, -0.09], 13);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
}).addTo(map);

const configData = JSON.parse(document.body.dataset.token);
const serviceAreaCenter = configData["service_area_center"];
const serviceAreaRadius = configData["service_area_radius"];
const cameraFov = configData["camera_fov"];
let height = configData["init_height"];
let period = configData["init_period"];


const HEIGHT_MIN = 10;
const HEIGHT_MAX = 100;


function projectedAreaFromHeight(height, hfov, vfov) {
    let areaWidth = 2 * height * Math.tan(hfov * Math.PI / 360);
    let areaHeight = 2 * height * Math.tan(vfov * Math.PI / 360);
    console.log("Tile size in meters:", areaWidth, areaHeight);

    return { width: areaWidth, height: areaHeight };
}


var circle = L.circle(serviceAreaCenter, {
    color: 'red',
    fillColor: '#f03',
    fillOpacity: 0.1,
    radius: serviceAreaRadius
}).addTo(map);


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
});


document.getElementById("set-height-button").addEventListener("click", () => {

    const newHeight = document.getElementById("height-input").value;
    if (HEIGHT_MIN <= newHeight && newHeight <= HEIGHT_MAX) {
        document.getElementById("height-current").textContent = newHeight;
        height = newHeight;
    }
});