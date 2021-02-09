/* globals */
var ModuleOpenDrive = null;
var OpenDriveMap = null;
var refline_lines = null;
var road_network_mesh = null;
var lane_outline_lines = null;
var sky_dome = null;
var disposable_objs = [];
var mouse = new THREE.Vector2();
var spotlight_info = document.getElementById('spotlight_info');
var INTERSECTED_ID = null;

const COLORS = {
    road: 0.68,
    outline: 0x757575,
    ref_line: 0x69f0ae,
    background: 0x444444,
    lane_highlight: 0x0288d1,
    road_highlight: 0x9A9A9A,
    outline_highlight: 0xc158dc,
};

/* event listeners */
window.addEventListener('resize', onWindowResize, false);
window.addEventListener('mousemove', onDocumentMouseMove, false);
window.addEventListener('dblclick', onDocumentMouseDbClick, false);

/* notifactions */
const notyf = new Notyf({
    duration: 3000,
    position: { x: 'right', y: 'bottom' },
    types: [{ type: 'info', background: '#607d8b', icon: false }]
});

/* THREEJS renderer */
const renderer = new THREE.WebGLRenderer({ antialias: true, sortObjects: false });
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.BasicShadowMap;
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('ThreeJS').appendChild(renderer.domElement);

/* THREEJS globals */
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100000);
camera.up.set(0, 0, 1); /* Coordinate system with Z pointing up */
const controls = new THREE.MapControls(camera, renderer.domElement);
controls.addEventListener('change', () => { renderer.render(scene, camera) });

/* THREEJS lights */
const light = new THREE.DirectionalLight(0xffffff);
light.position.set(0, 0, 1);
light.castShadow = true;
scene.add(light);

/* THREEJS auxiliary globals */
const picking_scene = new THREE.Scene();
picking_scene.background = new THREE.Color(0xffffff);
const picking_texture = new THREE.WebGLRenderTarget(1, 1, { type: THREE.FloatType });

/* THREEJS materials */
const idVertexShader = document.getElementById('idVertexShader').textContent;
const idFragmentShader = document.getElementById('idFragmentShader').textContent;
const skyDomeVertexShader = document.getElementById('skyDomeVertexShader').textContent;
const skyDomeFragmentShader = document.getElementById('skyDomeFragmentShader').textContent;

const refline_material = new THREE.LineBasicMaterial({
    color: COLORS.ref_line
});
const road_network_material = new THREE.MeshPhongMaterial({
    vertexColors: THREE.VertexColors,
    wireframe: PARAMS.wireframe,
    polygonOffset: true,
    polygonOffsetFactor: 20,
    polygonOffsetUnits: 1,
    shininess: 15,
    flatShading: true
});
const outlines_material = new THREE.LineBasicMaterial({
    color: COLORS.outline
});
const picking_material = new THREE.ShaderMaterial({
    vertexShader: idVertexShader,
    fragmentShader: idFragmentShader,
});
const sky_material = new THREE.ShaderMaterial({
    vertexShader: skyDomeVertexShader,
    fragmentShader: skyDomeFragmentShader,
    side: THREE.BackSide
});

renderer.render(scene, camera);

/* load WASM + odr map */
libOpenDrive().then(Module => {
    ModuleOpenDrive = Module;
    fetch("./eva.xodr").then((file_data) => {
        file_data.text().then((file_text) => {
            file_load(file_text, false);
        });
    });
});

function file_load(file_text, clear_map) {
    if (clear_map)
        ModuleOpenDrive['FS_unlink']('./data.xodr');
    ModuleOpenDrive['FS_createDataFile'](".", "data.xodr", file_text, true, true);
    OpenDriveMap = new ModuleOpenDrive.OpenDriveMap("./data.xodr", PARAMS.lateralProfile, PARAMS.laneHeight);
    load_odr_map(clear_map);
}

function on_file_select(file) {
    let file_reader = new FileReader();
    file_reader.onload = () => { file_load(file_reader.result, true); }
    file_reader.readAsText(file);
}

function reload_odr_map() {
    OpenDriveMap = new ModuleOpenDrive.OpenDriveMap("./data.xodr", PARAMS.lateralProfile, PARAMS.laneHeight);
    load_odr_map(true, false);
}

function load_odr_map(clear_map = true, fit_view = true) {
    const t0 = performance.now();
    if (clear_map) {
        scene.remove(road_network_mesh, refline_lines, lane_outline_lines, sky_dome);
        picking_scene.remove(...picking_scene.children);
        for (let obj of disposable_objs)
            obj.dispose();
    }

    const odr_roads = OpenDriveMap.roads;

    /* reflines */
    const reflines_geom = new THREE.BufferGeometry();
    const odr_refline_segments = OpenDriveMap.get_refline_segments(PARAMS.resolution);
    reflines_geom.setAttribute('position', new THREE.Float32BufferAttribute(get_std_vec_entries(odr_refline_segments.vertices).flat(), 3));
    reflines_geom.setIndex(get_std_vec_entries(odr_refline_segments.indices, true));
    refline_lines = new THREE.LineSegments(reflines_geom, refline_material);
    refline_lines.renderOrder = 10;
    disposable_objs.push(reflines_geom);
    scene.add(refline_lines);

    /* road network geometry */
    const road_network_geom = new THREE.BufferGeometry();
    const odr_road_network_mesh = OpenDriveMap.get_mesh(PARAMS.resolution);
    road_network_geom.setAttribute('position', new THREE.Float32BufferAttribute(get_std_vec_entries(odr_road_network_mesh.vertices, true).flat(), 3));
    road_network_geom.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(road_network_geom.attributes.position.count * 3), 3));
    road_network_geom.attributes.color.array.fill(COLORS.road);
    road_network_geom.setIndex(get_std_vec_entries(odr_road_network_mesh.indices, true));
    road_network_geom.setAttribute('id', new THREE.Float32BufferAttribute(new Float32Array(road_network_geom.attributes.position.count * 4), 4));
    for (const [vert_start_idx, lane_id] of get_std_map_entries(odr_road_network_mesh.lane_start_indices)) {
        const vert_idx_interval = odr_road_network_mesh.get_idx_interval_lane(vert_start_idx);
        const vert_count = vert_idx_interval[1] - vert_idx_interval[0];
        const vert_start_idx_encoded = encodeUInt32(vert_start_idx);
        const attr_arr = new Float32Array(vert_count * 4);
        for (let i = 0; i < vert_count; i++)
            attr_arr.set(vert_start_idx_encoded, i * 4);
        road_network_geom.attributes.id.array.set(attr_arr, vert_idx_interval[0] * 4);
    }
    disposable_objs.push(road_network_geom);

    /* road network mesh */
    road_network_mesh = new THREE.Mesh(road_network_geom, road_network_material);
    road_network_mesh.renderOrder = 0;
    road_network_mesh.receiveShadow = true;
    road_network_mesh.userData = { odr_road_network_mesh };
    scene.add(road_network_mesh);

    /* picking road network mesh */
    const picking_mesh = new THREE.Mesh(road_network_geom, picking_material);
    picking_scene.add(picking_mesh);

    /* lane outline */
    const outlines_geom = new THREE.BufferGeometry();
    outlines_geom.setAttribute('position', road_network_geom.attributes.position);
    outlines_geom.setIndex(get_std_vec_entries(odr_road_network_mesh.get_lane_outline_indices(), true));
    lane_outline_lines = new THREE.LineSegments(outlines_geom, outlines_material);
    lane_outline_lines.renderOrder = 9;
    scene.add(lane_outline_lines);

    /* sky dome */
    const bbox_reflines = new THREE.Box3().setFromObject(refline_lines);
    const bbox_center_pt = new THREE.Vector3();
    bbox_reflines.getCenter(bbox_center_pt);
    const dome_radius = Math.max(bbox_reflines.min.distanceTo(bbox_reflines.max) * 3, 800);
    const sky_geom = new THREE.SphereGeometry(dome_radius, 32, 15);
    sky_dome = new THREE.Mesh(sky_geom, sky_material);
    sky_dome.position.set(bbox_center_pt.x, bbox_center_pt.y, bbox_center_pt.z);
    disposable_objs.push(sky_geom);
    scene.add(sky_dome);

    if (fit_view)
        fitView(refline_lines);

    renderer.render(scene, camera);

    const t1 = performance.now();
    console.log("Heap size: " + ModuleOpenDrive.HEAP8.length / 1024 / 1024 + " mb");
    const info_msg = `
        <div class=popup_info>
        <h3>Finished loading</h3>
        <table>
            <tr><th>Time</th><th>${((t1 - t0) / 1e3).toFixed(2)}s</th></tr>
            <tr><th>Num Roads</th><th>${odr_roads.size()}</th></tr>
            <tr><th>Num Vertices</th><th>${renderer.info.render.triangles}</th></tr>
        </table>
        </div>`;
    notyf.open({ type: 'info', message: info_msg });

    spotlight_info.style.display = "none";
    animate();
}

function animate() {
    requestAnimationFrame(animate);

    if (PARAMS.spotlight) {
        renderer.setRenderTarget(picking_texture);
        camera.setViewOffset(renderer.domElement.width, renderer.domElement.height, mouse.x * window.devicePixelRatio | 0, mouse.y * window.devicePixelRatio | 0, 1, 1);
        renderer.render(picking_scene, camera);
        camera.clearViewOffset();

        const id_pixel_buffer = new Float32Array(4);
        renderer.readRenderTargetPixels(picking_texture, 0, 0, 1, 1, id_pixel_buffer);
        renderer.setRenderTarget(null);

        if (isValid(id_pixel_buffer)) {
            const decoded_id = decodeUInt32(id_pixel_buffer);
            if (INTERSECTED_ID != decoded_id) {
                if (INTERSECTED_ID) {
                    const prev_lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                    road_network_mesh.geometry.attributes.color.array.fill(COLORS.road, prev_lane_vert_idx_interval[0] * 3, prev_lane_vert_idx_interval[1] * 3);
                }
                INTERSECTED_ID = decoded_id;
                const lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                const vert_count = (lane_vert_idx_interval[1] - lane_vert_idx_interval[0]);
                applyVertexColors(road_network_mesh.geometry.attributes.color, new THREE.Color(COLORS.lane_highlight), lane_vert_idx_interval[0], vert_count);
                road_network_mesh.geometry.attributes.color.needsUpdate = true;
                renderer.render(scene, camera);
            }
        } else {
            if (INTERSECTED_ID) {
                const lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                road_network_mesh.geometry.attributes.color.array.fill(COLORS.road, lane_vert_idx_interval[0] * 3, lane_vert_idx_interval[1] * 3);
                road_network_mesh.geometry.attributes.color.needsUpdate = true;
                renderer.render(scene, camera);
            }
            INTERSECTED_ID = null;
        }
    }
}

function fitView(obj) {
    const bbox = new THREE.Box3().setFromObject(obj);
    let center_pt = new THREE.Vector3();
    bbox.getCenter(center_pt);

    const l2xy = 0.5 * Math.sqrt(Math.pow(bbox.max.x - bbox.min.x, 2.0) + Math.pow(bbox.max.y - bbox.min.y, 2));
    const fov2r = (camera.fov * 0.5) * (Math.PI / 180.0);
    const dz = l2xy / Math.tan(fov2r);

    camera.position.set(center_pt.x, center_pt.y, bbox.max.z + dz);
    controls.target.set(center_pt.x, center_pt.y, center_pt.z);
    controls.maxDistance = center_pt.distanceTo(bbox.max) * 2.0;
    controls.update();

    renderer.render(scene, camera);
}

function applyVertexColors(buffer_attribute, color, offset, count) {
    const colors = new Float32Array(count * buffer_attribute.itemSize);
    for (let i = 0; i < (count * buffer_attribute.itemSize); i += buffer_attribute.itemSize) {
        colors[i] = color.r;
        colors[i + 1] = color.g;
        colors[i + 2] = color.b;
    }
    buffer_attribute.array.set(colors, offset * buffer_attribute.itemSize);
}

function get_std_map_keys(std_map, delete_map = false) {
    let map_keys = [];
    const map_keys_vec = std_map.keys();
    for (let idx = 0; idx < map_keys_vec.size(); idx++)
        map_keys.push(map_keys_vec.get(idx));
    map_keys_vec.delete();
    if (delete_map)
        std_map.delete();
    return map_keys;
}

function get_std_map_entries(std_map) {
    let map_entries = [];
    for (let key of get_std_map_keys(std_map))
        map_entries.push([key, std_map.get(key)]);
    return map_entries;
}

function get_std_vec_entries(std_vec, delete_vec = false, ArrayType = null) {
    let entries = ArrayType ? new ArrayType(std_vec.size()) : new Array(std_vec.size());
    for (let idx = 0; idx < std_vec.size(); idx++)
        entries[idx] = std_vec.get(idx);
    if (delete_vec)
        std_vec.delete();
    return entries;
}

function isValid(rgba) {
    return !(rgba[0] == 1 && rgba[1] == 1 && rgba[2] == 1 && rgba[3] == 1);
}

function encodeUInt32(ui32) {
    rgba = new Float32Array(4);
    rgba[0] = (Math.trunc(ui32) % 256) / 255.;
    rgba[1] = (Math.trunc(ui32 / 256) % 256) / 255.;
    rgba[2] = (Math.trunc(ui32 / 256 / 256) % 256) / 255.;
    rgba[3] = (Math.trunc(ui32 / 256 / 256 / 256) % 256) / 255.;
    return rgba;
}

function decodeUInt32(rgba) {
    return Math.round(rgba[0] * 255) +
        Math.round(rgba[1] * 255) * 256 +
        Math.round(rgba[2] * 255) * 256 * 256 +
        Math.round(rgba[3] * 255) * 256 * 256 * 256;
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function onDocumentMouseMove(event) {
    event.preventDefault();
    mouse.x = event.clientX;
    mouse.y = event.clientY;
}

function onDocumentMouseDbClick(event) {
}