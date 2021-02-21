/* globals */
var ModuleOpenDrive = null;
var OpenDriveMap = null;
var refline_lines = null;
var road_network_mesh = null;
var lane_outline_lines = null;
var ground_grid = null;
var disposable_objs = [];
var mouse = new THREE.Vector2();
var spotlight_info = document.getElementById('spotlight_info');
var INTERSECTED_ID = 0xffffffff;
var spotlight_paused = false;

const COLORS = {
    road: 0.75,
    outline: 0.25,
    ref_line: 0x69f0ae,
    background: 0x444444,
    lane_highlight: 0x0288d1,
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
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById('ThreeJS').appendChild(renderer.domElement);

/* THREEJS globals */
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100000);
camera.up.set(0, 0, 1); /* Coordinate system with Z pointing up */
const controls = new THREE.MapControls(camera, renderer.domElement);
controls.addEventListener('start', () => { spotlight_paused = true; controls.autoRotate = false; });
controls.addEventListener('end', () => { spotlight_paused = false; });
controls.autoRotate = true;

/* THREEJS lights */
const light = new THREE.DirectionalLight(0xffffff, 1.0);
scene.add(light);
scene.add(light.target);

/* THREEJS auxiliary globals */
const picking_scene = new THREE.Scene();
picking_scene.background = new THREE.Color(0xffffff);
const xyz_scene = new THREE.Scene();
xyz_scene.background = new THREE.Color(0xffffff);
const st_scene = new THREE.Scene();
st_scene.background = new THREE.Color(0xffffff);
const picking_texture = new THREE.WebGLRenderTarget(1, 1, { type: THREE.FloatType });
const xyz_texture = new THREE.WebGLRenderTarget(1, 1, { type: THREE.FloatType });
const st_texture = new THREE.WebGLRenderTarget(1, 1, { type: THREE.FloatType });

/* THREEJS materials */
const idVertexShader = document.getElementById('idVertexShader').textContent;
const idFragmentShader = document.getElementById('idFragmentShader').textContent;
const xyzVertexShader = document.getElementById('xyzVertexShader').textContent;
const xyzFragmentShader = document.getElementById('xyzFragmentShader').textContent;
const stVertexShader = document.getElementById('stVertexShader').textContent;
const stFragmentShader = document.getElementById('stFragmentShader').textContent;

const refline_material = new THREE.LineBasicMaterial({
    color: COLORS.ref_line,
    depthTest: false,
});
const road_network_material = new THREE.MeshPhongMaterial({
    vertexColors: THREE.VertexColors,
    wireframe: PARAMS.wireframe,
    shininess: 15.0,
});
const outlines_material = new THREE.LineBasicMaterial({
    vertexColors: THREE.VertexColors,
});
const id_material = new THREE.ShaderMaterial({
    vertexShader: idVertexShader,
    fragmentShader: idFragmentShader,
});
const xyz_material = new THREE.ShaderMaterial({
    vertexShader: xyzVertexShader,
    fragmentShader: xyzFragmentShader,
});
const st_material = new THREE.ShaderMaterial({
    vertexShader: stVertexShader,
    fragmentShader: stFragmentShader,
});


/* load WASM + odr map */
libOpenDrive().then(Module => {
    ModuleOpenDrive = Module;
    fetch("./data.xodr").then((file_data) => {
        file_data.text().then((file_text) => {
            loadFile(file_text, false);
        });
    });
});

function onFileSelect(file) {
    let file_reader = new FileReader();
    file_reader.onload = () => { loadFile(file_reader.result, true); }
    file_reader.readAsText(file);
}

function loadFile(file_text, clear_map) {
    if (clear_map)
        ModuleOpenDrive['FS_unlink']('./data.xodr');
    ModuleOpenDrive['FS_createDataFile'](".", "data.xodr", file_text, true, true);
    if (OpenDriveMap)
        OpenDriveMap.delete();
    OpenDriveMap = new ModuleOpenDrive.OpenDriveMap("./data.xodr", PARAMS.lateralProfile, PARAMS.laneHeight);
    loadOdrMap(clear_map);
}

function reloadOdrMap() {
    if (OpenDriveMap)
        OpenDriveMap.delete();
    OpenDriveMap = new ModuleOpenDrive.OpenDriveMap("./data.xodr", PARAMS.lateralProfile, PARAMS.laneHeight);
    loadOdrMap(true, false);
}

function loadOdrMap(clear_map = true, fit_view = true) {
    const t0 = performance.now();
    if (clear_map) {
        road_network_mesh.userData.odr_road_network_mesh.delete();
        scene.remove(road_network_mesh, refline_lines, lane_outline_lines, ground_grid);
        picking_scene.remove(...picking_scene.children);
        xyz_scene.remove(...xyz_scene.children);
        st_scene.remove(...st_scene.children);
        for (let obj of disposable_objs)
            obj.dispose();
    }

    /* reflines */
    const reflines_geom = new THREE.BufferGeometry();
    const odr_refline_segments = OpenDriveMap.get_refline_segments(parseFloat(PARAMS.resolution));
    reflines_geom.setAttribute('position', new THREE.Float32BufferAttribute(getStdVecEntries(odr_refline_segments.vertices).flat(), 3));
    reflines_geom.setIndex(getStdVecEntries(odr_refline_segments.indices, true));
    refline_lines = new THREE.LineSegments(reflines_geom, refline_material);
    refline_lines.renderOrder = 10;
    refline_lines.visible = PARAMS.ref_line;
    refline_lines.matrixAutoUpdate = false;
    disposable_objs.push(reflines_geom);
    scene.add(refline_lines);

    /* road network geometry */
    const road_network_geom = new THREE.BufferGeometry();
    const odr_road_network_mesh = OpenDriveMap.get_mesh(parseFloat(PARAMS.resolution));
    road_network_geom.setAttribute('position', new THREE.Float32BufferAttribute(getStdVecEntries(odr_road_network_mesh.vertices, true).flat(), 3));
    road_network_geom.setAttribute('st', new THREE.Float32BufferAttribute(getStdVecEntries(odr_road_network_mesh.st_coordinates, true).flat(), 2));
    road_network_geom.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(road_network_geom.attributes.position.count * 3), 3));
    road_network_geom.setAttribute('id', new THREE.Float32BufferAttribute(new Float32Array(road_network_geom.attributes.position.count * 4), 4));
    road_network_geom.attributes.color.array.fill(COLORS.road);
    road_network_geom.setIndex(getStdVecEntries(odr_road_network_mesh.indices, true));
    for (const [vert_start_idx, _] of getStdMapEntries(odr_road_network_mesh.lane_start_indices)) {
        const vert_idx_interval = odr_road_network_mesh.get_idx_interval_lane(vert_start_idx);
        const vert_count = vert_idx_interval[1] - vert_idx_interval[0];
        const vert_start_idx_encoded = encodeUInt32(vert_start_idx);
        const attr_arr = new Float32Array(vert_count * 4);
        for (let i = 0; i < vert_count; i++)
            attr_arr.set(vert_start_idx_encoded, i * 4);
        road_network_geom.attributes.id.array.set(attr_arr, vert_idx_interval[0] * 4);
    }
    road_network_geom.computeVertexNormals();
    disposable_objs.push(road_network_geom);

    /* road network mesh */
    road_network_mesh = new THREE.Mesh(road_network_geom, road_network_material);
    road_network_mesh.renderOrder = 0;
    road_network_mesh.userData = { odr_road_network_mesh };
    road_network_mesh.matrixAutoUpdate = false;
    scene.add(road_network_mesh);

    /* picking road network mesh */
    const picking_mesh = new THREE.Mesh(road_network_geom, id_material);
    picking_mesh.matrixAutoUpdate = false;
    picking_scene.add(picking_mesh);

    /* xyz coords road network mesh */
    const xyz_mesh = new THREE.Mesh(road_network_geom, xyz_material);
    xyz_mesh.matrixAutoUpdate = false;
    xyz_scene.add(xyz_mesh);

    /* st coords road network mesh */
    const st_mesh = new THREE.Mesh(road_network_geom, st_material);
    st_mesh.matrixAutoUpdate = false;
    st_scene.add(st_mesh);

    /* lane outline */
    const outlines_geom = new THREE.BufferGeometry();
    outlines_geom.setAttribute('position', road_network_geom.attributes.position);
    outlines_geom.setAttribute('color', new THREE.Float32BufferAttribute(new Float32Array(outlines_geom.attributes.position.count * 3), 3));
    outlines_geom.attributes.color.array.fill(COLORS.outline);
    outlines_geom.setIndex(getStdVecEntries(odr_road_network_mesh.get_lane_outline_indices(), true));
    lane_outline_lines = new THREE.LineSegments(outlines_geom, outlines_material);
    lane_outline_lines.renderOrder = 9;
    lane_outline_lines.matrixAutoUpdate = false;
    disposable_objs.push(outlines_geom);
    scene.add(lane_outline_lines);

    /* fit view and camera */
    const bbox_reflines = new THREE.Box3().setFromObject(refline_lines);
    const max_diag_dist = bbox_reflines.min.distanceTo(bbox_reflines.max);
    camera.far = max_diag_dist * 1.5;
    if (fit_view)
        fitViewToBbox(bbox_reflines);

    /* ground grid */
    let bbox_center_pt = new THREE.Vector3();
    bbox_reflines.getCenter(bbox_center_pt);
    ground_grid = new THREE.GridHelper(max_diag_dist, max_diag_dist / 10, 0x2f2f2f, 0x2f2f2f);
    ground_grid.geometry.rotateX(Math.PI / 2);
    ground_grid.position.set(bbox_center_pt.x, bbox_center_pt.y, bbox_reflines.min.z - 0.1);
    disposable_objs.push(ground_grid.geometry);
    scene.add(ground_grid);

    /* fit light */
    light.position.set(bbox_reflines.min.x, bbox_reflines.min.y, bbox_reflines.max.z + max_diag_dist);
    light.target.position.set(bbox_center_pt.x, bbox_center_pt.y, bbox_center_pt.z);
    light.position.needsUpdate = true;
    light.target.position.needsUpdate = true;
    light.target.updateMatrixWorld();

    const t1 = performance.now();
    console.log("Heap size: " + ModuleOpenDrive.HEAP8.length / 1024 / 1024 + " mb");
    const info_msg = `
        <div class=popup_info>
        <h3>Finished loading</h3>
        <table>
            <tr><th>Time</th><th>${((t1 - t0) / 1e3).toFixed(2)}s</th></tr>
            <tr><th>Num Roads</th><th>${OpenDriveMap.roads.size()}</th></tr>
            <tr><th>Num Vertices</th><th>${renderer.info.render.triangles}</th></tr>
        </table>
        </div>`;
    notyf.open({ type: 'info', message: info_msg });

    spotlight_info.style.display = "none";
    controls.autoRotate = true;
    animate();
}

function animate() {
    setTimeout(function () {
        requestAnimationFrame(animate);
    }, 1000 / 40);

    controls.update();

    if (PARAMS.spotlight && !spotlight_paused) {
        camera.setViewOffset(renderer.domElement.width, renderer.domElement.height, mouse.x * window.devicePixelRatio | 0, mouse.y * window.devicePixelRatio | 0, 1, 1);
        renderer.setRenderTarget(picking_texture);
        renderer.render(picking_scene, camera);
        renderer.setRenderTarget(xyz_texture);
        renderer.render(xyz_scene, camera);
        renderer.setRenderTarget(st_texture);
        renderer.render(st_scene, camera);

        const id_pixel_buffer = new Float32Array(4);
        renderer.readRenderTargetPixels(picking_texture, 0, 0, 1, 1, id_pixel_buffer);
        const xyz_pixel_buffer = new Float32Array(4);
        renderer.readRenderTargetPixels(xyz_texture, 0, 0, 1, 1, xyz_pixel_buffer);
        const st_pixel_buffer = new Float32Array(4);
        renderer.readRenderTargetPixels(st_texture, 0, 0, 1, 1, st_pixel_buffer);

        camera.clearViewOffset();
        renderer.setRenderTarget(null);

        if (isValid(id_pixel_buffer)) {
            const decoded_id = decodeUInt32(id_pixel_buffer);
            if (INTERSECTED_ID != decoded_id) {
                if (INTERSECTED_ID != 0xffffffff) {
                    const prev_lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                    road_network_mesh.geometry.attributes.color.array.fill(COLORS.road, prev_lane_vert_idx_interval[0] * 3, prev_lane_vert_idx_interval[1] * 3);
                }
                INTERSECTED_ID = decoded_id;
                const lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                const vert_count = (lane_vert_idx_interval[1] - lane_vert_idx_interval[0]);
                applyVertexColors(road_network_mesh.geometry.attributes.color, new THREE.Color(COLORS.lane_highlight), lane_vert_idx_interval[0], vert_count);
                road_network_mesh.geometry.attributes.color.needsUpdate = true;
                spotlight_info.style.display = "block";
            }
        } else {
            if (INTERSECTED_ID != 0xffffffff) {
                const lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
                road_network_mesh.geometry.attributes.color.array.fill(COLORS.road, lane_vert_idx_interval[0] * 3, lane_vert_idx_interval[1] * 3);
                road_network_mesh.geometry.attributes.color.needsUpdate = true;
            }
            INTERSECTED_ID = 0xffffffff;
            spotlight_info.style.display = "none";
        }

        if (INTERSECTED_ID != 0xffffffff) {
            const road_id = road_network_mesh.userData.odr_road_network_mesh.get_road_id(INTERSECTED_ID);
            const lanesec_s0 = road_network_mesh.userData.odr_road_network_mesh.get_lanesec_s0(INTERSECTED_ID);
            const lane_id = road_network_mesh.userData.odr_road_network_mesh.get_lane_id(INTERSECTED_ID);
            const lane_type = OpenDriveMap.roads.get(road_id).s_to_lanesection.get(lanesec_s0).id_to_lane.get(lane_id).type;
            spotlight_info.innerHTML = `
                    <table>
                        <tr><th>road id</th><th>${road_id}</th></tr>
                        <tr><th>section s0</th><th>${lanesec_s0.toFixed(2)}</th></tr>
                        <tr><th>lane</th><th>${lane_id} <span style="color:gray;">${lane_type}</span></th></tr>
                        <tr><th>s/t</th><th>[${st_pixel_buffer[0].toFixed(2)}, ${st_pixel_buffer[1].toFixed(2)}]</th>
                        <tr><th>world</th><th>[${xyz_pixel_buffer[0].toFixed(2)}, ${xyz_pixel_buffer[1].toFixed(2)}, ${xyz_pixel_buffer[2].toFixed(2)}]</th></tr>
                    </table>`;
        }
    }

    renderer.render(scene, camera);
}

function fitViewToBbox(bbox, restrict_zoom = true) {
    let center_pt = new THREE.Vector3();
    bbox.getCenter(center_pt);

    const l2xy = 0.5 * Math.sqrt(Math.pow(bbox.max.x - bbox.min.x, 2.0) + Math.pow(bbox.max.y - bbox.min.y, 2));
    const fov2r = (camera.fov * 0.5) * (Math.PI / 180.0);
    const dz = l2xy / Math.tan(fov2r);

    camera.position.set(bbox.min.x, center_pt.y, bbox.max.z + dz);
    controls.target.set(center_pt.x, center_pt.y, center_pt.z);
    if (restrict_zoom)
        controls.maxDistance = center_pt.distanceTo(bbox.max) * 1.2;
    controls.update();
}

function fitViewToObj(obj) {
    const bbox = new THREE.Box3().setFromObject(obj);
    fitViewToBbox(bbox);
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

function getStdMapKeys(std_map, delete_map = false) {
    let map_keys = [];
    const map_keys_vec = std_map.keys();
    for (let idx = 0; idx < map_keys_vec.size(); idx++)
        map_keys.push(map_keys_vec.get(idx));
    map_keys_vec.delete();
    if (delete_map)
        std_map.delete();
    return map_keys;
}

function getStdMapEntries(std_map) {
    let map_entries = [];
    for (let key of getStdMapKeys(std_map))
        map_entries.push([key, std_map.get(key)]);
    return map_entries;
}

function getStdVecEntries(std_vec, delete_vec = false, ArrayType = null) {
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

function onDocumentMouseDbClick(e) {
    if (INTERSECTED_ID != 0xffffffff) {
        const lane_vert_idx_interval = road_network_mesh.userData.odr_road_network_mesh.get_idx_interval_lane(INTERSECTED_ID);
        const vertA = road_network_mesh.userData.odr_road_network_mesh.vertices.get(lane_vert_idx_interval[0]);
        const vertB = road_network_mesh.userData.odr_road_network_mesh.vertices.get(lane_vert_idx_interval[1] - 1);
        const bbox = new THREE.Box3();
        bbox.setFromArray([vertA, vertB].flat());
        fitViewToBbox(bbox, false);
    }
}