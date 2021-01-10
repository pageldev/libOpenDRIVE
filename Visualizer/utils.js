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