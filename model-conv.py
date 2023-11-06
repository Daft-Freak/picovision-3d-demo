import json
import numpy as np
from pathlib import Path
import struct
import sys
from transforms3d import affines, quaternions

component_size = {
    0x1403: 2, # GL_UNSIGNED_SHORT
    0x1406: 4, # GL_FLOAT
}

# returns numpy array with data
def get_accessor_data(filename, json_data, json_accessor, floatify = True):
    # get buffer view
    buffer_view = json_data['bufferViews'][json_accessor['bufferView']]
    
    # ... then the buffer
    buffer = json_data['buffers'][buffer_view['buffer']]

    # read view data
    # TODO: maybe don't open the file repeatedly...
    buffer_path = Path(filename).parent / buffer['uri']
    buffer_file = open(buffer_path, 'rb')
    buffer_file.seek(buffer_view['byteOffset'])
    view_data = buffer_file.read(buffer_view['byteLength'])

    # parse type
    vec_size = 0
    if json_accessor['type'] == 'SCALAR':
        vec_size = 1
    elif json_accessor['type'] == 'VEC2':
        vec_size = 2
    elif json_accessor['type'] == 'VEC3':
        vec_size = 3
    elif json_accessor['type'] == 'VEC4':
        vec_size = 4
    else:
        print(f'unhanded accessor type {json_accessor["componentType"]}')

    # size/count stuff
    accessor_count = json_accessor['count']
    comp_type = json_accessor['componentType']
    comp_size = component_size[comp_type]

    unpacked_data = None

    # trim if too big (padding?)
    if len(view_data) > vec_size * accessor_count * comp_size:
        view_data = view_data[:vec_size * accessor_count * comp_size]

    # decode data
    if comp_type == 0x1406: # GL_FLOAT
        unpacked_data = struct.unpack(f'<{accessor_count * vec_size}f', view_data)
    elif comp_type == 0x1403: # GL_UNSIGNED_SHORT
        unpacked_data = struct.unpack(f'<{accessor_count * vec_size}H', view_data)
        if floatify:
            unpacked_data = np.array(unpacked_data) / 0xFFFF
    else:
        print(f'unhanded accessor component type {json_accessor["componentType"]}')

    # TODO: convert to float if not indices

    if vec_size == 1:
        return np.array(unpacked_data)
    else:
        return np.array(unpacked_data).reshape((accessor_count, vec_size))


def handle_mesh(filename, json_data, json_mesh, matrix):
    # this is converting glTF primitives to our "meshes"...
    primitives = []

    t, rot_matrix, z, s = affines.decompose(matrix)

    accessors = json_data['accessors']

    for primitive in json_mesh['primitives']:
        out_vertices = []

        prim_attribs = primitive['attributes']

        # get accessors
        position_accessor = accessors[prim_attribs['POSITION']]

        # TODO: may not exist
        normal_accessor = accessors[prim_attribs['NORMAL']]

        colour_accessor = None
        if 'COLOR_0' in prim_attribs:
            colour_accessor = accessors[prim_attribs['COLOR_0']]

        tex_coord_accessor = None
        if 'TEXCOORD_0' in prim_attribs:
            tex_coord_accessor = accessors[prim_attribs['TEXCOORD_0']]

        # TODO: may not exist, non-indexed data
        index_accessor = accessors[primitive['indices']]

        # parse data
        position_data = get_accessor_data(filename, json_data, position_accessor)
        normal_data = get_accessor_data(filename, json_data, normal_accessor)

        # default colour if not set
        if colour_accessor:
            colour_data = get_accessor_data(filename, json_data, colour_accessor)
        else:
            colour_data = np.full((len(position_data), 4), [1, 1, 1, 1])

        # get tex coords or all zeros
        if tex_coord_accessor:
            tex_coord_data = get_accessor_data(filename, json_data, tex_coord_accessor)
        else:
            tex_coord_data = np.full((len(position_data), 2), [0, 0])

        index_data = get_accessor_data(filename, json_data, index_accessor, False)

        # check if material is textured
        material = None
        tex_id = 0 # no tex

        if 'material' in primitive:
            material_id = primitive['material']
            material = json_data['materials'][material_id]

            # TODO: other material types?
            if 'pbrMetallicRoughness' in material and 'baseColorTexture' in material['pbrMetallicRoughness']:
                tex_id = material['pbrMetallicRoughness']['baseColorTexture']['index'] + 1

        # triangles
        for i in index_data:
            # transform pos/normal
            pos = matrix @ np.append(position_data[i], 1)
            nor = rot_matrix @ normal_data[i]

            pos[1] *= -1 # flip y
            nor[1] *= -1

            tex_coord = tex_coord_data[i]

            # make tex coords positive
            if tex_coord[0] < 0.0:
                tex_coord[0] = 1.0 - (tex_coord[0] % 1)
            if tex_coord[1] < 0.0:
                tex_coord[1] = 1.0 - (tex_coord[1] % 1)

            # TODO: wrap >1?

            # convert data
            pos = np.floor(pos * (1 << 16)).astype(int)
            nor = np.floor(nor * 0x7FFF).astype(int)
            col = np.floor(colour_data[i] * 0xFF).astype(int)
            tex_coord = np.floor(tex_coord * 0xFFFF).astype(int)

            packed_vertex = struct.pack('<3i4B3h2Hxx',
                                        pos[0], pos[1], pos[2], 
                                        col[0], col[1], col[2], tex_id,
                                        nor[0], nor[1], nor[2],
                                        tex_coord[0], tex_coord[1])
            
            out_vertices.append(packed_vertex)

        # flip winding
        for i in range(0, len(out_vertices), 3):
            tmp = out_vertices[i]
            out_vertices[i] = out_vertices[i + 1]
            out_vertices[i + 1] = tmp

        primitives.append(out_vertices)

    return primitives

def handle_node(filename, json_data, json_node, parent_mat = None):
    meshes = []

    rotation = [0, 0, 0, 1]
    scale = [1, 1, 1]
    translation = [0, 0, 0]

    if 'rotation' in json_node:
        rotation = json_node['rotation']

    # xyzw -> wxyz
    rotation.insert(0, rotation[3])
    rotation.pop()

    if 'scale' in json_node:
        scale = json_node['scale']

    if 'translation' in json_node:
        translation = json_node['translation']

    # TODO
    if 'matrix' in json_node:
        print(f'unhandled transform for node {json_node["name"]}')
        print(json_node)

    mat = affines.compose(translation, quaternions.quat2mat(rotation), scale)

    if parent_mat is not None:
        mat = parent_mat @ mat

    if 'mesh' in json_node:
        meshes = meshes + handle_mesh(filename, json_data, json_data['meshes'][json_node['mesh']], mat)

    # children
    if 'children' in json_node:
        for node_id in json_node['children']:
            meshes = meshes + handle_node(filename, json_data, json_data['nodes'][node_id], mat)

    return meshes

# FIXME: proper args
in_filename = sys.argv[1]
out_filename = sys.argv[2]

json_data = json.load(open(in_filename))

scene = json_data['scenes'][json_data['scene']]

meshes = []

for node_id in scene['nodes']:
    meshes = meshes + handle_node(in_filename, json_data, json_data['nodes'][node_id])

out_file = open(out_filename, 'wb')

# header, num meshes
out_file.write(struct.pack('<4sI', b'BL3D', len(meshes)))

for mesh in meshes:
    # num verts
    out_file.write(struct.pack('<I', len(mesh)))

    for vertex in mesh:
        out_file.write(vertex) # pre-packed