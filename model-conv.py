import json
import math
import numpy as np
from pathlib import Path
import struct
import sys

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


def handle_mesh(filename, json_data, json_mesh):
    # this is converting glTF primitives to our "meshes"...
    primitives = []

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

        index_data = get_accessor_data(filename, json_data, index_accessor, False)

        # triangles
        for i in index_data:
            # convert data
            pos = np.floor(position_data[i] * (1 << 16)).astype(int)
            nor = np.floor(normal_data[i] * 0x7FFF).astype(int)
            col = np.floor(colour_data[i] * 0xFF).astype(int)

            packed_vertex = struct.pack('<3i4B3hxx',
                                        pos[0], pos[1], pos[2], 
                                        col[0], col[1], col[2], col[3],
                                        nor[0], nor[1], nor[2])
            
            out_vertices.append(packed_vertex)

        primitives.append(out_vertices)

    return primitives

def handle_node(filename, json_data, json_node):
    meshes = []

    # TODO
    if 'matrix' in json_node or 'rotation' in json_node or 'scale' in json_node or 'translation' in json_node:
        print(f'unhandled transform for node {json_node["name"]}')

    if 'mesh' in json_node:
        meshes = meshes + handle_mesh(filename, json_data, json_data['meshes'][json_node['mesh']])

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