import bpy
import math
from mathutils import Vector

# Function to generate vertices of a 4D tesseract
def generate_tesseract_vertices(edge_length=2, center=(0, 0, 0, 0)):
    half_length = edge_length / 2
    vertices = []
    # Generate all 16 vertices of the 4D hypercube
    for i in range(16):
        vertex = [
            center[0] + half_length * (1 if (i & 1) else -1),
            center[1] + half_length * (1 if (i & 2) else -1),
            center[2] + half_length * (1 if (i & 4) else -1),
            center[3] + half_length * (1 if (i & 8) else -1),
        ]
        vertices.append(vertex)
    return vertices

# Function to apply 4D rotation matrix using Rodrigues' rotation formula
def rotate_around_axis_4d(vertex, axis_point1, axis_point2, angle):
    # Calculate the axis vector (v) in 4D
    v = Vector((axis_point2[i] - axis_point1[i] for i in range(4)))
    v.normalize()  # Normalize the axis vector
    
    # Translate the vertex so that axis_point1 is the origin
    translated_vertex = Vector((vertex[i] - axis_point1[i] for i in range(4)))

    # Compute dot product and cross products in 4D
    dot_product = translated_vertex.dot(v)
    cross_product = Vector((
        translated_vertex[1] * v[2] - translated_vertex[2] * v[1] + translated_vertex[3] * v[0],
        translated_vertex[2] * v[3] - translated_vertex[3] * v[2] + translated_vertex[0] * v[1],
        translated_vertex[3] * v[0] - translated_vertex[0] * v[3] + translated_vertex[1] * v[2],
        translated_vertex[0] * v[1] - translated_vertex[1] * v[0] + translated_vertex[2] * v[3]
    ))

    # Calculate the rotated vertex using Rodrigues' rotation formula
    rotated_vertex = (
        translated_vertex * math.cos(angle) +
        cross_product * math.sin(angle) +
        v * dot_product * (1 - math.cos(angle))
    )
    
    # Translate back to the original coordinate system
    rotated_vertex = Vector((rotated_vertex[i] + axis_point1[i] for i in range(4)))
    return rotated_vertex

# Function to project 4D points to 3D
def project_to_3d(vertices, projection_point=(0, 0, 0, 2)):
    projected_vertices = []
    px, py, pz, pw = projection_point

    for vertex in vertices:
        vx, vy, vz, vw = vertex
        # Perspective projection formula from 4D to 3D
        w = vw - pw
        if w != 0:  # Prevent division by zero
            projected_vertex = (
                (vx - px) / w,
                (vy - py) / w,
                (vz - pz) / w,
            )
            projected_vertices.append(projected_vertex)
        else:
            # Handle case where projection is undefined
            projected_vertices.append((vx, vy, vz))
    return projected_vertices

# Function to create vertices in Blender
def create_mesh(vertices, edges, name="Tesseract Projection"):
    # Remove previous mesh if exists
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name], do_unlink=True)
    
    mesh = bpy.data.meshes.new(name=name)
    mesh.from_pydata(vertices, edges, [])
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj

# Generate edges of a tesseract (4D hypercube)
def generate_tesseract_edges(vertices_4d):
    edges = []
    for i in range(len(vertices_4d)):
        for j in range(i + 1, len(vertices_4d)):
            # If the vertices differ by exactly one coordinate
            if sum(a != b for a, b in zip(vertices_4d[i], vertices_4d[j])) == 1:
                edges.append((i, j))
    return edges

# Generate 4D tesseract vertices
vertices_4d = generate_tesseract_vertices()

# Generate edges for the tesseract
edges = generate_tesseract_edges(vertices_4d)

# Create a mesh for the initial tesseract
projected_vertices_3d = project_to_3d(vertices_4d)
obj = create_mesh(projected_vertices_3d, edges)

# Animation setup: rotate around an axis defined by two 4D points
total_frames = 250
angle_per_frame = 2 * math.pi / total_frames  # Full circle in 250 frames
axis_point1 = Vector((1, 1, 1, 1))  # Example 4D point for axis
axis_point2 = Vector((-1, -1, -1, -1))  # Another example 4D point for axis

# Function to update mesh for each frame
def update_tesseract(scene):
    frame = scene.frame_current
    angle = frame * angle_per_frame
    # Rotate all 4D vertices around the axis
    rotated_vertices_4d = [rotate_around_axis_4d(vertex, axis_point1, axis_point2, angle) for vertex in vertices_4d]
    # Project rotated vertices to 3D
    projected_vertices_3d = project_to_3d(rotated_vertices_4d)
    # Update the mesh with new positions
    obj.data.clear_geometry()  # Clear previous geometry
    obj.data.from_pydata(projected_vertices_3d, edges, [])
    obj.data.update()

# Add the handler to the frame change event
bpy.app.handlers.frame_change_pre.clear()  # Clear any existing handlers
bpy.app.handlers.frame_change_pre.append(update_tesseract)

print("Tesseract rotation around axis animation created!")
