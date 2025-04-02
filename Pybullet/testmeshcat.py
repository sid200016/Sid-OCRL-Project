from urdfpy import URDF
import meshcat
import meshcat.geometry as g
import numpy as np
import os

# Load URDF
robot = URDF.load(r"robot_arm_3/urdf/robot_arm_3.urdf")

# Start MeshCat
vis = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
vis.delete()

# Optional: Define a configuration (joint angles) — update as needed
# Assumes all continuous/revolute joints
joint_angles = {
    joint.name: 0.0 for joint in robot.actuated_joints
}
print(joint_angles)
joint_angles["j1_y"] = np.pi / 2  # example

# Compute forward kinematics at this config
link_poses = robot.link_fk(cfg=joint_angles)

# Load & visualize meshes
for link in robot.links:
    for i, visual in enumerate(link.visuals):
        if visual.geometry.mesh is None:
            continue

        mesh_filename = os.path.basename(visual.geometry.mesh.filename)
        mesh_path = os.path.join("robot_arm_3/meshes", mesh_filename)

        if mesh_filename.endswith(".stl"):
            obj_geom = g.StlMeshGeometry.from_file(mesh_path)
        elif mesh_filename.endswith(".obj"):
            obj_geom = g.ObjMeshGeometry.from_file(mesh_path)
        else:
            print(f"⚠️ Unsupported format: {mesh_filename}")
            continue

        # Compose transform: FK * local visual offset
        T_link = link_poses[link]         # 4x4 FK transform of link
        T_vis = visual.origin             # 4x4 offset of mesh from link frame
        T = T_link @ T_vis                # final mesh pose

        vis[f"{link.name}/{i}"].set_object(obj_geom)
        vis[f"{link.name}/{i}"].set_transform(T)

# Optional: Draw axes for each link
def draw_axes(vis, path, length=0.1, radius=0.002):
    # X axis – red
    vis[f"{path}/x"].set_object(g.Cylinder(length=length, radius=radius), g.MeshLambertMaterial(color=0xff0000))
    vis[f"{path}/x"].set_transform(g.transformations.rotation_matrix(np.pi/2, [0, 1, 0]) @
                                   g.transformations.translation_matrix([length/2, 0, 0]))

    # Y axis – green
    vis[f"{path}/y"].set_object(g.Cylinder(length=length, radius=radius), g.MeshLambertMaterial(color=0x00ff00))
    vis[f"{path}/y"].set_transform(g.transformations.rotation_matrix(np.pi/2, [-1, 0, 0]) @
                                   g.transformations.translation_matrix([0, length/2, 0]))

    # Z axis – blue
    vis[f"{path}/z"].set_object(g.Cylinder(length=length, radius=radius), g.MeshLambertMaterial(color=0x0000ff))
    vis[f"{path}/z"].set_transform(g.transformations.translation_matrix([0, 0, length/2]))

draw_axes = True
if draw_axes:
    axis_len = 0.05
    for link in robot.links:
        T_link = link_poses[link]
        #draw_axes(vis, f"axes/{link.name}", length=axis_len)
        vis[f"axes/{link.name}"].set_transform(T_link)

print("✅ MeshCat render complete — open http://127.0.0.1:7000/static")

# import pytinydiffsim as pd
# import pytinyopengl3 as gl
# import numpy as np
# import time

# print("🚀 Starting test script")

# urdf_path = "robot_arm_3/urdf/robot_arm_3.urdf"
# mesh_path = "robot_arm_3/urdf"
# texture_path = ""

# # Parse URDF
# try:
#     print("📦 Loading URDF...")
#     parser = pd.TinyUrdfParser()
#     urdf_struct = parser.load_urdf(urdf_path, False)
#     print("✅ URDF loaded")
# except Exception as e:
#     print("❌ URDF load failed:", e)
#     exit()

# # Create MB
# try:
#     print("🔧 Creating MultiBody...")
#     world = pd.TinyWorld()
#     mb = pd.TinyMultiBody(False)
#     urdf2mb = pd.UrdfToMultiBody2()
#     urdf2mb.convert2(urdf_struct, world, mb)
#     print("✅ MB created")
# except Exception as e:
#     print("❌ MB creation failed:", e)
#     exit()

# # Set joint values
# try:
#     print("⚙️ Setting joint configuration...")
#     q = mb.q
#     print("q", q.size)
#     # base_pose = [0, 0, 0.5, 0, 0, 0, 1]
#     # for i in range(7):
#     #     q[i] = base_pose[i]
#     joint_angles = [0, 0, 0, 0, 0, 0]
#     for i, angle in enumerate(joint_angles):
#         q[i] = angle
#     mb.set_q(q)
#     #mb.set_qd([0] * len(mb.qd))
#     print("q, ", q)
#     print("✅ Joint configuration set")
# except Exception as e:
#     print("❌ Setting joints failed:", e)
#     exit()

# # Set up OpenGL visualizer
# try:
#     print("🖼️  Starting OpenGL Visualizer...")
#     viz = gl.OpenGLUrdfVisualizer()
#     viz.path_prefix = gl.extract_path(mesh_path)
#     parser_gl = gl.UrdfParser()
#     visual_urdf = parser_gl.load_urdf(urdf_path)
#     viz.convert_visuals(visual_urdf, texture_path)
#     instances = viz.create_instances(visual_urdf, texture_path, 1)
#     print("✅ Visualizer ready")
# except Exception as e:
#     print("❌ OpenGL Visualizer setup failed:", e)
#     exit()

# # Render loop
# print("🔁 Entering render loop")
# while True:
#     pd.forward_kinematics(mb, mb.q, mb.qd)  # ensure transforms are up to date
#     #visual_transforms =  mb.m_base_X_visuals
#     #viz.sync_visual_transforms(instances, visual_transforms, 0, 2)
#     #viz.sync_visual_transforms(instances, mb.visual_transforms(), 0, 2)
#     viz.render()
#     time.sleep(0.01)

