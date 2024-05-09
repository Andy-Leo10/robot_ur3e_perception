import pyrealsense2 as rs

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
pipeline.start()

# Get the intrinsics of the depth camera
profile = pipeline.get_active_profile()
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
intrinsics = depth_profile.get_intrinsics()

fx = intrinsics.fx  # focal length in x
fy = intrinsics.fy  # focal length in y
cx = intrinsics.ppx  # optical center x
cy = intrinsics.ppy  # optical center y

print('fx:', fx, 'fy:', fy, 'cx:', cx, 'cy:', cy)

pipeline.stop()