# use laptop camera for testing object detection
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time
import open3d as o3d

from enum import Enum

# Constant Parameters 
# TODO: make project variables
MIN_BIN_AREA_THRESHOLD = 1800 # TO ADJUST
MIN_BIN_DIM_THRESHOLD = 0.04 # in meters

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    RED1    = ((0, 100, 75), (10, 255, 255))
    RED2    = ((170, 100, 75), (180, 255, 255))
    # GREEN   = ((35, 40, 40), (85, 255, 255))
    BLUE    = ((100, 200, 30), (110, 255, 255))
    # YELLOW  = ((15, 100, 100), (35, 255, 255))

    @property
    def lower(self):
        return np.array(self.value[0], dtype=np.uint8)

    @property
    def upper(self):
        return np.array(self.value[1], dtype=np.uint8)

# Define object shapes
class ObjectShape(Enum):
    CYLINDER = 1
    SQUARE_PRISM = 2
    TRIANGULAR_PRISM = 3
    RECTANGULAR_PRISM = 4
    STAR_PRISM = 5
    HEXAGONAL_PRISM = 6

    UNKNOWN = 0

class RealSenseD435i:
    __slots__ = ('context', 'pipeline', 'config', 'estimator', 'last_image', 'last_distance', 'align', 'num_people', 'last_position', 'last_angle',
                 'intrinsics', 'person_width', 'is_person_hug', 'cv', 'shared', 'lock', 'estimator_thread', 'estimator_thread_on', 'tracker', 'tracker_thread', 'tracker_thread_on' 
        )
    # CAMERA SPECS
    HFOV = 69.4     # Horizontal FOV
    VFOV = 42.5     # Vertical FOV
    
    # FEED SETTINGS
    HRES = 640      # Horizontal Resolution
    VRES = 480      # Vertical Resolution
    FPS = 60        # FPS
    
    def __init__(self):
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable depth stream (640x480 resolution, 30 FPS, 16-bit format)
        self.config.enable_stream(rs.stream.depth, RealSenseD435i.HRES, RealSenseD435i.VRES, rs.format.z16, RealSenseD435i.FPS)
        # Enable color stream (640x480 resolution, 30 FPS, BGR format)
        self.config.enable_stream(rs.stream.color, RealSenseD435i.HRES, RealSenseD435i.VRES, rs.format.bgr8, RealSenseD435i.FPS)
        
        self.context = rs.context()
        if self.is_connected():
            # Start streaming from the camera
            pipeline_profile = self.pipeline.start(self.config)
        
            # Get Intrinsics
            color_stream = pipeline_profile.get_stream(rs.stream.color)
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            # Setup camera settings
            sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]  # [1] = RGB sensor
            sensor.set_option(rs.option.enable_auto_exposure, True)
            # sensor.set_option(rs.option.exposure, 150)  # or tune to your lighting

            sensor.set_option(rs.option.enable_auto_white_balance, True)
            # sensor.set_option(rs.option.white_balance, 3500)  # or a value suited for your environment
        else:
            print("No RealSense Detected")
        
        # config alignment primitive with color as its target stream:
        self.align = rs.align(rs.stream.color)
        
        # default last pass values
        self.last_image = None
        
    # Helper check if a realsense device is connected
    def is_connected(self):
        devices = self.context.query_devices()
        return (len(devices) > 0)
    
    # Get latest depth and color frames.
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        aligned_depth_frame = frames.get_depth_frame()

        if not aligned_depth_frame or not color_frame:
            return None, None, None
            
        # Convert frames to NumPy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image, aligned_depth_frame
    
    # Stop the camera pipeline
    def stop(self):
        self.pipeline.stop()
    
    # return last recorded distance
    def distance_to_subject(self):
        return float(self.last_distance)
    
    # Helper gets points position respective to colour camera    
    def pixel_to_global(self, depth_image, pixel_pt):
        if depth_image is not None and self.intrinsics is not None and pixel_pt[0]<self.intrinsics.height and pixel_pt[1]<self.intrinsics.width:
            [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
            print("fine")
            return [x, y, z]
        elif pixel_pt[0]>self.intrinsics.width or pixel_pt[1]>self.intrinsics.height:
            print("Pixel out of bounds")
            return None
        else:
            return None
    
    # getter for last image
    def get_image(self):
        return self.last_image
    
   # OLD shape classifier
    def classify_shape_polyapprox(self, contour):
        # Polygonal approximation (Does not work due to struggling differentiating the faces with contours)
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True) # May need to find face among approximations
        vertices = len(approx)
        area = cv2.contourArea(contour)

        shape = ObjectShape.UNKNOWN
        if vertices == 3:
            shape = ObjectShape.TRIANGULAR_PRISM
        elif vertices == 4:
            shape = ObjectShape.SQUARE_PRISM
        elif 5 <= vertices <= 6:
            shape = ObjectShape.HEXAGONAL_PRISM
            
        is_bin = (area > MIN_BIN_AREA_THRESHOLD)
        
        return shape, is_bin, approx
    
    # TODO: Implement shape classification
    # Point cloud method to classify shape or 
    # ML based method could be implemented here
    def classify_shape(self, contour, depth_image):
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        vertices = len(approx)
        area = cv2.contourArea(contour)
        zero_mask = np.zeros((self.intrinsics.height, self.intrinsics.width), dtype=np.uint8)
        mask = cv2.drawContours(zero_mask, [contour], -1, (0, 255, 0), -1)
        
        shape = ObjectShape.CYLINDER
        # shape, _, _, pcd = self.fit_shape(depth_image, mask)
        # o3d.visualization.draw_geometries([pcd])
        # wait = input("Press Enter to continue...")
        # TODO: Change to use qr scaleing method to determine bin size / just qr says if it is a bin
        is_bin = cv2.contourArea(contour) > MIN_BIN_AREA_THRESHOLD       
        # is_bin = self.is_bin_helper(contour, depth_image, mask)
                
        return shape, is_bin, approx
    
    # Helper to determine if contour likely represents a bin
    # is vulnerable to occlusion and angle of view
    # good enough for before using point cloud method for more accurate info
    def is_bin_helper(self, contour, depth_img, mask):
        # Get average depth inside the contour
        depth_values = depth_img[mask == 255]
        depth_values = depth_values[depth_values > 0]
        if len(depth_values) == 0:
            return False
        Z = np.median(depth_values) * 0.001  # convert mm → m

        # Get bounding box in pixels
        x, y, w, h = cv2.boundingRect(contour)

        # Convert pixel distance to meters at depth Z
        width_m = (w / self.intrinsics.fx) * Z
        height_m = (h / self.intrinsics.fy) * Z
        
        return (width_m > MIN_BIN_DIM_THRESHOLD or height_m > MIN_BIN_DIM_THRESHOLD)
    
    # Convert masked depth image to an Open3D point cloud.
    def mask_to_pointcloud(self, depth_frame, mask):
        indices = np.where(mask > 0)
        z = depth_frame[indices] / 1000.0  # mm → m
        u = indices[1]
        v = indices[0]
        x = (u - self.intrinsics.ppx) * z / self.intrinsics.fx
        y = (v - self.intrinsics.ppy) * z / self.intrinsics.fy

        points = np.vstack((x, y, z)).T
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    # Estimate number of flat faces using convex hull.
    def count_hull_faces(self, pcd):
        try:
            hull, _ = pcd.compute_convex_hull()
            triangles = np.asarray(hull.triangles)
            vertices = np.asarray(hull.vertices)
            verts = np.asarray(hull.vertices)
            mesh = hull

            # Compute normals for each triangle
            triangle_normals = np.asarray(mesh.triangle_normals)

            # Cluster normals by angle similarity
            clusters = []
            threshold = np.deg2rad(15)  # within 15 degrees
            for n in triangle_normals:
                matched = False
                for c in clusters:
                    if np.arccos(np.clip(np.dot(n, c), -1.0, 1.0)) < threshold:
                        matched = True
                        break
                if not matched:
                    clusters.append(n)
            return len(clusters)
        except Exception:
            return 0

    ### Fit geometric model to classify prism shape.
    def fit_shape(self, depth_frame, mask):
        pcd = self.mask_to_pointcloud(depth_frame, mask)
        if len(pcd.points) < 100:
            return ObjectShape.UNKNOWN, None, 0, None

        # Clean noise
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)

        # Get bounding box info
        bbox = pcd.get_oriented_bounding_box()
        extents = bbox.extent
        aspect = np.sort(extents) / np.max(extents)

        # Compute convex hull and count flat faces
        n_faces = self.count_hull_faces(pcd)

        # --- Infer shape based on faces + proportions ---
        if n_faces == 0:
            shape = ObjectShape.UNKNOWN
        elif n_faces <= 6:
            shape = ObjectShape.TRIANGULAR_PRISM
        elif n_faces <= 8:
            shape = ObjectShape.SQUARE_PRISM
        elif n_faces <= 12:
            shape = ObjectShape.HEXAGONAL_PRISM
        else:
            shape = ObjectShape.CYLINDER
            
        # Catch flat objects
        if aspect[0] < 0.2:
            shape = ObjectShape.UNKNOWN

        center = bbox.center
        return shape, center, n_faces, pcd
    
    def detect_objects(self, colour_img, depth_img):
        # print(self.intrinsics)
        # print(depth_img)
        # Initialize msgs
        objects = []
        num_detected = 0

        if colour_img is None:
            return None, None
        annotated = colour_img.copy()
        
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(colour_img, cv2.COLOR_BGR2HSV)

        # TODO: Tune these values as needed
        # Define area thresholds # To be project parameters
        min_area = 100
        max_area = 100000
        
        # Loop through each colour range and detect objects of that colour
        for colour_range in ObjectColour:
            if colour_range == ObjectColour.RED2:
                continue
            mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            if colour_range == ObjectColour.RED1:
                mask2 = cv2.inRange(hsv_image, ObjectColour.RED2.lower, ObjectColour.RED2.upper)
                mask = cv2.bitwise_or(mask, mask2)
            # mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            # MIGHT NEED TO ADD MORPHOLOGICAL OPERATIONS HERE
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
            # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area or area > max_area:
                    continue
            
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    # Calculate the center of the object
                    cX = int(moments['m10'] / moments['m00'])
                    cY = int(moments['m01'] / moments['m00'])
                    
                    # Convert the pixel coordinates to 3D world coordinates
                    global_position = self.pixel_to_global(depth_img, [cX, cY])
                    if global_position is not None:
                        # Append the object to the list
                        shape, is_bin, approx = self.classify_shape(contour, depth_img)
                        num_detected += 1
                            
                        # Create and append marker for visualization
                        cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)
                        cv2.circle(annotated, (cX, cY), 3, colour_range.lower.tolist(), -1)
                        cv2.putText(annotated, f"{colour_range.name}-{shape.name}-{is_bin}-({global_position})", (cX + 10, cY - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        # cv2.putText(annotated, f"{shape.name}-{is_bin}-({global_position})", (cX + 10, cY - 10),
                                    # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                        objects.append({
                            'id': num_detected,
                            'colour': colour_range.name,
                            'shape': shape.name,
                            'is_bin': is_bin,
                            'position': global_position
                        })
                        
                    # Show the mask image
                    cv2.imshow("Mask", mask)
                    cv2.waitKey(1)  # Wait for a brief moment to update the window

        # Sort detections by colour and shape for consistent ordering
        # detections.sort(key=lambda d: (d['colour'], d['shape']))
        return objects, annotated
    
    # Main loop to update values, and OPTIONALLY display video and depth feed
    def run(self, test=False):
        try:
            while self.is_connected():
                tic = time.process_time() if test else 0

                # Grab latest images from RealSense
                depth_image, color_image, depth_frame = self.get_frames()
                if depth_image is None or color_image is None:
                    continue
                    
                time.sleep(0.001)
                                
                # Do the object detection to find keypoints
                
                objects, annotated_image = self.detect_objects(color_image, depth_image)
                if objects is not None:
                    print(f"Detected {len(objects)} objects : ")
                    for obj in objects:
                        print(obj)
                    print("\n")
                if annotated_image is not None:
                    cv2.imshow("Annotated", annotated_image)
                    cv2.waitKey(1)

                # Adding visualisation markers and depth image for demo/testing
                if test and color_image is not None:
                    print(f"\nRun Loop Time  (PRE DISPLAY): {1000*(time.process_time()-tic)}ms\n")
                    
                    # Convert depth to color
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                    depth_colormap_dim = depth_colormap.shape
                    color_colormap_dim = color_image.shape

                    # If depth and color resolutions are different, resize color image to match depth image for display
                    if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, depth_colormap))
                    else:
                        images = np.hstack((color_image, depth_colormap))

                    # Display the depth and color images
                    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense', images)

                    print(f"\nRun Loop Time (POST DISPLAY): {1000*(time.process_time()-tic)}ms\n")
                    
                    # Exit the loop when 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                time.sleep(0.1)
        finally:
            if self.is_connected():
                self.stop() 
            cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = RealSenseD435i()
    camera.run(True)
