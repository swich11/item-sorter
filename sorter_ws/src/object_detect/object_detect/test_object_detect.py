# use laptop camera for testing object detection
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
import time
import cv2.aruco as aruco

from enum import Enum

# Constant Parameters 
# TODO: make project variables
MIN_BIN_AREA_THRESHOLD = 1800 # TO ADJUST
MIN_BIN_DIM_THRESHOLD = 0.04 # in meters
MARKER_SIZE = 0.025  # Marker size in meters

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    RED    = ((0, 100, 75), (10, 255, 255))
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
    SPHERE = 7
    CUBE = 8

    UNKNOWN = 0

object_info = {
    0: {"shape" : ObjectShape.UNKNOWN, "is_bin" : False, "tf_to_centre" : (0,0,0)},
    1: {"shape" : ObjectShape.SPHERE, "is_bin" : False, "tf_to_centre" : (0,0,0)},
    2: {"shape" : ObjectShape.CUBE, "is_bin" : False, "tf_to_centre" : (0,0,0)},
    3: {"shape" : ObjectShape.UNKNOWN, "is_bin" : False, "tf_to_centre" : (0,0,0)},
    4: {"shape" : ObjectShape.CYLINDER, "is_bin" : True, "tf_to_centre" : (0,0,0)},
    5: {"shape" : ObjectShape.RECTANGULAR_PRISM, "is_bin" : True, "tf_to_centre" : (0,0,0)}
}

class RealSenseD435i:
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
            return
        
        # config alignment primitive with color as its target stream:
        self.align = rs.align(rs.stream.color)
        
        # default last pass values
        self.colour_image = None
        self.depth_image = None
        
        # Initialize Aruco parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()

        # Approximate intrinsics
        self.camera_matrix = np.array([
            [self.intrinsics.fx,  0, self.intrinsics.ppx],
            [0,  self.intrinsics.fy, self.intrinsics.ppy],
            [0,  0,   1]
        ])
        self.dist_coeffs = np.array(self.intrinsics.coeffs)
        
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
        # TODO: Change to use qr scaleing method to determine bin size / just qr says if it is a bin
        is_bin = cv2.contourArea(contour) > MIN_BIN_AREA_THRESHOLD       
        # is_bin = self.is_bin_helper(contour, depth_image, mask)
                
        return shape, is_bin, approx
    
    # Helper to determine if contour likely represents a bin
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
    
    def get_colour_masks(self):
        if self.colour_image is None:
            return None, None
        masks = {}
        complete_mask = np.zeros(self.colour_image.shape[:2], dtype=np.uint8)
        hsv_image = cv2.cvtColor(self.colour_image, cv2.COLOR_BGR2HSV)

        for colour_range in ObjectColour:
            if colour_range == ObjectColour.RED2:
                continue
            # Create a mask for the current colour range
            mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            if colour_range == ObjectColour.RED:
                # Combine masks for RED and RED2
                mask2 = cv2.inRange(hsv_image, ObjectColour.RED2.lower, ObjectColour.RED2.upper)
                mask = cv2.bitwise_or(mask, mask2)
            
            # Apply morphological operations to clean up the mask
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=3)
            # mask = cv2.GaussianBlur(mask, (5, 5), 0)
            
            masks[colour_range] = mask
            complete_mask = cv2.bitwise_or(complete_mask, mask)
        
        return masks, complete_mask
    
    def get_colour_contours(self, masks):
        all_contours = []
        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500 and area < 50000:
                    all_contours.append((colour, contour))
        return all_contours
    
    def find_aruco(self, contour = None):
        if self.colour_image is None:
            return None, None, None, None
        
        x,y = 0,0
        if contour is not None:
            x, y, w_box, h_box = cv2.boundingRect(contour)
            roi = self.colour_image[y:y+h_box, x:x+w_box]
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)         
        else:
            gray = cv2.cvtColor(self.colour_image, cv2.COLOR_BGR2GRAY)
    
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if contour is not None:
            # Adjust corners to full image coordinates
            for pts in corners:
                pts += np.array([[x, y]])
        
        tvecs, rvecs = None, None
        centers = []
        if ids is not None:
            for i in range(len(ids)):
                pts = corners[i][0]
                Cx = int(pts[:,0].mean())
                Cy = int(pts[:,1].mean())
                centers.append((Cx, Cy))
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, self.camera_matrix, self.dist_coeffs
            )
        
        return ids, tvecs, rvecs, centers
    
    def get_aruco_info(self, id):
        if id is None:
            return ObjectShape.UNKNOWN, False, (0,0,0)
        
        id_int = int(id)
        if id_int in object_info:
            info = object_info[id_int]
            return info["shape"], info["is_bin"], info["tf_to_centre"]
        else:
            return ObjectShape.UNKNOWN, False, (0,0,0)
    
    def detect_objects(self):
        objects = []
        num_detected = 0

        if self.colour_image is None:
            return None, None, None
        annotated = self.colour_image.copy()
        
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(self.colour_image, cv2.COLOR_BGR2HSV)

        # TODO: Tune these values as needed
        # Define area thresholds # To be project parameters
        min_area = 100
        max_area = 100000
        
        # Get masks for each colour range
        masks, complete_mask = self.get_colour_masks()
        
        # Get all contours from all colour masks
        contours = self.get_colour_contours(masks)  

        # Loop through each contour to classify and locate objects
        for colour_range, contour in contours:
            # detect aruco marker 
            ids, tvecs, rvecs, centers = self.find_aruco(contour)
            
            if ids is not None:
                # ArUco detected, use it to classify shape
                for idx, id in enumerate(ids):
                    shape, is_bin, transform = self.get_aruco_info(id)
                    
                    if centers:
                        cX, cY = centers[idx]
                    else:
                        cX, cY = 0, 0
                                
                    num_detected += 1        
                    # Create and append marker for visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)
                    bin_str = "BIN" if is_bin else "OBJ"
                    global_position = self.pixel_to_global(self.depth_image, [cX, cY])
                    cv2.putText(annotated, f"A-{colour_range.name}-{shape.name}-{bin_str}-({(global_position)})", (cX + 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                    objects.append({
                        'id': num_detected,
                        'colour': colour_range.name,
                        'shape': shape.name,
                        'is_bin': is_bin,
                        'img_position': (cX,cY),
                        'global_position': global_position
                        })

                    # Draw the center on the original image for opencv visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)
            
            else:
                # No ArUco detected, classify shape normally    
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    # Calculate the center of the contour object
                    cX = int(moments['m10'] / moments['m00'])
                    cY = int(moments['m01'] / moments['m00'])
                    
                    num_detected += 1        
                    shape, is_bin, _ = self.classify_shape(contour, None)
                    # Create and append marker for visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)
                    bin_str = "BIN" if is_bin else "OBJ"
                    global_position = self.pixel_to_global(self.depth_image, [cX, cY])
                    cv2.putText(annotated, f"{colour_range.name}-{shape.name}-{bin_str}-({global_position})", (cX + 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                    objects.append({
                        'id': num_detected,
                        'colour': colour_range.name,
                        'shape': shape.name,
                        'is_bin': is_bin,
                        'img_position': (cX,cY),
                        'global_position': global_position
                        })

                    # Draw the center on the original image for opencv visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)
        
                        
        # Show the mask image
        if complete_mask is not None:
            cv2.imshow("Mask", complete_mask)
            cv2.waitKey(1)  # Wait for a brief moment to update the window

        # Sort detections by colour and shape for consistent ordering
        # detections.sort(key=lambda d: (d['colour'], d['shape']))
        return objects, annotated, complete_mask
    
    # Main loop to update values, and OPTIONALLY display video and depth feed
    def run(self, test=False):
        try:
            while self.is_connected():
                tic = time.process_time() if test else 0

                # Grab latest images from RealSense
                self.depth_image, self.colour_image, depth_frame = self.get_frames()
                if self.depth_image is None or self.colour_image is None:
                    continue
                    
                time.sleep(0.001)
                                
                # Do the object detection to find keypoints
                
                objects, annotated_image, complete_mask = self.detect_objects()
                if objects is not None:
                    print(f"Detected {len(objects)} objects : ")
                    for obj in objects:
                        print(f" - ID: {obj['id']}, Colour: {obj['colour']}, Shape: {obj['shape']}, Image Position: {obj['img_position']}, Global Position: {obj['global_position']}, Is Bin: {obj['is_bin']}")
                    print("\n")
                    
                if annotated_image is not None and complete_mask is not None:
                    cv2.imshow("Annotated", annotated_image)
                    cv2.imshow("Complete Mask", complete_mask)
                    cv2.waitKey(1)

                # Adding visualisation markers and depth image for demo/testing
                if test and self.colour_image is not None:
                    print(f"\nRun Loop Time  (PRE DISPLAY): {1000*(time.process_time()-tic)}ms\n")
                    
                    # Convert depth to color
                    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                    depth_colormap_dim = depth_colormap.shape
                    color_colormap_dim = self.colour_image.shape

                    # If depth and color resolutions are different, resize color image to match depth image for display
                    if depth_colormap_dim != color_colormap_dim:
                        resized_color_image = cv2.resize(self.colour_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                        images = np.hstack((resized_color_image, depth_colormap))
                    else:
                        images = np.hstack((self.colour_image, depth_colormap))

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
