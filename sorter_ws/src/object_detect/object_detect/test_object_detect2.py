# use laptop camera for testing object detection

import numpy as np
import cv2
import cv2.aruco as aruco
import threading
import time

from enum import Enum

# Constant Parameters 
# TODO: make project variables
MIN_BIN_AREA_THRESHOLD = 2000 # TO ADJUST
MARKER_SIZE = 0.025  # Size of the ArUco marker in meters


# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
# HOME AND LAB REQUIRE DIFFERENT NUMBERS
class ObjectColour(Enum):
    RED    = ((0, 100, 75), (10, 255, 255))
    RED2    = ((170, 100, 75), (180, 255, 255))
    # GREEN   = ((35, 40, 40), (85, 255, 255))
    BLUE    = ((100, 150, 30), (110, 255, 255))
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
        # default last pass values
        self.colour_image = None
        
        # Initialize Aruco parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        self.aruco_params = aruco.DetectorParameters()
        
        # Approximate intrinsics (good enough for testing)
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        if not cap.isOpened():
            print("❌ Could not open camera")
            exit()
        h, w = frame.shape[:2]
        self.camera_matrix = np.array([
            [w,  0, w/2],
            [0,  w, h/2],
            [0,  0,   1]
        ])
        self.dist_coeffs = np.zeros((5,))
    
    # TODO: Implement shape classification
    def classify_shape(self, contour, depth_image):
        # Polygonal approximation (Does not work due to struggling differentiating the faces with contours)
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True) # May need to find face among approximations
        vertices = len(approx)
        area = cv2.contourArea(contour)

        shape = ObjectShape.UNKNOWN
            
        is_bin = (area > MIN_BIN_AREA_THRESHOLD)
        
        return shape, is_bin, approx
    
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
        if self.colour_image is None:
            return None, None, None
        
        annotated = self.colour_image.copy()
        approx_img = np.zeros((self.VRES, self.HRES, 3), dtype=np.uint8)
        num_detected = 0
        objects = []
        
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
                    cv2.putText(annotated, f"A-{colour_range.name}-{shape.name}-{bin_str}-({(cX,cY)})", (cX + 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                    objects.append({
                        'id': num_detected,
                        'colour': colour_range.name,
                        'shape': shape.name,
                        'is_bin': is_bin,
                        'position': (cX,cY)
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
                    cv2.putText(annotated, f"{colour_range.name}-{shape.name}-{bin_str}-({(cX,cY)})", (cX + 10, cY - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                    objects.append({
                        'id': num_detected,
                        'colour': colour_range.name,
                        'shape': shape.name,
                        'is_bin': is_bin,
                        'position': (cX,cY)
                        })

                    # Draw the center on the original image for opencv visualization
                    cv2.circle(annotated, (cX, cY), 5, (0, 0, 255), -1)

        return objects, annotated, complete_mask
    
    # Main loop to update values, and OPTIONALLY display video and depth feed
    def run(self, test=False):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        try:
            while True:
                # Grab latest images from RealSense
                ret, self.colour_image = cap.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break
                
                cv2.imshow("Color Image", self.colour_image)
                    
                time.sleep(0.001)
                                
                # Do the object detection to find keypoints
                
                objects, annotated_image, complete_mask = self.detect_objects()
                if objects is not None:
                    print(f"Detected {len(objects)} objects")
                    for obj in objects:
                        print(f" - ID: {obj['id']}, Colour: {obj['colour']}, Shape: {obj['shape']}, Position: {obj['position']}, Is Bin: {obj['is_bin']}")
                if annotated_image is not None and complete_mask is not None:
                    cv2.imshow("Annotated", annotated_image)
                    cv2.imshow("Complete Mask", complete_mask)
                    cv2.waitKey(1)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                time.sleep(0.1)
        finally:
            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = RealSenseD435i()
    camera.run(True)
