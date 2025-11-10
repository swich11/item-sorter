# use laptop camera for testing object detection

import numpy as np
import cv2
import threading
import time

from enum import Enum

# Constant Parameters 
# TODO: make project variables
MIN_BIN_AREA_THRESHOLD = 2000 # TO ADJUST

# Define object colours with their HSV ranges
# Simply need to add more colours here if needed no other code changes required
class ObjectColour(Enum):
    RED1    = ((0, 120, 120), (10, 255, 255))
    RED2    = ((170, 120, 120), (180, 255, 255))
    # GREEN   = ((35, 120, 120), (85, 255, 255))
    BLUE    = ((90, 110, 110), (140, 255, 255))
    # YELLOW  = ((15, 120, 120), (35, 255, 255))

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
        # default last pass values
        self.last_image = None
    
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
    def classify_shape(self, contour, depth_image):
        # Polygonal approximation (Does not work due to struggling differentiating the faces with contours)
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True) # May need to find face among approximations
        vertices = len(approx)
        area = cv2.contourArea(contour)

        shape = ObjectShape.UNKNOWN
            
        is_bin = (area > MIN_BIN_AREA_THRESHOLD)
        
        return shape, is_bin, approx
    
    def detect_objects(self, colour_img):
        # Initialize msgs
        objects = []
        num_detected = 0

        if colour_img is None:
            return None, None
        annotated = colour_img.copy()
        approx_img = np.zeros((self.VRES, self.HRES, 3), dtype=np.uint8)
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(colour_img, cv2.COLOR_BGR2HSV)

        # Define area thresholds # To be project parameters
        min_area = 1000
        max_area = 50000
        
        # Loop through each colour range and detect objects of that colour
        for colour_range in ObjectColour:
            mask = cv2.inRange(hsv_image, colour_range.lower, colour_range.upper)
            # MIGHT NEED TO ADD MORPHOLOGICAL OPERATIONS HERE
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
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
                    shape, is_bin, approx = self.classify_shape(contour, None)
                    cv2.drawContours(approx_img, [approx], -1, (255,255,255), 2)
                    num_detected += 1
                            
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
                        
                    # Show the mask image
                    cv2.imshow("Mask", mask)
                    cv2.imshow("Approx", approx_img)
                    cv2.waitKey(1)  # Wait for a brief moment to update the window

        # Sort detections by colour and shape for consistent ordering
        # detections.sort(key=lambda d: (d['colour'], d['shape']))
        return objects, annotated
    
    # Main loop to update values, and OPTIONALLY display video and depth feed
    def run(self, test=False):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()
        try:
            while True:
                # Grab latest images from RealSense
                ret, color_image = cap.read()
                if not ret:
                    print("Can't receive frame (stream end?). Exiting ...")
                    break
                
                cv2.imshow("Color Image", color_image)
                    
                time.sleep(0.001)
                                
                # Do the object detection to find keypoints
                
                objects, annotated_image = self.detect_objects(color_image)
                if objects is not None:
                    print(f"Detected {len(objects)} objects")
                    for obj in objects:
                        print(f" - ID: {obj['id']}, Colour: {obj['colour']}, Shape: {obj['shape']}, Position: {obj['position']}, Is Bin: {obj['is_bin']}")
                if annotated_image is not None:
                    cv2.imshow("Annotated", annotated_image)
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
