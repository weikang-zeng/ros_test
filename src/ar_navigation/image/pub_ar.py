#!/usr/bin/env python3




import cv2
import apriltag

# Load the image
image = cv2.imread("ARtag1.pdf")

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Create an AprilTag detector
options = apriltag.DetectorOptions()
detector = apriltag.Detector(options)

# Detect AprilTags in the image
results = detector.detect(gray)

# Process the detected AprilTags
for r in results:
    # Get tag ID and pose
    tag_id = r.tag_id
    tag_pose = r.pose_t

    # Print tag information
    print("Tag ID:", tag_id)
    print("Tag Pose:")
    print(tag_pose)

    # Draw tag outline and ID on the image
    r.draw(image)

    # Display the image
    cv2.imshow("AprilTags", image)
    cv2.waitKey(0)

# Close the image display
#cv2.destroyAllWindows()