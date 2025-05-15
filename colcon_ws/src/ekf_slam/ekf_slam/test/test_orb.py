#!/usr/bin/env python3
import numpy as np
import cv2

img1 = cv2.imread('desert1.png')
img2 = cv2.imread('desert2.png')

def measurement(img1, img2):
    orb = cv2.ORB_create(nfeatures = 64)

    gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    keypoints1, des1 = orb.detectAndCompute(gray, None)

    pts = np.array([kp.pt for kp in keypoints1])
    responses = np.array([kp.response for kp in keypoints1])
    sorted_ind = np.argsort(-responses)

    pts1 = pts[sorted_ind]
    responses1 = responses[sorted_ind]
    descriptors1 = des1[sorted_ind]
    keypoints1 = [keypoints1[i] for i in sorted_ind]

    gray  = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    keypoints2, des2 = orb.detectAndCompute(gray, None)
    pts = np.array([kp.pt for kp in keypoints2])
    responses = np.array([kp.response for kp in keypoints2])
    sorted_ind = np.argsort(-responses)

    pts2 = pts[sorted_ind]
    responses2 = responses[sorted_ind]
    descriptors2 = des2[sorted_ind]
    keypoints2 = [keypoints2[i] for i in sorted_ind]

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    matches = sorted(matches, key=lambda x: x.distance)
    # print(matches)
    # print("Matches:", len(matches))
    # print("Keypoints1:", len(keypoints1))

    # Apply RANSAC filtering
    pts1_match = np.float32([ keypoints1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
    pts2_match = np.float32([ keypoints2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

    H, mask = cv2.findHomography(pts1_match, pts2_match, cv2.RANSAC, 5.0)
    inlier_matches = [m for i,m in enumerate(matches) if mask[i]]

    final_img = cv2.drawMatches(img1, keypoints1, 
    img2, keypoints2, inlier_matches,None)

    # After matching, do update of those keypoints that are matched. 

    return final_img
 
# final_img = cv2.resize(final_img, (1000,650))

# Show the final image
final_img = measurement(img1, img2)
cv2.imshow("Matches", final_img)
cv2.waitKey()
cv2.destroyAllWindows()
# detect new keypoints
"""
detect new image
descriptors = orb.detectAndCompute(gray, None)
keypoints, des = orb.detectAndCompute(gray, None)
matches = bf.match(descriptors, des)
matches = sorted(matches, key=lambda x: x.distance)

"""

# # img = cv2.drawKeypoints(img1;, None)
# # cv2.imshow('ORB Keypoints', img)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()

# #!/usr/bin/env python3
# import numpy as np
# import cv2

# img1 = cv2.imread('desert1.png')
# img2 = cv2.imread('desert2.png')
# orb = cv2.ORB_create(scaleFactor=1.2, nlevels=8, edgeThreshold=15, patchSize=31, nfeatures=2000)

# # Process first image
# gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
# keypoints1, des1 = orb.detectAndCompute(gray, None)
# responses = np.array([kp.response for kp in keypoints1])
# sorted_ind = np.argsort(-responses)
# keypoints1 = [keypoints1[i] for i in sorted_ind]
# descriptors1 = des1[sorted_ind]

# # Process second image
# gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
# keypoints2, des2 = orb.detectAndCompute(gray, None)
# responses = np.array([kp.response for kp in keypoints2])
# sorted_ind = np.argsort(-responses)
# keypoints2 = [keypoints2[i] for i in sorted_ind]
# descriptors2 = des2[sorted_ind]

# # Match
# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# matches = bf.match(descriptors1, descriptors2)
# matches = sorted(matches, key=lambda x: x.distance)

# print("Matches:", len(matches))
# print("Keypoints1:", len(keypoints1))

# # Apply RANSAC filtering
# pts1_match = np.float32([ keypoints1[m.queryIdx].pt for m in matches ]).reshape(-1,1,2)
# pts2_match = np.float32([ keypoints2[m.trainIdx].pt for m in matches ]).reshape(-1,1,2)

# H, mask = cv2.findHomography(pts1_match, pts2_match, cv2.RANSAC, 5.0)
# inlier_matches = [m for i,m in enumerate(matches) if mask[i]]

# print("Inlier Matches after RANSAC:", len(inlier_matches))

# # Draw matches
# final_img = cv2.drawMatches(img1, keypoints1, img2, keypoints2, inlier_matches, None)

# cv2.imshow("Matches", final_img)
# cv2.waitKey()
# cv2.destroyAllWindows()
