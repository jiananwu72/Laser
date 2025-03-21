import cv2
import numpy as np
import glob

def FindCalibration():
    # Define the chessboard size and square size (in any unit, e.g., centimeters)
    chessboard_size = (7, 5)  # number of inner corners per a chessboard row and column
    square_size = 2.8  # actual size of squares on your chessboard

    # Prepare object points (0,0,0), (1,0,0), (2,0,0), ...
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []
    imgpoints = []

    # Load calibration images
    images = glob.glob('Laser/LaserPath/RawImages/Calibration/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        # print(ret)
        
        if ret:
            objpoints.append(objp)
            # Refine corner locations (optional but recommended)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            imgpoints.append(corners2)

            # Draw and display the corners for visualization
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imwrite('Laser/LaserPath/ProcessedImages/Calibration/calib.jpg', img)
            cv2.waitKey(100)

    cv2.destroyAllWindows()

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera matrix (intrinsic parameters):\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)
    np.save('Laser/LaserPath/Utils/CalibMatrix.npy', camera_matrix)
    np.save('Laser/LaserPath/Utils/CalibDistCoeffs.npy', dist_coeffs)

    return camera_matrix, dist_coeffs

# Camera matrix (intrinsic parameters):
#  [[693.66029914   0.         324.18593401]
#  [  0.         688.22194787 234.97248167]
#  [  0.           0.           1.        ]]
# Distortion coefficients:
#  [[ 1.27379011e-01  3.51281650e-02  1.72963393e-04 -5.26145128e-04
#   -1.48159817e+00]]

def Undistort(image):
    camera_matrix = np.load('Laser/LaserPath/Utils/CalibMatrix.npy')
    dist_coeffs = np.load('Laser/LaserPath/Utils/CalibDistCoeffs.npy', dist_coeffs)
    # image = cv2.imread('Laser/LaserPath/RawImages/Degree tests/various degrees/65.jpg')
    h,  w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w,h), 1, (w,h))
    dst = cv2.undistort(image, camera_matrix, dist_coeffs, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    # cv2.imwrite('Laser/LaserPath/ProcessedImages/Calibration/caled65.jpg', dst)
    return dst