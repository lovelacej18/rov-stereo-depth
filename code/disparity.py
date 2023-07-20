import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from PIL import Image

# Read both images, provided rectified, in cv format

for i in range(1, 16):

    if i < 10:
        img_num = '0' + str(i)
    else:
        img_num = str(i)

    img1_rectified = cv.imread('/home/rfal/Desktop/images/KatzaaL' + img_num + '.tif', cv.IMREAD_GRAYSCALE)
    img2_rectified = cv.imread('/home/rfal/Desktop/images/KatzaaR' + img_num + '.tif', cv.IMREAD_GRAYSCALE)

    # # ------------------------------------------------------------
    # # CALCULATE DISPARITY (DEPTH MAP)
    # # Adapted from: https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py
    # # and: https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html

    # # StereoSGBM Parameter explanations:
    # # https://docs.opencv.org/4.5.0/d2/d85/classcv_1_1StereoSGBM.html

    # # Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
    block_size = 11
    min_disp = -128
    max_disp = 128
    # Maximum disparity minus minimum disparity. The value is always greater than zero.
    # In the current implementation, this parameter must be divisible by 16.
    num_disp = max_disp - min_disp
    # Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
    # Normally, a value within the 5-15 range is good enough
    uniquenessRatio = 5
    # Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
    # Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    speckleWindowSize = 200
    # Maximum disparity variation within each connected component.
    # If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
    # Normally, 1 or 2 is good enough.
    speckleRange = 2
    disp12MaxDiff = 0

    stereo = cv.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        disp12MaxDiff=disp12MaxDiff,
        P1=8 * 1 * block_size * block_size,
        P2=32 * 1 * block_size * block_size,
    )
    disparity_SGBM = stereo.compute(img1_rectified, img2_rectified)

    # plt.imshow(disparity_SGBM, cmap='plasma')
    # plt.colorbar()
    # plt.show()

    # Normalize the values to a range from 0..255 for a grayscale image
    disparity_SGBM = cv.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
                                beta=0, norm_type=cv.NORM_MINMAX)
    disparity_SGBM = np.uint8(disparity_SGBM)
    # cv.imshow("Disparity", disparity_SGBM)
    cv.imwrite('/home/rfal/Desktop/images/results_gray/disparity_SGBM_Katzaa' + img_num + '.png', disparity_SGBM)
    print('completed ' + img_num + '/15')

cv.waitKey()
cv.destroyAllWindows()
# # ---------------------------------------------------------------

# stereo = cv.StereoBM.create(numDisparities=256, blockSize=11)

# stereo.setPreFilterType(1)
# # stereo.setMinDisparity(-128)
# stereo.setUniquenessRatio(5)
# stereo.setSpeckleWindowSize(100)
# # stereo.setSpeckleRange(2)

# for i in range(1, 16):

#     if i < 10:
#             img_num = '0' + str(i)
#     else:
#             img_num = str(i)

#     imgL = cv.imread('/home/rfal/Desktop/images/KatzaaL' + img_num + '.tif', cv.IMREAD_GRAYSCALE)
#     imgR = cv.imread('/home/rfal/Desktop/images/KatzaaR' + img_num + '.tif', cv.IMREAD_GRAYSCALE)

#     disparity = stereo.compute(imgL, imgR)
#     # plt.imshow(disparity, 'gray')
#     # plt.show()
#     cv.imwrite('/home/rfal/Desktop/images/results/disparity_BM_' + img_num + '.png', disparity)
#     print('completed ' + img_num + '/15')

