import numpy as np
import cv2
import math

# constant for the "shape" of the image
from image_shape_definition import X, Y, C, BLUE, GREEN, RED, ALPHA


# a function that copies an image inside another, images need to have alpha channel and has to be colored
def copy_and_paste_image(paste_to_: np.ndarray, copy_from_: np.ndarray, middle_point_x: int, middle_point_y: int) -> np.ndarray:

    # copy_from = copy_from_
    # paste_to = paste_to_

    middle_point_x = int(middle_point_x)
    middle_point_y = int(middle_point_y)

    # compatibility check
    assert len(paste_to_.shape) == 3, "image format not recognized"
    assert len(copy_from_.shape) == 3, "image format not recognized"

    # compatibility check
    assert 3 <= paste_to_.shape[C] <= 4, "image format not recognized"
    assert 3 <= copy_from_.shape[C] <= 4, "image format not recognized"

    # insert alpha channel if missing
    if paste_to_.shape[C] == 3:
        shape = paste_to_.shape
        # print("b: ", shape)
        shape = list(shape)
        shape[2] += 1
        shape = tuple(shape)
        # print("a: ", shape)
        paste_to = np.ones(shape = shape, dtype=np.uint8)*255
        paste_to[:, :, 0:3] = paste_to_.copy()
    else:
        paste_to = paste_to_.copy()

    # insert alpha channel if missing
    if copy_from_.shape[C] == 3:
        shape = copy_from_.shape
        shape = list(shape)
        shape[2] += 1
        shape = tuple(shape)
        copy_from = np.ones(shape = shape, dtype=np.uint8)*255
        copy_from[:, :, 0:3] = copy_from_.copy()
    else:
        copy_from = copy_from_.copy()

    # range to take the copy image form
    copy_from_x1 = 0
    copy_from_x2 = copy_from.shape[X]
    copy_from_y1 = 0
    copy_from_y2 = copy_from.shape[Y]

    # range to paste the image to
    paste_to_x1 = middle_point_x - int(copy_from.shape[X] / 2)
    paste_to_x2 = middle_point_x + math.ceil(copy_from.shape[X] / 2)
    paste_to_y1 = middle_point_y - int(copy_from.shape[Y] / 2)
    paste_to_y2 = middle_point_y + math.ceil(copy_from.shape[Y] / 2)

    assert copy_from_x2-copy_from_x1 == paste_to_x2 - paste_to_x1, "something is wrong here, check the code"
    assert copy_from_y2 - copy_from_y1 == paste_to_y2 - paste_to_y1, "something is wrong here, check the code"

    # insert boundary to avoid overflowing the array
    if paste_to_x1 < 0:
        offset = paste_to_x1
        paste_to_x1 -= offset
        copy_from_x1 -= offset

    if paste_to_y1 < 0:
        offset = paste_to_y1
        paste_to_y1 -= offset
        copy_from_y1 -= offset

    if paste_to_x2 > paste_to.shape[X]:
        offset = paste_to_x2-paste_to.shape[X]
        paste_to_x2 -= offset
        copy_from_x2 -= offset

    if paste_to_y2 > paste_to.shape[Y]:
        offset = paste_to_y2-paste_to.shape[Y]
        paste_to_y2 -= offset
        copy_from_y2 -= offset

    # check the dimensions are the same
    assert copy_from_x2 - copy_from_x1 == paste_to_x2 - paste_to_x1, "something is wrong here, check the code"
    assert copy_from_y2 - copy_from_y1 == paste_to_y2 - paste_to_y1, "something is wrong here, check the code"

    # copy the original image
    return_img = paste_to.copy()

    # copy the slice we need of the first image
    copy_from_slice = copy_from[copy_from_y1:copy_from_y2, copy_from_x1:copy_from_x2, :].copy()

    # creating a reference slice of the part of the image I have to copy
    return_img_slice = return_img[paste_to_y1:paste_to_y2, paste_to_x1:paste_to_x2, :]

    # print(paste_to_y1, paste_to_y2, paste_to_x1, paste_to_x2)

    # print(return_img_slice.shape, copy_from_slice.shape)
    assert return_img_slice.shape == copy_from_slice.shape, "something is wrong in the code"

    # putting the data in, only if alpha channel is different from 0
    return_img_slice[copy_from_slice[:, :, ALPHA] != 0] = copy_from_slice[copy_from_slice[:, :, ALPHA] != 0]

    return return_img
def test():

    track = cv2.imread("../images/track.png", cv2.IMREAD_UNCHANGED)

    res = copy_and_paste_image(track.copy(), track.copy(), 0, 0)

    cv2.imshow('gfg', res)
    cv2.waitKey(0)

#test();


def test2():

    track = cv2.imread("../images/track.png", cv2.IMREAD_UNCHANGED)

    # np.shape(track[track[:,:,3] == 0] (60204, 4)

    #track[track[:,:,3] == 0][:][:] =

    cv2.imshow('gfg', track)
    cv2.waitKey(0)


#test2()