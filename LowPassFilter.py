import sys
import cv2 as cv
import numpy as np


#  Global Variables
DELAY_CAPTION = 0  #1500
DELAY_BLUR = 0  #100
MAX_KERNEL_LENGTH = 31
src = None
dst = None
window_name = 'Smoothing Demo'


def main(argv):
    cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
    # Load the source image
    imageName = argv[0] if len(argv) > 0 else 'lena.jpg'
    global src
    src = cv.imread(cv.samples.findFile(imageName))
    if src is None:
        print('Error opening image')
        print('Usage: smoothing.py [image_name -- default ../data/lena.jpg] \n')
        return -1
    if display_caption(src, 'Original Image') != 0:
        return 0
    display_dst(src, DELAY_CAPTION)

    src = add_noise(src)
    display_caption(src, "Noised Image")
    display_dst(src, DELAY_CAPTION)

    global dst
    dst = np.copy(src)
    if display_dst(dst, DELAY_CAPTION) != 0:
        return 0
    # Applying Homogeneous blur
    if display_caption(src, 'Homogeneous Blur') != 0:
        return 0

    for i in range(1, MAX_KERNEL_LENGTH, 2):
        dst = cv.blur(src, (i, i))
        show_text(dst, "Kernel size = %d" % i)
        if display_dst(dst, DELAY_BLUR) != 0:
            return 0

    # Applying Gaussian blur
    if display_caption(src, 'Gaussian Blur') != 0:
        return 0

    for i in range(1, MAX_KERNEL_LENGTH, 2):
        dst = cv.GaussianBlur(src, (i, i), 0)
        show_text(dst, "Kernel size = %d" % i)
        if display_dst(dst, DELAY_BLUR) != 0:
            return 0

    # Applying Median blur
    if display_caption(src, 'Median Blur') != 0:
        return 0

    for i in range(1, MAX_KERNEL_LENGTH, 2):
        dst = cv.medianBlur(src, i)
        show_text(dst, "Kernel size = %d" % i)
        if display_dst(dst, DELAY_BLUR) != 0:
            return 0

    # Applying Bilateral Filter
    """
    if display_caption(src, 'Bilateral Blur') != 0:
        return 0

    for i in range(1, MAX_KERNEL_LENGTH, 2):
        dst = cv.bilateralFilter(src, i, i * 2, i / 2)
        if display_dst(dst, DELAY_BLUR) != 0:
            return 0
    """

    #  Done
    display_caption(src, 'Done!')
    return 0


def add_noise(image):
    noise = np.random.randint(-20, 20, image.shape)
    is_noised = np.random.randint(0, 2, image.shape)
    image = image.astype(np.int32)
    image[np.where(is_noised == 0)] += noise[np.where(is_noised == 0)]
    image[np.where(image < 0)] = 0
    image = image.astype(np.uint8)
    return image


def show_text(image, text, pos=(20, 20), color=(0, 0, 0)):
    cv.putText(image, text, pos, cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def display_caption(src, caption):
    dst = np.zeros(src.shape, src.dtype)
    rows, cols, _ch = src.shape
    cv.putText(dst, caption,
               (int(cols / 4), int(rows / 2)),
               cv.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255))
    return display_dst(dst, DELAY_CAPTION)


def display_dst(dst, delay):
    cv.imshow(window_name, dst)
    c = cv.waitKey(delay)
    #if c >= 0: return -1
    return 0


if __name__ == "__main__":
    main(sys.argv[1:])