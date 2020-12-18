import math
from utils import *


def find_lanes(target_image,
               blur_kernel,
               roi,
               canny_low, canny_high,
               rho, theta, hough_threshold, min_line_len, max_line_gap, slope,
               white_low_threshold, order):
    """
    Draw lanes on the given image
    """
    orig = target_image.copy()
    # filter out all colors but white and yellow
    img = filter_color(orig, white_low_threshold=white_low_threshold)
    # convert to grayscale
    img = grayscale(img)
    # apply blurring to reduce noise
    img = gaussian_blur(img, blur_kernel)
    # apply canny to get edges in images
    lines = canny(img, canny_low, canny_high)
    # crop the image only to our region of interest
    img = region_of_interest(lines, roi)
    # compute lines using Hough algorithm
    lines = hough_lines(img, rho, theta*math.pi/180, hough_threshold, min_line_len, max_line_gap)
    # draw the lines (or an approximation of them)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines, color=[255,0,0], thickness=6, slope=slope, order=order)
    # crop only region of interest for the new lines as well
    line_img = region_of_interest(line_img, roi)
    # merge the drawn lines with the original image
    img = weighted_img(line_img, orig)
    return img


params = {
    'roi': [[[0, 600], [300, 330], [500, 330], [800, 600]]],
    'blur_kernel': 5,
    'canny_low': 50,
    'canny_high': 150,
    'rho': 2,
    'theta': 1,
    'hough_threshold': 15,
    'min_line_len': 20,
    'max_line_gap': 40,
    'slope': 0.5,
    'white_low_threshold': 180,
    'order': 1
}


def process_img(img):
    """
    Callback for the Carla camera client
    """
    return find_lanes(img, **params)
