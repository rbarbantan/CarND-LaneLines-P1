import cv2
import numpy as np


def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def filter_color(img, white_low_threshold=225):
    """
    Filter out other colors except white and yellow (lane colors).
    I had some problem getting the white just right, so I left the lower threshold to be parametrizable.
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    ## mask of WHITE
    mask1 = cv2.inRange(hsv, (0, 0, white_low_threshold), (180, 255, 255))

    # mask of YELLOW
    mask2 = cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))

    ## final mask and masked image
    mask = cv2.bitwise_or(mask1, mask2)
    target = cv2.bitwise_and(img, img, mask=mask)
    return target


def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)


def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def region_of_interest(img, vertices):
    """
    Applies an image mask.

    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    `vertices` should be a numpy array of integer points.
    """
    # defining a blank mask to start with
    mask = np.zeros_like(img)

    # defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, np.array(vertices), ignore_mask_color)

    # returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def create_lines_from_points(points, lines=[]):
    """
    Given a list of points (x,y pairs) it creates a list of lines,
    a line being defined by two consecutive points.
    """
    if len(points) == 0:
        return lines
    else:
        if len(lines) == 0:
            if len(points) == 1:
                raise ValueError('Cannot create line from one point')
            return create_lines_from_points(points[2:], [(points[0], points[1])])
        else:
            return create_lines_from_points(points[1:], lines + [(lines[-1][1], points[0])])


def approx_line(points, x_min=0, x_max=2000, order=1):
    """
    Approximate a list of points by a single line, by fitting a line through all the given points.
    """
    points = np.array(points)

    # get x,y coordinates for all points
    x = points[:, :, ::2]
    y = points[:, :, 1::2]
    # compute the polynomial coefficients
    coeff = np.polyfit(x.flatten(), y.flatten(), order)

    # evaluate the polynomial for x in a given range (can be larger than the image, as clipping will hapen later)
    new_x = np.linspace(x_min, x_max, 10)
    new_y = np.polyval(coeff, new_x)

    # convert the points describing the polynomial into a list of lines (to be drawn on top of the image)
    lines = create_lines_from_points(list(zip(new_x, new_y)))
    lines = [[x0, y0, x1, y1] for ((x0, y0), (x1, y1)) in lines]
    return np.array(lines).astype(int)


def draw_lines(img, lines, color=[255, 0, 0], thickness=2, slope=0.5, order=1):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).

    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.

    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    # gven a minimum absolute slope, we filter out any lines in the [-slope,slope] interval (close to perpendicular)
    if lines is None:
        return
    left_lines = [l for l in lines if compute_slope(l[0]) < -slope]
    right_lines = [l for l in lines if compute_slope(l[0]) > slope]

    # once the lines are slipt in right/left clusters, we approximate them by a polynom (line, curve, etc.)
    new_lines = []
    if len(left_lines) > 0:
        new_lines.append(approx_line(left_lines, order=order))
    if len(right_lines) > 0:
        new_lines.append(approx_line(right_lines, order=order))

    # draw new simplified lines
    for line in new_lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)


def compute_slope(line):
    """
    computes the slope of a given line ([x1,y1,x2,y2])
    """
    x1, y1, x2, y2 = line
    return (y2 - y1) / (x2 - x1) if x2 != x1 else 0


def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.

    Returns the list of lines
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len,
                            maxLineGap=max_line_gap)
    return lines


# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1., γ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.

    `initial_img` should be the image before any processing.

    The result image is computed as follows:

    initial_img * α + img * β + γ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, γ)


def get_roi(img_shape, roi_top_ratio, roi_top_width_ratio):
    """
    Region of interest is defined as an isosceles trapezoid, with the base being the entire botton of the image.
    We then only define its height (roi_top) and the width of the top edge (roi_top_width)
    """
    roi_top = roi_top_ratio * img_shape[0]
    roi_top_width = roi_top_width_ratio * img_shape[1]
    roi = np.array([[
        [0, img_shape[0]],
        [int((img_shape[1] - roi_top_width) / 2), roi_top],
        [int((img_shape[1] + roi_top_width) / 2), roi_top],
        [img_shape[1], img_shape[0]]]], dtype=np.int32)
    return roi
