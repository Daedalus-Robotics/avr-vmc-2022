import socket
import subprocess

import pickle
import cv2
import numpy as np


DEFAULT_PARAMS = (int(cv2.IMWRITE_JPEG_QUALITY), 90)


def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized


def encode_frame(frame: np.ndarray, params: list = DEFAULT_PARAMS) -> (bool, bytes):
    result, jpeg_frame = cv2.imencode('.jpg', frame, params)
    if result:
        encoded_frame = jpeg_frame.dumps()
        return True, encoded_frame
    else:
        return False, None


def encode_frame_uncompressed(frame: np.ndarray) -> str:
    encoded_frame = frame.dumps()
    return encoded_frame


def decode_frame(encoded_frame: bytes) -> (bool, np.ndarray):
    try:
        jpeg_frame: np.ndarray = pickle.loads(encoded_frame)
        jpeg_frame = jpeg_frame.astype(np.uint8)
        frame: np.ndarray = cv2.imdecode(jpeg_frame, 1)
        if frame is not None and type(frame) == np.ndarray:
            return True, frame
        else:
            return False, None
    except (pickle.PickleError, TypeError, EOFError, cv2.error):
        return False, None
