#! /usr/bin/env python

import cv2
import cvlib as cv

class YoloV3():
    def detect(self, cv_image):
        bboxes, labels, conf = cv.detect_common_objects(cv_image, model='yolov3-tiny', enable_gpu=True)
        indices = cv2.dnn.NMSBoxes(bboxes, conf, score_threshold=0.8, nms_threshold=0.6)
        return bboxes, indices