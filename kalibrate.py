import cv2
from mrcnn_colab_engine import detect_contours_maskrcnn, draw_mask
img = cv2.imread("lego.png")
# 1. Get objects mask with Mask RCNN
class_ids, boxes, masks = detect_contours_maskrcnn(lego_model, img)
for class_id, box, object_contours in zip(class_ids, boxes, masks):
    # Box
    y1, x1, y2, x2 = box
    cv2.rectangle(img, (x1, y1), (x2, y2), colors[class_id], 15)