'''
Frame parser function for YOLOv3 based object classfication, frame_buffer is a simple buffer function
using a buffer function is better than directly parsing the data because it allows for adding explicit delays
NMS suppression and 
'''

import numpy as np
import cv2
FONT = cv2.FONT_HERSHEY_PLAIN
COLOR = (225, 225, 225)

def frame_buffer(feed):
    '''
    Parameters
    ----------
    feed : Image frame
        DESCRIPTION: Frame buffer for parsing vision sensor data
        
    Returns
    -------
    detection_output : Image, bounding box and state variable
        DESCRIPTION: output from object_detection function
    '''  
    #passes data for processing in object detection, helps to have explicit calling function, delays can be adjusted
    detection_output = object_detection(feed)
    
    # print("Data parsed to DNN")
    return detection_output

def object_detection(frames):
    '''
    Parameters
    ----------
    frames : Image frames
        DESCRIPTION: Image frames from main function

    Returns
    -------
    img : Xresolution*Yresolution*RGB channels
        DESCRIPTION: Image frames in cv2 numpy format
    x : X coordinates
        DESCRIPTION: X coordinates of bounding box detected after NMS
    y : Y coordinates
        DESCRIPTION: Y coordinates of bounding box detected after NMS
    w : Image coorindates
        DESCRIPTION: width of bounding box detected after NMS
    h : Image coorindates
        DESCRIPTION: height of bounding box detected after NMS
    state : Boolean
        DESCRIPTION: 1 for detection, 0 for no detection
    '''    
    # Load Yolo
    net = cv2.dnn.readNet("yolov3_tiny_last.weights", "yolov3_tiny.cfg")
    classes = ["Rover"]

    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    # colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # Loading image
    img = frames
    height, width, channels = img.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    # Bounding Box information with NMS
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                printer_confidence = confidence
                class_ids.append(class_id)
    
    #NMS supression for detections
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
    state = 0
    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            cv2.rectangle(img, (x, y), (x + w, y + h), COLOR, 2)
            cv2.putText(img, label, (x, y + 30), FONT, 2, COLOR, 2)
            cv2.putText(img, label + " "+str(round(printer_confidence*100, 2)), (x, y+30), FONT, 2, (6, 0, 225), 2)
            
            #Logic for hypothesis verification phase, currently set to 60% comnfidence as threshold
            if printer_confidence*100 > 60:
                state = 1
            else:
                state = 0
        
        #Can be omitted if you dont want to display detection output
        cv2.imshow("Image", img)
    cv2.waitKey(100) & 0XFF

    cv2.destroyAllWindows()
    return img, x, y, w, h, state
