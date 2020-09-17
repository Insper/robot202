import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import time

CONFIDENCE = 0.7
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

def detect(frame):
    """
        Recebe - uma imagem colorida
        Devolve: objeto encontrado
    """
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > CONFIDENCE:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    # show the output image
    return image, results

proto = "../mobilenet_detection/MobileNetSSD_deploy.prototxt.txt"
model = "../mobilenet_detection/MobileNetSSD_deploy.caffemodel"

net = cv2.dnn.readNetFromCaffe(proto, model)

cap = cv2.VideoCapture(0)

print("Known classes")
print(CLASSES)

result_list = list()
historic = ['.','..','...','....','.....']
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    result_frame, result_tuples = detect(frame)
    print(result_tuples)
    try:
        historic.append(result_tuples[0][0])
    except:
        historic = historic
    if len(historic) > 5:
        historic.remove(historic[0])

    # Display the resulting frame
    try:
        if result_tuples[0][0] == historic[-1] == historic[-2] == historic[-3] == historic[-4] == historic[-5] == 'person':
            cv2.imshow('frame',result_frame)
        else:
            cv2.imshow('frame', frame)
    except:
        cv2.imshow('frame', frame)

    # Prints the structures results:
    # Format:
    # ("CLASS", confidence, (x1, y1, x2, y3))
    
    print('HISTORIC OF DETECTED OBJECTS:\n', historic)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()