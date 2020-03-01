import cv2 as cv
import numpy as np
import os

confThreshold = 0.5
nmsThreshold = 0.4
inpWidth = 416
inpHeight = 416
classes = []

class yolo_detector:
    dirname = os.path.dirname(os.path.realpath(__file__))
    classesFile = os.path.join(dirname, "yolo/coco.names")
    modelConfiguration = os.path.join(dirname, "yolo/yolov3.cfg")
    modelWeights = os.path.join(dirname, "yolo/yolov3.weights")
    def __init__(self):
        print("Yolo Detector Initiate")


    def load_configuration(self):
        f = open(self.classesFile, 'r')
        for line in f:
            classes.append(line)
        f.close()
        self.network = cv.dnn.readNetFromDarknet(self.modelConfiguration, self.modelWeights)
        self.network.setPreferableBackend(cv.dnn.DNN_BACKEND_DEFAULT)
        self.network.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
    
    def detect(self, frame):

        self.load_configuration()
        
        # Create a 4D blob from a frame
        blob = cv.dnn.blobFromImage(frame, 1/255.0, (inpWidth, inpHeight), (0.0, 0.0, 0.0), True, False)
        
        # Sets the input to the network
        self.network.setInput(blob)
        
        # Runs the forward pass to get output of the output layers
        names = self.getOutputsNames(self.network)
        outs = self.network.forward(names)
        # Remove the bounding boxes with low confidence
        self.postprocess(frame, outs)

        #Write the frame with the detection boxes
        detectedFrame = np.uint8(frame)#frame.convertTo(detectedFrame, CV_8U)
        
        #cv.imwrite("output.png", detectedFrame)
        return detectedFrame

    def postprocess(self, frame, outs):
        classIds = []
        confidences = []
        boxes = []
                
        for i in range(len(outs)):
            data = outs[i].flatten()
            dataidx = 0
            for j in range(outs[i].shape[0]):
                rows = outs[i].shape[0]
                cols = outs[i].shape[1]

                scores = outs[i][j][5: cols]
                foo, confidence, boo, classIdPoint = cv.minMaxLoc(scores)
                if confidence > confThreshold :
                    frame_rows = frame.shape[0]
                    frame_cols = frame.shape[1]
                    centerX = int(np.multiply(data[dataidx + 0], float(frame_cols)))
                    centerY = int(np.multiply(data[dataidx + 1], float(frame_rows)))# int(data[1] * float(frame_cols))
                    width = int(np.multiply(data[dataidx + 2], float(frame_cols)))# int(data[2] * float(frame_cols))
                    height = int(np.multiply(data[dataidx + 3], float(frame_rows))) # int(data[3] * float(frame_cols))
                    left = int(centerX - width / 2)
                    top = int(centerY - height / 2)
                    classIds.append(classIdPoint[1])
                    confidences.append(float(confidence))
                    boxes.extend([[int(left), int(top), int(width), int(height)]])
                
                dataidx += cols
        indices = cv.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
        for i in range(len(indices)):
            idx = indices[i][0]
            box = boxes[idx]
            self.draw_predictions(classIds[idx], confidences[idx], int(box[0]), int(box[1]), int(box[0] + box[2]), int(box[1] + box[3]), frame)


    def draw_predictions(self, classId, conf, left, top, right, bottom, frame):
        cv.rectangle(frame, (left, top), (right, bottom), (255, 178, 50), 3)
        label = "{:.2f}".format(conf) #cv.format("%.2f", conf)
        if len(classes) != 0:
            assert(classId < int(len(classes)))
            label = classes[classId] + ':' + label
        labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
         
        top = max(top, labelSize[1])
        cv.rectangle(frame, (int(left), int(top-round(1.5*labelSize[1]))), (int(left + round(1.5*labelSize[0])), int(top+baseLine)),(255,255,255), cv.FILLED)
        cv.putText(frame, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 1)


    def getOutputsNames(self, net):
        names = []
        if len(names) == 0:
            # Get the indices of the output layers, i.e. the layers with unconnected outputs
            outLayers = net.getUnconnectedOutLayers();
            
            # get the names of all the layers in the network
            layerNames = net.getLayerNames();
            
            # Get the names of the output layers in names
            for i in range(len(outLayers)):
                names.append(layerNames[(outLayers[i]-1)[0]])

            return names


'''
if __name__ == "__main__":
    image = cv.imread('yolo/input_complex.jpg')
    detector = yolo_detector()
    detected_image = detector.detect(image)
    cv.imwrite("Output.png", detected_image)
''' 
