#include <fstream>

#include <opencv2/opencv.hpp>

using std::vector;
using std::string;
using cv::Mat;
using namespace cv::dnn;

float confThreshold = 0.5;
float nmsThreshold = 0.4;
int inpWidth = 416;
int inpHeight = 416;
vector<string> classes;

class yolo_detector {
    string classesFile = "yolo/coco.names";
    string modelConfiguration = "yolo/yolov3.cfg";
    string modelWeights = "yolo/yolov3.weights";

    Net network;

public:
    void load_configuration() {
        std::ifstream ifs(classesFile.c_str());
        string line;
        while (getline(ifs, line)) classes.push_back(line);
        network = readNetFromDarknet(modelConfiguration, modelWeights);
        network.setPreferableBackend(DNN_BACKEND_OPENCV);
        network.setPreferableTarget(DNN_TARGET_CPU);
    }

    cv::Mat detect(Mat frame) {
        load_configuration();

        // Create a 4D blob from a frame.
        cv::Mat blob;
        blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), cv::Scalar(0, 0, 0), true, false);

        //Sets the input to the network
        network.setInput(blob);

        // Runs the forward pass to get output of the output layers
        vector<Mat> outs;
        network.forward(outs, getOutputsNames(network));

        // Remove the bounding boxes with low confidence
        postprocess(frame, outs);

        // Write the frame with the detection boxes
        Mat detectedFrame;
        frame.convertTo(detectedFrame, CV_8U);
        // imwrite("output.png", detectedFrame);
        return detectedFrame;
    }

    void postprocess(Mat &frame, const vector<Mat> &outs) {
        vector<int> classIds;
        vector<float> confidences;
        vector<cv::Rect> boxes;

        for (size_t i = 0; i < outs.size(); ++i) {
            auto data = reinterpret_cast<float *>(outs[i].data);
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                cv::Point classIdPoint;
                double confidence;
                minMaxLoc(scores, nullptr, &confidence, nullptr, &classIdPoint);
                if (confidence > confThreshold) {
                    int centerX = static_cast<int>(data[0] * static_cast<float>(frame.cols));
                    int centerY = static_cast<int>(data[1] * static_cast<float>(frame.rows));
                    int width = static_cast<int>(data[2] * static_cast<float>(frame.cols));
                    int height = static_cast<int>(data[3] * static_cast<float>(frame.rows));
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float) confidence);
                    boxes.emplace_back(cv::Rect(left, top, width, height));
                }
            }
        }

        vector<int> indices;
        NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cv::Rect box = boxes[idx];
            draw_predictions(classIds[idx], confidences[idx], box.x, box.y,
                             box.x + box.width, box.y + box.height, frame);
        }
    }

    void draw_predictions(int classId, float conf, int left, int top, int right, int bottom, Mat &frame) {
        //Draw a rectangle displaying the bounding box
        rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

        //Get the label for the class name and its confidence
        string label = cv::format("%.2f", conf);
        if (!classes.empty()) {
            CV_Assert(classId < (int) classes.size());
            label = classes[classId] + ":" + label;
        }

        //Display the label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = cv::max(top, labelSize.height);
        rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)),
                  cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255),
                  cv::FILLED);
        putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    vector<cv::String> getOutputsNames(const Net &net) {
        static vector<cv::String> names;
        if (names.empty()) {
            //Get the indices of the output layers, i.e. the layers with unconnected outputs
            vector<int> outLayers = net.getUnconnectedOutLayers();

            //get the names of all the layers in the network
            vector<cv::String> layersNames = net.getLayerNames();

            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }

};


