#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

using std::vector;
using std::string;
using cv::Mat;

class cascade_face_detector {
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;

  std::string face_cascade_file;
  std::string eyes_cascade_file;

  bool configuration_loaded = false;

  std::vector<cv::Point> get_prediction_centers(std::vector<cv::Rect> faces)
  {
  }

public:
    cascade_face_detector(std::string face_cascade_file, std::string eyes_cascade_file) 
      : face_cascade_file(face_cascade_file), eyes_cascade_file(eyes_cascade_file) {
    }

    void load_configuration()
      {
      if (configuration_loaded)
        return;

      auto loaded_face = face_cascade.load(face_cascade_file);
      auto loaded_eyes = eyes_cascade.load(eyes_cascade_file);

      if (!loaded_face)
      {
	      std::cerr << "Couldn't load face cascade classifier data file from " << face_cascade_file << std::endl;
	      exit(2);
      }
      if (!loaded_eyes)
      {
	      std::cerr << "Couldn't load eyes cascade classifier data file from " << eyes_cascade_file << std::endl;
	      exit(2);
      }


      configuration_loaded = true;
    }

    using faces_vector = std::vector<cv::Rect>;
    using eyes_map = std::map<size_t, std::vector<cv::Rect>>;
    using detection_pair = std::pair<faces_vector, eyes_map>;

    detection_pair detect_raw(Mat frame) {
        load_configuration();

        /* Convert the frame to grayscale and equalize the histogram */
        Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(frame_gray, frame_gray);

        /* Detect faces */
        std::vector<cv::Rect> faces;
        std::map<size_t, std::vector<cv::Rect>> all_eyes;
        face_cascade.detectMultiScale(frame_gray, faces);

        /* Detect eyes for each face */
        for (size_t i = 0; i < faces.size(); i++)
        {
            /* Find the region in which the face is and extract it */
            cv::Mat region = frame_gray(faces[i]);

            /* Detect eyes and copy the result in our big face-eyes map */
            std::vector<cv::Rect> eyes;
            eyes_cascade.detectMultiScale(region, eyes);

            std::sort(eyes.begin(), eyes.end(), [](const cv::Rect& a, const cv::Rect& b) {
                return b.width < a.width;
            });


            std::vector<cv::Rect> first_two_eyes;
            if (eyes.size() > 0)
                first_two_eyes.push_back(eyes.front());
            if (eyes.size() > 1)
                first_two_eyes.push_back(eyes[1]);

            all_eyes[i] = first_two_eyes;
        }

        return std::make_pair(faces, all_eyes);
    }

    cv::Mat detect(Mat frame)
    {
        auto predictions = detect_raw(frame);
        auto faces = predictions.first;
        auto all_eyes = predictions.second;

        draw_predictions(frame, faces, all_eyes);
        return frame;
    }

    void draw_predictions(cv::Mat& frame, std::vector<cv::Rect>& faces, std::map<size_t, std::vector<cv::Rect>>& eyes) {
        /* Draw a rectangle for each face */
        for(auto& face : faces)
        {
            cv::rectangle(frame,
              cv::Point(face.x, face.y),
              cv::Point(face.x + face.width, face.y + face.height),
              cv::Scalar(255, 178, 50), 3);
        }

        /* Draw eyes detected in each face */
        for(auto& pair : eyes)
        {
            /* Extract face index / eye rectangle */
            auto& [i, eyes] = pair;

            for(auto& eye : eyes) {
            /* Determine the centre of the eye rectangle */
            auto eye_center = cv::Point(
              faces[i].x + eye.x + eye.width / 2,
              faces[i].y + eye.y + eye.height / 2);

            /* Determine the radius of the circle we want to draw around the eye */
            int radius = cvRound((eye.width + eye.height) * 0.25);

            /* Draw the circle */
            cv::circle(frame, eye_center, radius, cv::Scalar(255, 0, 0), 4);
            }
        }
    }
};
