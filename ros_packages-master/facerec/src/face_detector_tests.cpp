#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <boost/filesystem.hpp>

using std::vector;
using std::string;
using cv::Mat;

const char* face_cascade_file = "data/haarcascade_frontalface_default.xml";
const char* eyes_cascade_file = "data/haarcascade_eye.xml";

namespace fs = boost::filesystem;

#include "face_detector.h"

cascade_face_detector get_detector()
{
    cascade_face_detector detector(face_cascade_file, eyes_cascade_file);
    return detector;
}

cv::Mat load_image(std::string name)
{
    name = "faces/" + name;
    assert(fs::exists(name));
    return cv::imread(name);
}

bool faces_match(cv::Rect first, cv::Rect second)
{
    const double max_relative_difference_height = 0.05;
    const double max_relative_difference_width = 0.05;
    const double max_relative_difference_left = 0.05;
    const double max_relative_difference_top = 0.05;

    auto relative_diff_height = static_cast<double>(std::abs(first.height - second.height)) / first.height;
    auto relative_diff_width = static_cast<double>(std::abs(first.width - second.width)) / first.width;

    auto relative_diff_x = static_cast<double>(std::abs(first.x - second.x)) / first.x;
    auto relative_diff_y = static_cast<double>(std::abs(first.y - second.y)) / first.y;

    return relative_diff_height <= max_relative_difference_height 
           && relative_diff_width <= max_relative_difference_width 
           && relative_diff_x <= max_relative_difference_left 
           && relative_diff_y <= max_relative_difference_top;
}

bool eyes_match(std::vector<cv::Rect> first, std::vector<cv::Rect> second)
{
    if (first.size() != second.size())
        return false;

    if (first.empty())
        return true;

    if (first.size() == 2)
        return (faces_match(first[0], second[0]) && faces_match(first[1], second[1])) ||
               (faces_match(first[1], second[0]) && faces_match(first[0], second[1]));

    return faces_match(first[0], second[0]);
}

void display_frame(const cv::Mat& frame)
{
    static bool window_created = false;
    if(!window_created)
    {
        cv::namedWindow("My Window", cv::WINDOW_NORMAL);
        window_created = true;
    }

    cv::imshow("My Window", frame);
    cv::waitKey(0);
}

using face_vector = std::vector<cv::Rect>;
using eye_map = std::map<size_t, std::vector<cv::Rect>>;

void display_predictions(face_vector& faces, eye_map& eyes) 
{
    for(auto i = 0; i < faces.size(); i++)
    {
        std::cout << "Face " << i << ": " << faces[i] << std::endl;
        for(auto eye : eyes[i])
        {
            std::cout << '\t' << eye << std::endl;
        }
    }

}

void test_face_1()
{
    std::string path = "1.jpg";
    cv::Mat frame = load_image(path);

    auto detector = get_detector();
    auto data = detector.detect_raw(frame);
    
    auto faces = data.first;
    auto eyes = data.second;

    cv::Rect expected_face(89, 109, 1230, 1230);

    std::vector<cv::Rect> expected_eyes = {
        cv::Rect(690, 300, 314, 314),
        cv::Rect(184, 357, 259, 259)
    };

    assert(faces_match(expected_face, faces.front()));
    assert(eyes_match(eyes[0], expected_eyes));
}

void test_face_2()
{
    std::string path = "2.jpg";
    cv::Mat frame = load_image(path);

    auto detector = get_detector();
    auto data = detector.detect_raw(frame);
    
    auto faces = data.first;
    auto eyes = data.second;

    cv::Rect expected_face(76, 8, 331, 331);

    for(auto& rect : eyes[0])
        std::cout << rect << std::endl;

    // Image is too bad to detect eyes
    std::vector<cv::Rect> expected_eyes = {};

    assert(faces_match(expected_face, faces.front()));
    assert(eyes_match(eyes[0], expected_eyes));
}

void test_face_3()
{
    std::string path = "3.jpg";
    cv::Mat frame = load_image(path);

    auto detector = get_detector();
    auto data = detector.detect_raw(frame);
    
    auto faces = data.first;
    auto eyes = data.second;

    cv::Rect expected_face(124, 205, 638, 638);
    std::vector<cv::Rect> expected_eyes = {
        cv::Rect(398, 171, 146, 146),
        cv::Rect(155, 176, 146, 146),
    };

    assert(faces_match(expected_face, faces.front()));
    assert(eyes_match(eyes[0], expected_eyes));
}

void test_face_4()
{
    std::string path = "4.jpg";
    cv::Mat frame = load_image(path);

    auto detector = get_detector();
    auto data = detector.detect_raw(frame);
    
    auto faces = data.first;
    auto eyes = data.second;

    assert(faces.size() == 0);
    assert(eyes.size() == 0);
}

void test_face_5() 
{
    std::cerr << "TEST 5 TODO" << std::endl;
    std::string path = "5.jpg";
    cv::Mat frame = load_image(path);

    auto detector = get_detector();
    auto data = detector.detect_raw(frame);
    
    auto faces = data.first;
    auto eyes = data.second;

    cv::Rect expected_face(124, 205, 638, 638);
    std::vector<cv::Rect> expected_eyes = {
        cv::Rect(398, 171, 146, 146),
        cv::Rect(155, 176, 146, 146),
    };

    assert(faces_match(expected_face, faces.front()));
    assert(eyes_match(eyes[0], expected_eyes));    display_predictions(faces, eyes);
    detector.draw_predictions(frame, faces, eyes);
    display_frame(frame);
}

void all_tests()
{
    std::cout << "Running tests" << std::endl;
    auto functions = std::vector<std::function<void(void)>> {
        test_face_1,
        test_face_2,
        test_face_3,
        test_face_4
    };

    for(size_t i = 0; i < functions.size(); i++)
    {
        std::cout << (i + 1) << " / " << functions.size() << std::endl;
        auto function = functions.at(i);
        function();
    }

    std::cout << "All tests passed!" << std::endl;
}


int main()
{
    std::cout << fs::current_path() << std::endl;
    all_tests();
    return 0;
}
