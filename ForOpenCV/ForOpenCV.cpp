#include "affine_transformations.h"

// Affine transformations
#define WIN_NAME "AFFINE TRANSFORMATION"

cv::Mat baseImage;
cv::Mat windowImage;
cv::Vec2f currentPosition = cv::Vec2f(0,0); float positionChangeSpeed = 20;
float currentAngleRadians = 0, angleChangeSpeed = CV_PI/36;
cv::Vec2f currentScale = cv::Vec2f(1, 1); float scaleChangeSpeed = 0.1f;
float currentSkew = 0, skewChangeSpeed = 0.05f;
cv::Vec2f currentPerspective = cv::Vec2f(0, 0); float perspectiveChangeSpeed = 1e-4f;


bool handleInput(int key) {
    if (key == 27) return false;
    switch (key) {
        case 'w':
            currentPosition[0] -= positionChangeSpeed;
            break;
        case 's':
            currentPosition[0] += positionChangeSpeed;
            break;
        case 'a':
            currentPosition[1] -= positionChangeSpeed;
            break;
        case 'd':
            currentPosition[1] += positionChangeSpeed;
            break;
        case 'q':
            currentAngleRadians += angleChangeSpeed;
            break;
        case 'e':
            currentAngleRadians -= angleChangeSpeed;
            break;
        case 'W':
            currentScale[0] -= scaleChangeSpeed;
            break;
        case 'S':
            currentScale[0] += scaleChangeSpeed;
            break;
        case 'A':
            currentScale[1] -= scaleChangeSpeed;
            break;
        case 'D':
            currentScale[1] += scaleChangeSpeed;
            break;
        case 'Q':
            currentSkew += skewChangeSpeed;
            break;
        case 'E':
            currentSkew -= skewChangeSpeed;
            break;
        case 'i':
            currentPerspective[0] += perspectiveChangeSpeed;
            break;
        case 'k':
            currentPerspective[0] -= perspectiveChangeSpeed;
            break;
        case 'j':
            currentPerspective[1] += perspectiveChangeSpeed;
            break;
        case 'l':
            currentPerspective[1] -= perspectiveChangeSpeed;
            break;
    }
    return true;
}

void redraw(cv::Mat& img) {
    cv::imshow(WIN_NAME, img);
}

bool start() {
    baseImage = cv::imread("T:\\opencv\\sources\\samples\\data\\apple.jpg");
    windowImage = baseImage;
    cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
    redraw(windowImage);
    if (windowImage.empty())
    {
        std::cout << "Couldn't read image!" << std::endl;
        return false;
    }
    return true;
}
void loop() {
    bool wasPressed = false;
    int key;
    while (true) {
        key = cv::pollKey();
        if (key != -1) {
            wasPressed = true;
            if (!handleInput(key)) return;
        }
        else if (wasPressed) {
            std::cout << "a key was pressed" << std::endl;
            cv::Mat T = Affine::createTransformation(currentAngleRadians, 
                                                     currentPosition[0], currentPosition[1],
                                                     currentScale[0], currentScale[1],
                                                     currentSkew, 
                                                     currentPerspective[0], currentPerspective[1]);
            windowImage = Affine::applyTransformation(baseImage, T, true, true);
            redraw(windowImage);
            wasPressed = false;
        }
    }
}

int main()
{
    if (start())
        loop();
	return 0;
}