/*
 * Created by: Raad Khan
 * Created On: July 1, 2017
 * Description: Detects lane lines and generates destination point.
 * Usage: LaneFollow node instantiates this helper class.
 */

#include <LineDetect.h>
#include <Eigen/QR>

using namespace cv;

LineDetect::LineDetect() : white(255), numVerticalSlices(10), degree(3) {}

std::vector<Polynomial>
LineDetect::getLaneLines(std::vector<std::vector<cv::Point2d>> lanePoints) {

    std::vector<Polynomial> laneLines;
    Polynomial polyLine;

    for (std::vector<cv::Point2d> const &points : lanePoints) {

        polyLine = this->fitPolyToLine(points, degree);
        laneLines.emplace_back(polyLine);
    }

    return laneLines;
}

std::vector<Window> LineDetect::getBaseWindows(cv::Mat &filteredImage) {

    windowWidth = filteredImage.cols / 6;

    intVec baseHistogram = this->getHistogram(filteredImage);

    std::pair<int, int> peaks = this->getBaseHistogramPeakPositions(baseHistogram);

    std::vector<Window> baseWindows;

    Window leftWindow{peaks.first, windowWidth};
    Window rightWindow{peaks.second, windowWidth};

    baseWindows.emplace_back(leftWindow);
    baseWindows.emplace_back(rightWindow);

    return baseWindows;
}

intVec LineDetect::getHistogram(cv::Mat &ROI) {

    intVec histogram(ROI.cols, 0);

    for (int i = 0; i < ROI.rows; i++) {
        for (int j = 0; j < ROI.cols; j++) {

            int pixelValue = ROI.at<uchar>(i, j);

            if (pixelValue == white) {

                histogram[j]++;
            }
        }
    }

    return histogram;
}

std::pair<int, int> LineDetect::getBaseHistogramPeakPositions(intVec baseHistogram) {

    std::pair<int, int> peaks(0, 0);
    int peakValues = 0;

    for (int i = 0; i < (baseHistogram.size() / 2); i++) {

        if(baseHistogram[i] > peakValues) {

            peakValues = baseHistogram[i];
            peaks.first = i;
        }
    }

    peakValues = 0;

    for (auto i = static_cast<int>(baseHistogram.size() / 2); i < baseHistogram.size(); i++) {

        if (baseHistogram[i] > peakValues) {

            peakValues = baseHistogram[i];
            peaks.second = i;
        }
    }

    return peaks;
}

cv::Mat LineDetect::getWindowSlice(cv::Mat &filteredImage,
                                   Window baseWindow,
                                   int windowSliceIndex) {

    cv::Mat windowSlice = filteredImage(

            Range(windowSliceIndex * filteredImage.rows / numVerticalSlices,
                  (windowSliceIndex + 1) * filteredImage.rows / numVerticalSlices),

            Range(baseWindow.getLeftSide(), baseWindow.getRightSide())
    );

    return windowSlice;
}

int LineDetect::getWindowHistogramPeakPosition(intVec windowHistogram) {

    int peak = 0;
    int peakValue = 0;

    for (int i = 0; i < windowHistogram.size(); i++) {

        if (windowHistogram[i] > peakValue) {

            peakValue = windowHistogram[i];
            peak = i;
        }
    }

    return peak;
}

std::vector<std::vector<cv::Point2d>>
LineDetect::getLanePoints(cv::Mat &filteredImage, std::vector<Window> baseWindows) {

    std::vector<std::vector<cv::Point2d>> lanePoints(baseWindows.size(),
                                                     std::vector<cv::Point2d>());

    for (int windowSliceIndex = 0; windowSliceIndex < numVerticalSlices;
         windowSliceIndex++) {

        for (int windowIndex = 0; windowIndex < baseWindows.size(); windowIndex++) {

            Window baseWindow = baseWindows.at(static_cast<unsigned long>(windowIndex));

            cv::Mat windowSlice = this->getWindowSlice(filteredImage,
                                                       baseWindow,
                                                       windowSliceIndex);

            intVec windowHistogram = this->getHistogram(windowSlice);

            int peak = this->getWindowHistogramPeakPosition(windowHistogram);

            baseWindow.center += peak - windowSlice.cols / 2;
            int y = (windowSliceIndex * filteredImage.rows / numVerticalSlices)
                    + windowSlice.rows / 2;

            cv::Point2d point{(double)baseWindow.center, (double)y};

            lanePoints[windowIndex].push_back(point);

        }
    }

    return lanePoints;
}

Polynomial LineDetect::fitPolyToLine(std::vector<cv::Point2d> points, int order) {

    Polynomial polyLine = Polynomial();

    int orderPLusOne = order + 1;
    assert(points.size() >= orderPLusOne);
    assert(order <= 30);

    std::vector<double> xv(points.size(), 0);
    std::vector<double> yv(points.size(), 0);

    for (size_t i = 0; i < points.size(); i++) {

        xv[i] = points[i].x;
        yv[i] = points[i].y;
    }

    Eigen::MatrixXd A(xv.size(), orderPLusOne);
    Eigen::VectorXd yvMapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
    Eigen::VectorXd result;

    // create matrix
    for (size_t i = 0; i < points.size(); i++)
        for (size_t j = 0; j < orderPLusOne; j++)
            A(i, j) = pow((xv.at(i)), j);

    // solve for linear least squares fit
    result = A.householderQr().solve(yvMapped);

    for (auto i = static_cast<size_t>(result.size()); i < result.size(); i++) {
        // append coefficients to end of coefficients vector from least to greatest order
        polyLine.coefficients.emplace_back((result[i]));
    }

    return polyLine;
}