#include <opencv2/opencv.hpp>
#include <iostream>
#include <queue>
#include <vector>
#include <map>
#include <utility>
#include <opencv2/ximgproc.hpp>
#include <fstream>
#include <cmath>

//////// Function to scale image to fit a window
cv::Mat scaleImageToFit(const cv::Mat& src, int maxWidth, int maxHeight) {
    double scaleX = (double)maxWidth / src.cols;
    double scaleY = (double)maxHeight / src.rows;
    double scale = std::min(scaleX, scaleY);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(), scale, scale, cv::INTER_AREA);

    // Increase contrast using cv::convertScaleAbs()
    double alpha = 0.5;  // Contrast factor (<1 increases contrast)
    double beta =5;    // Brightness factor (>0 increases brightness)
    
    cv::Mat contrastEnhanced;
    resized.convertTo(contrastEnhanced, -1, alpha, beta);

    return contrastEnhanced;
}

//////// Detect red dots and return their coordinates
std::vector<cv::Point> findRedDots(const cv::Mat& src) {
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::Mat redMask1, redMask2, redMask;
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), redMask1);
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), redMask2);
    cv::bitwise_or(redMask1, redMask2, redMask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point> redDotLocations;
    for (const auto& contour : contours) {
        cv::Moments moment = cv::moments(contour);
        if (moment.m00 != 0) {
            redDotLocations.emplace_back(moment.m10 / moment.m00, moment.m01 / moment.m00);
        }
    }
    return redDotLocations;
}

//////// Skeletonize the maze
cv::Mat skeletonize(const cv::Mat& binary) {
    cv::Mat skel;
    cv::ximgproc::thinning(binary, skel, cv::ximgproc::THINNING_ZHANGSUEN);
    return skel;
}

//////// Find shortest path from start to end
std::vector<cv::Point> findShortestPath(cv::Mat& maze, cv::Point start, cv::Point end, int endRadius, const std::string& outputFile) {
    int rows = maze.rows, cols = maze.cols;
    std::queue<cv::Point> q;
    std::map<cv::Point, cv::Point, bool(*)(cv::Point, cv::Point)> parent([](cv::Point a, cv::Point b) {
        return a.y == b.y ? a.x < b.x : a.y < b.y;
    });
    q.push(start);
    parent[start] = start;
    int dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
    int dy[] = {1, -1, 0, 0, 1, -1, -1, 1};
    bool found = false;
    while (!q.empty() && !found) {
        cv::Point current = q.front();
        q.pop();
        for (int i = 0; i < 8; ++i) {
            cv::Point next(current.x + dx[i], current.y + dy[i]);
            if (next.x >= 0 && next.x < cols && next.y >= 0 && next.y < rows &&
                maze.at<uchar>(next.y, next.x) == 255 && parent.find(next) == parent.end()) {
                q.push(next);
                parent[next] = current;
                for (int ey = end.y - endRadius; ey <= end.y + endRadius; ++ey) {
                    for (int ex = end.x - endRadius; ex <= end.x + endRadius; ++ex) {
                        if (ex == next.x && ey == next.y) {
                            found = true;
                            break;
                        }
                    }
                    if (found) break;
                }
                if (found) break;
            }
        }
    }
    if (!found) {
        std::cerr << "No path found!" << std::endl;
        return {};
    }
    std::vector<cv::Point> path;
    cv::Point at = end;
    for (int ey = end.y - endRadius; ey <= end.y + endRadius; ++ey) {
        for (int ex = end.x - endRadius; ex <= end.x + endRadius; ++ex) {
            if (parent.find(cv::Point(ex, ey)) != parent.end()) {
                at = cv::Point(ex, ey);
                break;
            }
        }
        if (at != end) break;
    }
    while (at != start) {
        path.push_back(at);
        at = parent[at];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());

    std::vector<cv::Point> turnPoints;
    turnPoints.push_back(start);

    double angleThreshold = 0.5; // Adjust this value to tweak sensitivity
    int minPointDistance = 10; // Minimum distance between turn points
    int turnRadius = 3; // radius check for turn consistency

    for (size_t i = turnRadius; i < path.size() - turnRadius - 1; ++i) {
        cv::Point prev = path[i - turnRadius];
        cv::Point curr = path[i];
        cv::Point next = path[i + turnRadius];

        double angle1 = std::atan2(curr.y - prev.y, curr.x - prev.x);
        double angle2 = std::atan2(next.y - curr.y, next.x - curr.x);
        double angleDiff = std::abs(angle2 - angle1);
        if (angleDiff > M_PI) angleDiff = 2 * M_PI - angleDiff;

        if (angleDiff > angleThreshold) {
            bool addPoint = true;
            for (const auto& existingPoint : turnPoints) {
                double distance = std::sqrt(std::pow(curr.x - existingPoint.x, 2) + std::pow(curr.y - existingPoint.y, 2));
                if (distance < minPointDistance) {
                    addPoint = false;
                    break;
                }
            }
            if (addPoint) {
                turnPoints.push_back(curr);
            }
        }
    }

    turnPoints.push_back(end);

    std::cout << "Turn Points:" << std::endl;
    for (const auto& point : turnPoints) {
        std::cout << "(" << point.x << ", " << point.y << ")" << std::endl;
    }

    std::ofstream outFile(outputFile);
    if (outFile.is_open()) {
        for (const auto& point : turnPoints) {
            outFile << point.x << " " << point.y << std::endl;
        }
        outFile.close();
        std::cout << "Turn points saved to " << outputFile << std::endl;
    } else {
        std::cerr << "Error: Could not open file " << outputFile << std::endl;
    }

    return path;
}

//////// detects corners of the maze boundry 
std::vector<cv::Point> detectMazeBoundingBoxCorners(const cv::Mat& binaryImage, cv::Mat& imageToDrawOn) {
    std::vector<cv::Point> corners;
    std::vector<std::vector<cv::Point>> contours;

    // Invert the binary image to find black contours
    cv::Mat invertedBinary;
    cv::bitwise_not(binaryImage, invertedBinary);

    // Find contours
    cv::findContours(invertedBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        std::cerr << "Error: No contours found." << std::endl;
        return corners;
    }

    // Find the largest contour by area
    double maxArea = 0;
    std::vector<cv::Point> bestContour;
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > maxArea) {
            maxArea = area;
            bestContour = contour;
        }
    }

    if (bestContour.empty()) {
        std::cerr << "Error: No valid maze contour found." << std::endl;
        return corners;
    }

    // Approximate the contour with a polygon
    std::vector<cv::Point> approxCurve;
    double epsilon = 0.02 * cv::arcLength(bestContour, true); // Adjust epsilon for accuracy
    cv::approxPolyDP(bestContour, approxCurve, epsilon, true);

    // Ensure we have exactly 4 corners
    if (approxCurve.size() == 4) {
        corners = approxCurve;
    } else {
        std::cerr << "Warning: Found " << approxCurve.size() << " corners instead of 4." << std::endl;
        // If we don't have 4 corners, we can still use the bounding box corners as a fallback
        cv::Rect boundingBox = cv::boundingRect(bestContour);
        corners.push_back(boundingBox.tl());
        corners.push_back(cv::Point(boundingBox.x + boundingBox.width, boundingBox.y));
        corners.push_back(cv::Point(boundingBox.x, boundingBox.y + boundingBox.height));
        corners.push_back(boundingBox.br());
    }

    // Draw blue dots on the corners
    for (const auto& corner : corners) {
        cv::circle(imageToDrawOn, corner, 5, cv::Scalar(255, 0, 0), -1);
    }

    // Print detected corners
    std::cout << "Detected Maze Corners:" << std::endl;
    for (size_t i = 0; i < corners.size(); ++i) {
        std::cout << "Corner " << i + 1 << ": (" << corners[i].x << ", " << corners[i].y << ")" << std::endl;
    }

    return corners;
}

////////calculates distances in mm between each point
std::vector<std::pair<double, double>> calculateRealWorldDistances(const std::vector<cv::Point>& turnPoints, const std::vector<cv::Point>& corners) {
    std::vector<std::pair<double, double>> distances;

    if (corners.size() != 4 || turnPoints.size() < 2) {
        std::cerr << "Error: Invalid corners or turn points." << std::endl;
        return distances;
    }

    // Calculate pixel distance between two corners (e.g., top-left and top-right)
    double pixelDistance = std::sqrt(std::pow(corners[1].x - corners[0].x, 2) + std::pow(corners[1].y - corners[0].y, 2));

    // Calculate pixel to mm ratio
    double mmPerPixel = 200.0 / pixelDistance; // 200mm is the real-world size

    // Calculate real-world distances between turn points
    for (size_t i = 1; i < turnPoints.size(); ++i) {
        double dx = (turnPoints[i].x - turnPoints[i - 1].x) * mmPerPixel;
        double dy = (turnPoints[i].y - turnPoints[i - 1].y) * mmPerPixel;
        distances.emplace_back(dx, dy);
    }

    return distances;
}

int main() {
    std::string image_path = "/home/nathan/ros2_ws/src/ur3_simulation/Images/maze.jpg";
    cv::Mat img = cv::imread(image_path);

    if (img.empty()) {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        return -1;
    }

    cv::Mat scaledImg = scaleImageToFit(img, 800, 600);

    std::vector<cv::Point> redDots = findRedDots(scaledImg);

    if (redDots.size() != 2) {
        std::cerr << "Error: Expected 2 red dots (start and end)." << std::endl;
        return -1;
    }

    cv::Mat maze = scaledImg.clone();
    cv::Mat gray, binary;
    cv::cvtColor(maze, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::threshold(gray, binary, 35, 255, cv::THRESH_BINARY);

    cv::Mat skel = skeletonize(binary);

    int radius = 5;
    cv::Point start, end;
    bool startFound = false, endFound = false;

    for (int y = redDots[0].y - radius; y <= redDots[0].y + radius; ++y) {
        for (int x = redDots[0].x - radius; x <= redDots[0].x + radius; ++x) {
            if (x >= 0 && x < skel.cols && y >= 0 && y < skel.rows && skel.at<uchar>(y, x) == 255) {
                start = cv::Point(x, y);
                startFound = true;
                break;
            }
        }
        if (startFound) break;
    }

    for (int y = redDots[1].y - radius; y <= redDots[1].y + radius; ++y) {
        for (int x = redDots[1].x - radius; x <= redDots[1].x + radius; ++x) {
            if (x >= 0 && x < skel.cols && y >= 0 && y < skel.rows && skel.at<uchar>(y, x) == 255) {
                end = cv::Point(x, y);
                endFound = true;
                break;
            }
        }
        if (endFound) break;
    }

    if (!startFound || !endFound) {
        std::cerr << "Error: Could not find start or end point on skeleton!" << std::endl;
        return -1;
    }

    std::string outputFile = "turn_points.txt";
    std::vector<cv::Point> path = findShortestPath(skel, start, end, radius, outputFile); //change this for different algorithm

    // Draw start and end points in red
    cv::circle(maze, start, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(maze, end, 5, cv::Scalar(0, 0, 255), -1);

    // Detect corners and draw them on the maze image
    std::vector<cv::Point> corners = detectMazeBoundingBoxCorners(binary, maze);

    // Draw turn points in green
    std::ifstream inFile(outputFile);
    std::vector<cv::Point> turnPoints; // Store turn points
    if (inFile.is_open()) {
        int x, y;
        while (inFile >> x >> y) {
            cv::circle(maze, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
            turnPoints.emplace_back(x, y); // Populate turnPoints vector
        }
        inFile.close();
    } else {
        std::cerr << "Error: Could not open file " << outputFile << std::endl;
    }

    // Calculate real-world distances between turn points
    std::vector<std::pair<double, double>> realWorldDistances = calculateRealWorldDistances(turnPoints, corners);

    // Print real-world distances
    std::cout << "Real World Distances (mm):" << std::endl;
    for (const auto& distance : realWorldDistances) {
        std::cout << "dx: " << distance.first << ", dy: " << distance.second << std::endl;
    }
    // Create an inverted copy for display
    cv::Mat invertedSkel;
    cv::bitwise_not(skel.clone(), invertedSkel);

    // Overlay black pixels from inverted skeleton on binary
    cv::Mat overlayed = binary.clone();

    for (int y = 0; y < invertedSkel.rows; ++y) {
        for (int x = 0; x < invertedSkel.cols; ++x) {
            if (invertedSkel.at<uchar>(y, x) == 0) {
                overlayed.at<uchar>(y, x) = 0;
            }
        }
    }

    cv::imshow("Skeleton Overlay", overlayed);
    cv::imshow("Animated Path Overlay", maze);

    while (true) {
        cv::Mat animatedOverlay = maze.clone();
        for (size_t i = 1; i < path.size(); ++i) {
            cv::line(animatedOverlay, path[i - 1], path[i], cv::Scalar(127), 2);
            cv::imshow("Animated Path Overlay", animatedOverlay);
            cv::waitKey(1);
        }
        cv::waitKey(100);
    }

    return 0;
}