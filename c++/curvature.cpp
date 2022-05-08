#include <iostream>
#include <eigen3/Eigen/Dense>
#include <string>
#include <fstream>
#include <math.h>


using namespace Eigen;
using namespace std;

#define MAXBUFSIZE ((int) 1e6)

float curvature(float x1, float y1, float x2, float y2, float x3, float y3);

int main()
{
    float kappa;
    float x1 = 1.0245043e+00, x2 = 1.0487726e+00, x3 = 1.0725712e+00;
    float y1 = -1.2038183e-03, y2 = -4.8036799e-03, y3 = -1.0764916e-02;

    kappa = curvature(x1, y1, x2, y2, x3, y3);

    cout << kappa << endl;


}

// This function is used to create a matrix when given a file with values.
MatrixXd readMatrix(const char *filename)
{
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers from file into buffer
    ifstream infile;
    infile.open(filename);
    while (! infile.eof())
        {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;
        if (cols == 0)
            cols = temp_cols;

        rows++;
        }
    infile.close();

    rows--;

    // Populate matrix with numbers
    MatrixXd result(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[cols*i*j];

    return result;
}

// This function is used to calculate the curvature when given three points

float curvature(float x1, float y1, float x2, float y2, float x3, float y3)
{
    // x-coordinates
    Eigen::Vector3f x;
    x << x1, x2, x3;

    // y-coordinates
    Eigen::Vector3f y;
    y << y1, y2, y3;

    // Point 1
    Eigen::Vector2f p1;
    p1 << x1, y1;

    // Point 2
    Eigen::Vector2f p2;
    p2 << x2, y2;

    // Point 3
    Eigen::Vector2f p3;
    p3 << x3, y3;

    // Distance between the different points
    Vector2f diff_1 = p2 - p1;
    Vector2f diff_2 = p3 - p2;

    // The norm or the distance
    float t_a = diff_1.norm();
    float t_b = diff_2.norm();

    Eigen::Matrix3f M;
    M << 1, -t_a, pow(t_a, 2),
        1, 0, 0,
        1, t_b, pow(t_b, 2);

    //std::cout << M << std::endl;
    Matrix3f M_inv = M.inverse();

    //std::cout << M_inv * x << std::endl;
    Vector3f a = M_inv * x;
    Vector3f b = M_inv * y;



    //Formula for the curvature
    float kappa = -2*(a(2) * b(1) - b(2) * a(1)) / (pow(a(1),2) + pow(pow(b(1),2),1.5));
    return kappa;
}