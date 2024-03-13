#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;
using namespace std;

Vector3d normalize(const Vector3d& v) {
    return v.normalized();
}

Vector3d findOrthogonalVector(const Vector3d& v) {
    if (v[0] == 0 && v[1] == 0) {
        if (v[2] == 0) {
            return Vector3d::Zero();
        }
        return Vector3d(1, 0, 0);
    }
    return Vector3d(-v[1], v[0], 0);
}

Vector3d calculateNormal(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3) {
    Vector3d vector1 = v2 - v1;
    Vector3d vector2 = v3 - v1;
    Vector3d normal = vector1.cross(vector2);
    return normalize(normal);
}

Vector3d centroid(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3) {
    return (v1 + v2 + v3) / 3.0;
}

vector<Vector3d> getPossibleDirections(const vector<Vector3d>& vertices,
  const Vector3d& position,
  double radius = 1, 
  int numPoints = 100)
   {
    Vector3d v1 = vertices[0], v2 = vertices[1], v3 = vertices[2];
    Vector3d normal = calculateNormal(v1, v2, v3);
    Vector3d center = centroid(v1, v2, v3);
    
    Vector3d vec = findOrthogonalVector(normal);
    vec = normalize(vec);
    Vector3d vecPerp = normal.cross(vec);
    vecPerp = normalize(vecPerp);
    
    vector<Vector3d> directionVectors;
    double theta = 0;
    for (int i = 0; i < numPoints; ++i) {
        theta = i * 2 * M_PI / numPoints;
        Vector3d circlePoint = center + radius * cos(theta) * vec + radius * sin(theta) * vecPerp;
        Vector3d direction = circlePoint - position;
        directionVectors.push_back(normalize(direction));
    }
    return directionVectors;
}

