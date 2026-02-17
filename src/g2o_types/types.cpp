#include "g2o_types/types.h"

#include <g2o/core/factory.h>

#include <iomanip>

bool VertexSE3::read(std::istream &is) {
  double data[7];
  for (size_t i = 0; i < 7; ++i) {
    is >> data[i];
  }
  setEstimate(
      Sophus::SE3d(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                   Eigen::Vector3d(data[0], data[1], data[2])));
  return true;
}

bool VertexSE3::write(std::ostream &os) const {
  os << "VERTEX_SE3:QUAT " << id() << " ";
  const Sophus::SE3d &pose = estimate();
  const Eigen::Quaterniond &quat = pose.so3().unit_quaternion();

  os << std::setprecision(18) << pose.translation().x() << " "
     << pose.translation().y() << " " << pose.translation().z();
  os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
     << " ";
  os << std::endl;

  return true;
}

void VertexSE3::setToOriginImpl() { _estimate = Sophus::SE3d(); }

void VertexSE3::oplusImpl(const double *update) {
  const Sophus::SE3d delta =
      Sophus::SE3d::exp(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(update));
  _estimate = _estimate * delta;
};

bool EdgeRelSE3::read(std::istream &is) {
  double data[7];
  for (size_t i = 0; i < 7; ++i) {
    is >> data[i];
  }
  setMeasurement(
      Sophus::SE3d(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                   Eigen::Vector3d(data[0], data[1], data[2])));

  for (long i = 0; i < information().rows() && is.good(); ++i) {
    for (long j = i; j < information().cols() && is.good(); ++j) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }

  return true;
}

bool EdgeRelSE3::write(std::ostream &os) const {
  os << "EDGE_SE3:QUAT ";
  auto v0 = static_cast<VertexSE3 *>(_vertices[0]);
  auto v1 = static_cast<VertexSE3 *>(_vertices[1]);
  os << v0->id() << " " << v1->id() << " ";

  const Eigen::Vector3d &translate = measurement().translation();
  os << std::setprecision(18) << translate.x() << " " << translate.y() << " "
     << translate.z() << " ";

  const Eigen::Quaterniond &quat = measurement().so3().unit_quaternion();
  os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
     << " ";
  for (long i = 0; i < information().rows(); ++i) {
    for (long j = i; j < information().cols(); ++j) {
      os << information()(i, j) << " ";
    }
  }
  os << std::endl;

  return true;
}

void EdgeRelSE3::computeError() {
  auto v0 = static_cast<VertexSE3 *>(_vertices[0]);
  auto v1 = static_cast<VertexSE3 *>(_vertices[1]);
  _error << (measurement().inverse() * v0->estimate().inverse() *
             v1->estimate())
                .log();
}

void EdgeRelSE3::linearizeOplus() {
  auto v0 = static_cast<VertexSE3 *>(_vertices[0]);
  auto v1 = static_cast<VertexSE3 *>(_vertices[1]);
  const Eigen::Matrix<double, 6, 6> Jr_inv =
      Sophus::SE3d::leftJacobianInverse(-_error);

  _jacobianOplusXi =
      -Jr_inv * (v1->estimate().inverse() * v0->estimate()).Adj();
  _jacobianOplusXj = Jr_inv;
}

bool EdgeGNSSPos::read(std::istream &is) {
  double data[3];
  for (size_t i = 0; i < 3; ++i) {
    is >> data[i];
  }
  setMeasurement(Eigen::Map<const Eigen::Vector3d>(data));
  for (long i = 0; i < information().rows() && is.good(); ++i) {
    for (long j = i; j < information().cols() && is.good(); ++j) {
      is >> information()(i, j);
      if (i != j) {
        information()(j, i) = information()(i, j);
      }
    }
  }
  return true;
}
bool EdgeGNSSPos::write(std::ostream &os) const {
  auto vertex = static_cast<VertexSE3 *>(_vertices[0]);
  os << "EDGE_SE3_XYZ_PRIOR " << vertex->id() << " ";
  const Eigen::Vector3d &translate = measurement();
  os << std::setprecision(18) << translate.x() << " " << translate.y() << " "
     << translate.z() << " ";
  for (long i = 0; i < information().rows(); ++i) {
    for (long j = i; j < information().cols(); ++j) {
      os << information()(i, j) << " ";
    }
  }
  os << std::endl;

  return true;
}
void EdgeGNSSPos::computeError() {
  auto vertex = static_cast<VertexSE3 *>(_vertices[0]);
  _error = vertex->estimate().translation() - measurement();
}

namespace g2o {
G2O_REGISTER_TYPE_GROUP(mapping3d)
// can be any name
// names below are registered already for g2o viewer
G2O_REGISTER_TYPE_NAME("VERTEX_SE3:QUAT", VertexSE3);
G2O_REGISTER_TYPE_NAME("EDGE_SE3:QUAT", EdgeRelSE3);
G2O_REGISTER_TYPE_NAME("EDGE_SE3_XYZ_PRIOR", EdgeGNSSPos);

} // namespace g2o