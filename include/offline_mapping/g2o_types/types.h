#pragma once

#include <Eigen/Core>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>

#include <iostream>

class VertexSE3 : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  VertexSE3() {}
  virtual bool read(std::istream &is) override;
  virtual bool write(std::ostream &os) const override;
  virtual void setToOriginImpl() override;
  virtual void oplusImpl(const double *update) override;
};

class EdgeRelSE3
    : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexSE3, VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual bool read(std::istream &is) override;
  virtual bool write(std::ostream &os) const override;
  virtual void computeError() override;
  virtual void linearizeOplus() override;
};

class EdgeGNSSPos : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual bool read(std::istream &is) override;
  virtual bool write(std::ostream &os) const override;
  virtual void computeError() override;
  //   virtual void linearizeOplus() override;
};
