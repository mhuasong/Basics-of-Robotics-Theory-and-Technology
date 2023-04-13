/**
 * @file   dualQuaternion.hpp
 * @author Huasong Min
 * @date   1 April 2017
 * @version 0.1
 * @brief   A DualQuaternion Class used only for teaching Kinematics of Robot.
 * @see https://github.com/mhuasong/ for a full description and follow-up descriptions.
 */

class DualQuaternion {
private:
  Eigen::Quaterniond r_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond d_;
  friend DualQuaternion operator*(const DualQuaternion &_lhs,
                                  const DualQuaternion &_rhs);

public:
  DualQuaternion(const Eigen::Quaterniond &_r, const Eigen::Vector3d &_t)
      : r_(_r), t_(_t), 
        d_(Eigen::Quaterniond(0, _t[0] / 2, _t[1] / 2, _t[2] / 2) * _r) {}

  DualQuaternion(const Eigen::Quaterniond &_r)
      : r_(_r), t_(Eigen::Vector3d(0, 0, 0)) {}

  DualQuaternion(const Eigen::Vector3d &_t)
      : r_(Eigen::Quaterniond(1, 0, 0, 0)), t_(_t) {}

  DualQuaternion(const Eigen::Quaterniond &_r, const Eigen::Quaterniond &_d)
      : r_(_r), d_(_d) {
    const Eigen::Quaterniond qt = this->d_ * this->r_.inverse();
    t_ = Eigen::Vector3d(2 * qt.x(), 2 * qt.y(), 2 * qt.z());
  }

  Eigen::Quaterniond r() const { return r_; }
  Eigen::Quaterniond d() const { return d_; }
  Eigen::Vector3d t() const { return t_; }
  Eigen::Vector3d vr() const { return Eigen::Vector3d(r_.x(), r_.y(), r_.z()); }
  Eigen::Vector3d vd() const { return Eigen::Vector3d(d_.x(), d_.y(), d_.z()); }

  /// Transform inverse.
  DualQuaternion inverse() const {
    return DualQuaternion(this->r_.inverse(), this->r_.inverse() * (-this->t_));
  }

  DualQuaternion conjugate() const {
    const Eigen::Quaterniond d_tmp(-this->d_.w(), this->d_.x(), this->d_.y(), this->d_.z());
    return DualQuaternion(this->r_.inverse(), d_tmp);
  }

  Eigen::Vector3d transformPoint(const Eigen::Vector3d &_p) const {
    const DualQuaternion dp0(Eigen::Quaterniond(1, 0, 0, 0),
                             Eigen::Quaterniond(0, _p[0], _p[1], _p[2]));
    const DualQuaternion dp1 = (*this) * dp0 * (this->conjugate());
    return Eigen::Vector3d(dp1.d_.x(), dp1.d_.y(), dp1.d_.z());
  }
};

DualQuaternion operator*(const DualQuaternion &_lhs,
                         const DualQuaternion &_rhs) {
  Eigen::Quaterniond r = (_lhs.r_ * _rhs.r_).normalized();

  Eigen::Quaterniond qd = _lhs.r_ *(_rhs.d_*_lhs.r_.conjugate());
  Eigen::Quaterniond qt = qd ;
  qt.w() = qt.w()+_lhs.d().w(); qt.x() = qt.x()+_lhs.d().x(); qt.y() = qt.y()+_lhs.d().y(); qt.z() = qt.z()+_lhs.d().z();

  return DualQuaternion(r, qt);
}
/*
DualQuaternion operator*(const DualQuaternion &_lhs,
                         const DualQuaternion &_rhs) {
  const Eigen::Quaterniond r = (_lhs.r_ * _rhs.r_).normalized();
  const Eigen::Quaterniond tmp0 = _lhs.r_ * _rhs.d_;
  const Eigen::Quaterniond tmp1 = _lhs.d_ * _rhs.r_;
  const Eigen::Quaterniond qd(tmp0.w() + tmp1.w(), tmp0.x() + tmp1.x(),
                              tmp0.y() + tmp1.y(), tmp0.z() + tmp1.z());
  const Eigen::Quaterniond qt = qd * r.inverse();

  const Eigen::Vector3d t(2 * qt.x(), 2 * qt.y(), 2 * qt.z());

  return DualQuaternion(r, t);
}
*/
