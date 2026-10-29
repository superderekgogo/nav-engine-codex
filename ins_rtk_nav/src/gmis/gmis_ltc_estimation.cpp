#include "nav/gmis/gmis_ltc_estimation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav::gmis {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
Eigen::Vector3d originEcef(const GeodeticOrigin& o) {
  const double e2 = kF * (2.0-kF), s=std::sin(o.latitude_rad), c=std::cos(o.latitude_rad);
  const double n=kA/std::sqrt(1.0-e2*s*s);
  return {(n+o.height_m)*c*std::cos(o.longitude_rad),
          (n+o.height_m)*c*std::sin(o.longitude_rad),
          (n*(1.0-e2)+o.height_m)*s};
}
Eigen::Matrix3d nedToEcef(const GeodeticOrigin& o) {
  const double s=std::sin(o.latitude_rad), c=std::cos(o.latitude_rad);
  const double so=std::sin(o.longitude_rad), co=std::cos(o.longitude_rad);
  Eigen::Matrix3d r;
  r << -s*co, -so, -c*co, -s*so, co, -c*so, c, 0.0, -s;
  return r;
}
}

GmisLtcEstimatorSession::GmisLtcEstimatorSession(GmisLcConfiguration c, double regularization)
  : config_(std::move(c)), clone_regularization_(regularization) {
  if (!(clone_regularization_ > 0.0 && clone_regularization_ < 1e-3))
    throw std::invalid_argument("GMIS-LTC clone regularization must be in (0, 1e-3)");
}

void GmisLtcEstimatorSession::initialize(double time, const Eigen::VectorXd& x,
                                         const Eigen::MatrixXd& p) {
  constexpr int n=StateIndex::dimension;
  if (x.size()!=n || p.rows()!=n || p.cols()!=n || !std::isfinite(time))
    throw std::invalid_argument("invalid GMIS-LTC initial state/covariance");
  Eigen::VectorXd xx(2*n); xx.head(n)=x; xx.tail(n)=x;
  Eigen::MatrixXd pp=Eigen::MatrixXd::Zero(2*n,2*n);
  pp.topLeftCorner(n,n)=p; pp.bottomRightCorner(n,n)=p;
  pp.topRightCorner(n,n)=(1.0-clone_regularization_)*p;
  pp.bottomLeftCorner(n,n)=pp.topRightCorner(n,n);
  estimator_.initialize(xx,pp);
  time_seconds_=time; initialized_=true;
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateImu(const ImuSample& sample) {
  if (!initialized_ || sample.time_seconds < time_seconds_)
    throw std::logic_error("GMIS-LTC IMU events must be initialized and ordered");
  if (sample.time_seconds > time_seconds_) {
    const auto pred=GmisLcModels::predict(estimator_.state().tail(StateIndex::dimension),
                                         sample.time_seconds-time_seconds_,config_.process_noise);
    Eigen::VectorXd x=estimator_.state(); x.tail(StateIndex::dimension)=pred.state;
    Eigen::MatrixXd f=Eigen::MatrixXd::Identity(2*StateIndex::dimension,2*StateIndex::dimension);
    f.bottomRightCorner(StateIndex::dimension,StateIndex::dimension)=pred.jacobian;
    Eigen::MatrixXd q=Eigen::MatrixXd::Zero(2*StateIndex::dimension,2*StateIndex::dimension);
    q.bottomRightCorner(StateIndex::dimension,StateIndex::dimension)=pred.process_noise;
    estimator_.predict(x,f,q); time_seconds_=sample.time_seconds;
  }
  const auto obs=config_.imu_model==GmisImuModelKind::EarthAwareInertialRate
      ? GmisLcModels::linearizeImuEarthAware(estimator_.state().tail(StateIndex::dimension),sample,
                                             config_.imu_measurement_covariance,config_.origin)
      : GmisLcModels::linearizeImu(estimator_.state().tail(StateIndex::dimension),sample,
                                   config_.imu_measurement_covariance);
  Eigen::MatrixXd h=Eigen::MatrixXd::Zero(obs.jacobian.rows(),2*StateIndex::dimension);
  h.rightCols(StateIndex::dimension)=obs.jacobian;
  return estimator_.update(obs.innovation,h,obs.noise);
}

Eigen::Vector3d GmisLtcEstimatorSession::antennaEcef(const Eigen::VectorXd& x) const {
  const auto r=GmisLcModels::cNedToBody(x.segment<3>(StateIndex::attitude)).transpose();
  const Eigen::Vector3d lever_ned=r*config_.antenna_lever_arm_body_m;
  return originEcef(config_.origin)+nedToEcef(config_.origin)*
      (x.segment<3>(StateIndex::position)+lever_ned);
}

double GmisLtcEstimatorSession::ddGeometry(
    const Eigen::Vector3d& receiver,const navcore::TdDdCarrierPhaseObservation& o,
    bool current,const Eigen::Vector3d& base) const {
  const auto& sat=current?o.satellite_position_current_ecef_m:o.satellite_position_previous_ecef_m;
  const auto& pivot=current?o.pivot_position_current_ecef_m:o.pivot_position_previous_ecef_m;
  return (sat-receiver).norm()-(sat-base).norm()-(pivot-receiver).norm()+(pivot-base).norm();
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateEpoch(
    double time,const Eigen::Vector3d& antenna_ned,const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& base,const navcore::PreparedTdDdCarrierPhaseEpoch* tdcp,
    double inflation) {
  constexpr int n=StateIndex::dimension;
  if (!(inflation>=1.0) || !std::isfinite(inflation))
    throw std::invalid_argument("TDCP variance inflation must be finite and >= 1");
  if (std::abs(time-time_seconds_)>0.02)
    throw std::invalid_argument("GMIS-LTC GNSS update must be synchronized to the current IMU state");
  std::vector<Eigen::Index> l1;
  if (tdcp) for (Eigen::Index i=0;i<static_cast<Eigen::Index>(tdcp->observations.size());++i)
    if (tdcp->observations[static_cast<std::size_t>(i)].band==navcore::CarrierPhaseBand::L1 &&
        !tdcp->observations[static_cast<std::size_t>(i)].cycle_slip_detected) l1.push_back(i);
  const Eigen::Index m=3+static_cast<Eigen::Index>(l1.size());
  Eigen::VectorXd innovation(m); Eigen::MatrixXd h=Eigen::MatrixXd::Zero(m,2*n);
  Eigen::MatrixXd r=Eigen::MatrixXd::Zero(m,m);
  const Eigen::VectorXd xprev=estimator_.state().head(n), xcur=estimator_.state().tail(n);
  const auto antennaNed=[&](const Eigen::VectorXd& x) {
    return x.segment<3>(StateIndex::position)+
        GmisLcModels::cNedToBody(x.segment<3>(StateIndex::attitude)).transpose()*config_.antenna_lever_arm_body_m;
  };
  const Eigen::Vector3d predicted_pos=antennaNed(xcur);
  innovation.head<3>()=antenna_ned-predicted_pos;
  h.block<3,3>(0,n+StateIndex::position).setIdentity();
  r.topLeftCorner<3,3>()=pos_cov;
  for (int axis=0;axis<3;++axis) {
    const double eps=1e-6;
    Eigen::VectorXd plus=xcur, minus=xcur; plus(StateIndex::attitude+axis)+=eps; minus(StateIndex::attitude+axis)-=eps;
    h.block<3,1>(0,n+StateIndex::attitude+axis)=(antennaNed(plus)-antennaNed(minus))/(2.0*eps);
  }
  for (Eigen::Index j=0;j<static_cast<Eigen::Index>(l1.size());++j) {
    const Eigen::Index row=3+j, source=l1[static_cast<std::size_t>(j)];
    const auto& o=tdcp->observations[static_cast<std::size_t>(source)];
    const auto previous_receiver=antennaEcef(xprev);
    const double predicted=ddGeometry(antennaEcef(xcur),o,true,base)-ddGeometry(previous_receiver,o,false,base);
    innovation[row]=o.delta_double_difference_m-predicted;
    auto derivative=[&](Eigen::Index block,const Eigen::VectorXd& nominal,bool is_current) {
      for (int k=0;k<n;++k) {
        const double step=(k>=StateIndex::attitude && k<StateIndex::attitude+3)?1e-6:1e-4;
        Eigen::VectorXd p=nominal,mn=nominal; p[k]+=step; mn[k]-=step;
        const double gp=ddGeometry(antennaEcef(p),o,is_current,base);
        const double gm=ddGeometry(antennaEcef(mn),o,is_current,base);
        h(row,block+k)+=(is_current?1.0:-1.0)*(gp-gm)/(2.0*step);
      }
    };
    derivative(0,xprev,false); derivative(n,xcur,true);
  }
  if (tdcp && !l1.empty()) {
    for (Eigen::Index i=0;i<static_cast<Eigen::Index>(l1.size());++i)
      for (Eigen::Index j=0;j<static_cast<Eigen::Index>(l1.size());++j)
        r(3+i,3+j)=tdcp->covariance_m2(l1[static_cast<std::size_t>(i)],l1[static_cast<std::size_t>(j)])*inflation;
  }
  auto result=estimator_.update(innovation,h,r);
  tdcp_rows_=static_cast<std::size_t>(l1.size()); tdcp_nis_=result.nis;
  return result;
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateZupt(double sigma,double gate) {
  constexpr int n=StateIndex::dimension;
  if(sigma<=0||gate<0)throw std::invalid_argument("invalid ZUPT noise/gate");
  Eigen::Matrix<double,3,1> innovation=-estimator_.state().tail(n).segment<3>(StateIndex::velocity);
  Eigen::Matrix<double,3,Eigen::Dynamic> h=Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,2*n);
  h.block<3,3>(0,n+StateIndex::velocity).setIdentity();
  Eigen::Matrix3d r=Eigen::Matrix3d::Identity()*(sigma*sigma);
  nav::core::EstimatorUpdateOptions options;if(gate>0)options.nis_gate=gate;
  return estimator_.update(innovation,h,r,options);
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateNhc(double sigma_lateral,
                                                                    double sigma_vertical,double gate) {
  constexpr int n=StateIndex::dimension;
  if(sigma_lateral<=0||sigma_vertical<=0||gate<0)throw std::invalid_argument("invalid NHC noise/gate");
  const Eigen::VectorXd current=estimator_.state().tail(n);
  const auto body_constraint=[](const Eigen::VectorXd& x) {
    const Eigen::Vector3d body=GmisLcModels::cNedToBody(x.segment<3>(StateIndex::attitude))*
                               x.segment<3>(StateIndex::velocity);
    return Eigen::Vector2d(body.y(),body.z()); // FRD lateral/right and down velocities.
  };
  const Eigen::Vector2d innovation=-body_constraint(current);
  Eigen::Matrix<double,2,Eigen::Dynamic> h=Eigen::Matrix<double,2,Eigen::Dynamic>::Zero(2,2*n);
  for(int col=0;col<6;++col) {
    const int state_index=col<3?StateIndex::velocity+col:StateIndex::attitude+(col-3);
    const double step=col<3?1e-5:1e-6;
    Eigen::VectorXd plus=current,minus=current;plus[state_index]+=step;minus[state_index]-=step;
    h.col(n+state_index)=(body_constraint(plus)-body_constraint(minus))/(2.0*step);
  }
  Eigen::Matrix2d r=Eigen::Matrix2d::Zero();
  r(0,0)=sigma_lateral*sigma_lateral;r(1,1)=sigma_vertical*sigma_vertical;
  nav::core::EstimatorUpdateOptions options;if(gate>0)options.nis_gate=gate;
  return estimator_.update(innovation,h,r,options);
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateZeroPositionChange(double sigma,double gate) {
  constexpr int n=StateIndex::dimension;
  if(sigma<=0||gate<0)throw std::invalid_argument("invalid ZPC noise/gate");
  const Eigen::Vector3d change=estimator_.state().segment<3>(n+StateIndex::position)-
                               estimator_.state().segment<3>(StateIndex::position);
  Eigen::Matrix<double,3,Eigen::Dynamic> h=Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,2*n);
  h.block<3,3>(0,StateIndex::position)=-Eigen::Matrix3d::Identity();
  h.block<3,3>(0,n+StateIndex::position)=Eigen::Matrix3d::Identity();
  Eigen::Matrix3d r=Eigen::Matrix3d::Identity()*(sigma*sigma);
  nav::core::EstimatorUpdateOptions options;if(gate>0)options.nis_gate=gate;
  return estimator_.update(-change,h,r,options);
}

nav::core::EstimatorUpdateResult GmisLtcEstimatorSession::updateZeroHeadingChange(double sigma,double gate) {
  constexpr int n=StateIndex::dimension;constexpr double two_pi=6.28318530717958647692;
  if(sigma<=0||gate<0)throw std::invalid_argument("invalid ZHC noise/gate");
  const double prev=estimator_.state()(StateIndex::attitude+2);
  const double curr=estimator_.state()(n+StateIndex::attitude+2);
  const double change=std::remainder(curr-prev,two_pi);
  Eigen::Matrix<double,1,Eigen::Dynamic> h=Eigen::Matrix<double,1,Eigen::Dynamic>::Zero(1,2*n);
  h(0,StateIndex::attitude+2)=-1.0;h(0,n+StateIndex::attitude+2)=1.0;
  Eigen::Matrix<double,1,1> r;r(0,0)=sigma*sigma;
  nav::core::EstimatorUpdateOptions options;if(gate>0)options.nis_gate=gate;
  return estimator_.update(Eigen::VectorXd::Constant(1,-change),h,r,options);
}

void GmisLtcEstimatorSession::rollLagWindow() {
  constexpr int n=StateIndex::dimension;
  const Eigen::VectorXd cur=estimator_.state().tail(n);
  Eigen::MatrixXd p=estimator_.covariance().bottomRightCorner(n,n);
  Eigen::VectorXd x(2*n); x.head(n)=cur; x.tail(n)=cur;
  Eigen::MatrixXd joint=Eigen::MatrixXd::Zero(2*n,2*n);
  joint.topLeftCorner(n,n)=p; joint.bottomRightCorner(n,n)=p;
  joint.topRightCorner(n,n)=(1.0-clone_regularization_)*p;
  joint.bottomLeftCorner(n,n)=joint.topRightCorner(n,n);
  estimator_.initialize(x,joint);
}
} // namespace nav::gmis
