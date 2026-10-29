#include "nav/gmis/gmis_tc_estimation.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav::gmis::tc {
namespace {
constexpr double kC=299792458.0, kF1=1575.42e6, kF2=1227.60e6;
constexpr double kLambda1=kC/kF1, kLambda2=kC/kF2, kAlpha2=(kF1/kF2)*(kF1/kF2);
constexpr double kA=6378137.0, kF=1.0/298.257223563;
Eigen::Vector3d originEcef(const GeodeticOrigin& o) {
  const double e2=kF*(2-kF),s=std::sin(o.latitude_rad),c=std::cos(o.latitude_rad),n=kA/std::sqrt(1-e2*s*s);
  return {(n+o.height_m)*c*std::cos(o.longitude_rad),(n+o.height_m)*c*std::sin(o.longitude_rad),(n*(1-e2)+o.height_m)*s};
}
Eigen::Matrix3d nedToEcef(const GeodeticOrigin& o) {
  const double s=std::sin(o.latitude_rad),c=std::cos(o.latitude_rad),so=std::sin(o.longitude_rad),co=std::cos(o.longitude_rad);
  Eigen::Matrix3d r; r<<-s*co,-so,-c*co,-s*so,co,-c*so,c,0.0,-s; return r;
}
const navcore::PreparedSatellite* findSatellite(const navcore::PreparedGnssEpoch& e,int svid) {
  const auto it=std::find_if(e.satellites.begin(),e.satellites.end(),[&](const auto& s){return s.svid==svid;});
  return it==e.satellites.end()?nullptr:&*it;
}
bool valid(const std::optional<double>& x) { return x && std::isfinite(*x) && *x>0.0; }
double elevation(const navcore::PreparedSatellite& s) { return s.satellite_elevation_deg.value_or(30.0); }
Eigen::Vector3d vector3(const navcore::Json& j,const char* key) {
  const auto& a=j.at(key).asArray(); if(a.size()!=3) throw std::runtime_error(std::string("expected vector: ")+key);
  return {a[0].asNumber(),a[1].asNumber(),a[2].asNumber()};
}
}

Configuration Configuration::fromJson(const navcore::Json& j) {
  Configuration c; c.common=GmisLcConfiguration::fromJson(j);
  if(c.common.combination_mode!="gmis_tc") throw std::invalid_argument("GMIS TC configuration required");
  const auto& t=j.at("tc");
  c.c1_sigma_m=t.at("c1_sigma_per_receiver_m").asNumber(); c.p2_sigma_m=t.at("p2_sigma_per_receiver_m").asNumber();
  c.l1_sigma_m=t.at("l1_phase_sigma_per_receiver_m").asNumber(); c.l2_sigma_m=t.at("l2_phase_sigma_per_receiver_m").asNumber();
  c.ionosphere_initial_sigma_m=t.at("ionosphere_initial_sigma_m").asNumber();
  c.ambiguity_initial_sigma_cycles=t.at("ambiguity_initial_sigma_cycles").asNumber();
  c.ionosphere_rw_m_sqrt_s=t.at("ionosphere_rw_m_sqrt_s").asNumber();
  c.ambiguity_rw_cycles_sqrt_s=t.at("ambiguity_rw_cycles_sqrt_s").asNumber();
  c.max_arc_gap_seconds=t.at("max_arc_gap_seconds").asNumber(); c.minimum_elevation_deg=t.at("minimum_elevation_deg").asNumber();
  c.minimum_common_satellites=static_cast<std::size_t>(t.at("minimum_common_satellites").asNumber());
  c.nis_gate=t.at("nis_gate").asNumber();
  if(c.c1_sigma_m<=0||c.p2_sigma_m<=0||c.l1_sigma_m<=0||c.l2_sigma_m<=0||c.ionosphere_initial_sigma_m<=0||
     c.ambiguity_initial_sigma_cycles<=0||c.ionosphere_rw_m_sqrt_s<0||c.ambiguity_rw_cycles_sqrt_s<0||
     c.max_arc_gap_seconds<=0||c.minimum_elevation_deg<0||c.minimum_common_satellites<2||c.nis_gate<0)
    throw std::invalid_argument("invalid GMIS TC measurement/state configuration");
  return c;
}

GmisTcEstimatorSession::GmisTcEstimatorSession(Configuration c):config_(std::move(c)) {}

void GmisTcEstimatorSession::initialize(double time,const Eigen::VectorXd& x,const Eigen::MatrixXd& p) {
  if(x.size()!=StateIndex::dimension||p.rows()!=x.size()||p.cols()!=x.size()||!std::isfinite(time))
    throw std::invalid_argument("invalid GMIS TC initial navigation state/covariance");
  estimator_.initialize(x,p); time_seconds_=time; initialized_=true;
}

Eigen::Vector3d GmisTcEstimatorSession::antennaEcef(const Eigen::VectorXd& x) const {
  const Eigen::Vector3d antenna_ned=x.segment<3>(StateIndex::position)+
      GmisLcModels::cNedToBody(x.segment<3>(StateIndex::attitude)).transpose()*config_.common.antenna_lever_arm_body_m;
  return originEcef(config_.common.origin)+nedToEcef(config_.common.origin)*antenna_ned;
}

double GmisTcEstimatorSession::geometricDoubleDifference(const Eigen::Vector3d& rover,
    const navcore::PreparedGnssEpoch& e,int svid,int pivot) const {
  const auto* s=findSatellite(e,svid); const auto* p=findSatellite(e,pivot);
  if(!s||!p) return std::numeric_limits<double>::quiet_NaN();
  const auto& base=e.base_station_position_ecef_m;
  const double rs=(rover-s->satellite_position_ecef_m).norm()-(base-s->satellite_position_ecef_m).norm();
  const double rp=(rover-p->satellite_position_ecef_m).norm()-(base-p->satellite_position_ecef_m).norm();
  return rs-rp;
}

std::vector<GmisTcEstimatorSession::RawRow> GmisTcEstimatorSession::makeRawRows(
    const navcore::PreparedGnssEpoch& e,int pivot,const Eigen::Vector3d&) const {
  std::vector<RawRow> rows; const auto* p=findSatellite(e,pivot); if(!p) return rows;
  auto add=[&](const navcore::PreparedSatellite& s,const char* type,int freq,
               const std::optional<double>& rv,const std::optional<double>& bv,
               const std::optional<double>& pivot_rv,const std::optional<double>& pivot_bv,double sig) {
    if(!valid(rv)||!valid(bv)||!valid(pivot_rv)||!valid(pivot_bv)) return;
    const double el=std::max(config_.minimum_elevation_deg,std::min(elevation(s),elevation(*p)));
    const double scale=1.0/std::sqrt(std::max(0.15,std::sin(el*3.14159265358979323846/180.0)));
    rows.push_back({type,s.svid,freq,(*rv-*bv)-(*pivot_rv-*pivot_bv),sig*scale});
  };
  for(const auto& s:e.satellites) {
    if(s.svid==pivot||elevation(s)<config_.minimum_elevation_deg) continue;
    if(valid(s.rover_c1_m)&&valid(s.base_c1_m)&&valid(p->rover_c1_m)&&valid(p->base_c1_m))
      add(s,"code",1,s.rover_c1_m,s.base_c1_m,p->rover_c1_m,p->base_c1_m,config_.c1_sigma_m);
    if(valid(s.rover_p2_m)&&valid(s.base_p2_m)&&valid(p->rover_p2_m)&&valid(p->base_p2_m))
      add(s,"code",2,s.rover_p2_m,s.base_p2_m,p->rover_p2_m,p->base_p2_m,config_.p2_sigma_m);
    if(valid(s.rover_l1_cycles)&&valid(s.base_l1_cycles)&&valid(p->rover_l1_cycles)&&valid(p->base_l1_cycles)&&
       !s.rover_l1_cycle_slip&&!s.base_l1_cycle_slip&&!p->rover_l1_cycle_slip&&!p->base_l1_cycle_slip) {
      const double dd=((*s.rover_l1_cycles-*s.base_l1_cycles)-(*p->rover_l1_cycles-*p->base_l1_cycles))*kLambda1;
      const double scale=1.0/std::sqrt(std::max(0.15,std::sin(std::min(elevation(s),elevation(*p))*3.14159265358979323846/180.0)));
      rows.push_back({"phase",s.svid,1,dd,config_.l1_sigma_m*scale});
    }
    if(valid(s.rover_l2_cycles)&&valid(s.base_l2_cycles)&&valid(p->rover_l2_cycles)&&valid(p->base_l2_cycles)&&
       !s.rover_l2_cycle_slip&&!s.base_l2_cycle_slip&&!p->rover_l2_cycle_slip&&!p->base_l2_cycle_slip) {
      const double dd=((*s.rover_l2_cycles-*s.base_l2_cycles)-(*p->rover_l2_cycles-*p->base_l2_cycles))*kLambda2;
      const double scale=1.0/std::sqrt(std::max(0.15,std::sin(std::min(elevation(s),elevation(*p))*3.14159265358979323846/180.0)));
      rows.push_back({"phase",s.svid,2,dd,config_.l2_sigma_m*scale});
    }
  }
  return rows;
}

void GmisTcEstimatorSession::synchronizeStates(const navcore::PreparedGnssEpoch& e,int pivot) {
  constexpr Eigen::Index navN=StateIndex::dimension;
  const Eigen::VectorXd oldx=estimator_.state(); const Eigen::MatrixXd oldp=estimator_.covariance();
  const auto oldSlots=slots_; const int oldPivot=pivot_svid_; const Eigen::Index oldN=oldx.size();
  std::map<int,StateSlot> desired;
  for(const auto& s:e.satellites) {
    if(s.svid==pivot||elevation(s)<config_.minimum_elevation_deg) continue;
    const auto it=oldSlots.find(s.svid);
    StateSlot slot; slot.last_seen=e.gps_seconds;
    if(it!=oldSlots.end()) slot=it->second,slot.last_seen=e.gps_seconds;
    desired.emplace(s.svid,slot);
  }
  for(const auto& [svid,slot]:oldSlots) {
    if(svid==pivot||desired.count(svid)) continue;
    if(e.gps_seconds-slot.last_seen<=config_.max_arc_gap_seconds) desired.emplace(svid,slot);
  }
  std::vector<int> sats; for(const auto& [svid,_]:desired)sats.push_back(svid);
  Eigen::VectorXd newx=Eigen::VectorXd::Zero(navN+3*static_cast<Eigen::Index>(sats.size()));
  newx.head(navN)=oldx.head(navN);
  Eigen::MatrixXd transform=Eigen::MatrixXd::Zero(newx.size(),oldN); transform.topLeftCorner(navN,navN).setIdentity();
  slots_.clear(); std::vector<Eigen::Index> newlyInitialized;
  const auto oldComponent=[&](int svid,int component)->Eigen::Index {
    if(svid==oldPivot) return -1;
    auto it=oldSlots.find(svid); if(it==oldSlots.end()) return -1;
    const StateSlot& q=it->second; return component==0?q.ionosphere:(component==1?q.l1:q.l2);
  };
  for(std::size_t i=0;i<sats.size();++i) {
    const int svid=sats[i]; StateSlot slot; slot.ionosphere=navN+static_cast<Eigen::Index>(3*i);slot.l1=slot.ionosphere+1;slot.l2=slot.ionosphere+2;slot.last_seen=desired.at(svid).last_seen;
    slots_[svid]=slot;
    const StateSlot& targetOld=desired.at(svid);
    for(int comp=0;comp<3;++comp) {
      const Eigen::Index out=slot.ionosphere+comp;
      const Eigen::Index a=oldComponent(svid,comp),b=oldComponent(pivot,comp);
      if(a>=navN) transform(out,a)+=1.0;
      if(b>=navN) transform(out,b)-=1.0;
      const bool mapped=(a>=navN||svid==oldPivot)&&(b>=navN||pivot==oldPivot)&&oldPivot!=0;
      if(mapped) {
        newx(out)=(a>=navN?oldx(a):0.0)-(b>=navN?oldx(b):0.0);
      } else newlyInitialized.push_back(out);
      (void)targetOld;
    }
  }
  Eigen::MatrixXd pnew=transform*oldp*transform.transpose();
  const Eigen::Vector3d rover=antennaEcef(newx.head(navN));
  const auto raw=makeRawRows(e,pivot,rover);
  std::map<std::pair<int,int>,double> observed;
  for(const auto& r:raw) observed[{r.svid,r.type=="code"?r.frequency:10+r.frequency}]=r.observed;
  for(const Eigen::Index idx:newlyInitialized) {
    const Eigen::Index rel=idx-navN; const std::size_t si=static_cast<std::size_t>(rel/3);const int comp=static_cast<int>(rel%3),svid=sats[si];
    const double geom=geometricDoubleDifference(rover,e,svid,pivot);
    const auto c1=observed.find({svid,1}),p2=observed.find({svid,2});
    double ion=0.0;
    if(c1!=observed.end()&&p2!=observed.end()) ion=(p2->second-c1->second)/(kAlpha2-1.0);
    if(comp==0) { newx(idx)=ion;pnew(idx,idx)=config_.ionosphere_initial_sigma_m*config_.ionosphere_initial_sigma_m; }
    else {
      const int freq=comp;
      const auto phase=observed.find({svid,10+freq});
      const double lambda=freq==1?kLambda1:kLambda2,alpha=freq==1?1.0:kAlpha2;
      newx(idx)=phase!=observed.end()?(phase->second-geom+alpha*ion)/lambda:0.0;
      pnew(idx,idx)=config_.ambiguity_initial_sigma_cycles*config_.ambiguity_initial_sigma_cycles;
    }
  }
  // Cycle slips reset only the affected frequency ambiguity while retaining the ionosphere state.
  for(const auto& s:e.satellites) {
    auto it=slots_.find(s.svid); if(it==slots_.end())continue;
    for(int f=1;f<=2;++f) {
      const bool slipped=f==1?(s.rover_l1_cycle_slip||s.base_l1_cycle_slip):(s.rover_l2_cycle_slip||s.base_l2_cycle_slip);
      const Eigen::Index idx=f==1?it->second.l1:it->second.l2;
      if(slipped&&idx>=navN) {
        newx(idx)=0.0; pnew.row(idx).setZero();pnew.col(idx).setZero();
        pnew(idx,idx)=config_.ambiguity_initial_sigma_cycles*config_.ambiguity_initial_sigma_cycles;
      }
    }
  }
  for(Eigen::Index i=navN;i<newx.size();++i) if(pnew(i,i)<=0.0) pnew(i,i)=1e-9;
  pnew=0.5*(pnew+pnew.transpose());
  estimator_.initialize(newx,pnew);pivot_svid_=pivot;
}

nav::core::EstimatorUpdateResult GmisTcEstimatorSession::updateImu(const ImuSample& sample) {
  constexpr Eigen::Index navN=StateIndex::dimension;
  if(!initialized_||sample.time_seconds<time_seconds_) throw std::logic_error("GMIS TC IMU events must be ordered");
  const Eigen::Index n=estimator_.state().size();
  if(sample.time_seconds>time_seconds_) {
    const auto pred=GmisLcModels::predict(estimator_.state().head(navN),sample.time_seconds-time_seconds_,config_.common.process_noise);
    Eigen::VectorXd x=estimator_.state();x.head(navN)=pred.state;
    Eigen::MatrixXd f=Eigen::MatrixXd::Identity(n,n),q=Eigen::MatrixXd::Zero(n,n);
    f.topLeftCorner(navN,navN)=pred.jacobian;q.topLeftCorner(navN,navN)=pred.process_noise;
    const double dt=sample.time_seconds-time_seconds_;
    for(const auto& [_,slot]:slots_) { q(slot.ionosphere,slot.ionosphere)=std::pow(config_.ionosphere_rw_m_sqrt_s,2)*dt;
      q(slot.l1,slot.l1)=std::pow(config_.ambiguity_rw_cycles_sqrt_s,2)*dt;q(slot.l2,slot.l2)=std::pow(config_.ambiguity_rw_cycles_sqrt_s,2)*dt; }
    estimator_.predict(x,f,q);time_seconds_=sample.time_seconds;
  }
  const auto obs=config_.common.imu_model==GmisImuModelKind::EarthAwareInertialRate
      ?GmisLcModels::linearizeImuEarthAware(estimator_.state().head(navN),sample,config_.common.imu_measurement_covariance,config_.common.origin)
      :GmisLcModels::linearizeImu(estimator_.state().head(navN),sample,config_.common.imu_measurement_covariance);
  Eigen::MatrixXd h=Eigen::MatrixXd::Zero(obs.jacobian.rows(),n);h.leftCols(navN)=obs.jacobian;
  return estimator_.update(obs.innovation,h,obs.noise);
}

nav::core::EstimatorUpdateResult GmisTcEstimatorSession::updateGnss(const navcore::PreparedGnssEpoch& e) {
  constexpr Eigen::Index navN=StateIndex::dimension;
  if(!initialized_||e.gps_seconds<time_seconds_-0.02) throw std::logic_error("GMIS TC GNSS epoch is out of order");
  const Eigen::Vector3d rover=antennaEcef(estimator_.state().head(navN));
  const auto pivotValid=[&](const navcore::PreparedSatellite& s) {
    const bool phase=(valid(s.rover_l1_cycles)&&valid(s.base_l1_cycles)&&!s.rover_l1_cycle_slip&&!s.base_l1_cycle_slip)||
                     (valid(s.rover_l2_cycles)&&valid(s.base_l2_cycles)&&!s.rover_l2_cycle_slip&&!s.base_l2_cycle_slip);
    return valid(s.rover_c1_m)&&valid(s.base_c1_m)&&valid(s.rover_p2_m)&&valid(s.base_p2_m)&&
      phase&&elevation(s)>=config_.minimum_elevation_deg;
  };
  int pivot=0;
  if(pivot_svid_!=0) { const auto* old=findSatellite(e,pivot_svid_);if(old&&pivotValid(*old))pivot=pivot_svid_; }
  if(pivot==0) {
    double best=-1.0;
    for(const auto& s:e.satellites)if(pivotValid(s)&&elevation(s)>best){best=elevation(s);pivot=s.svid;}
  }
  last_rows_.clear();
  if(pivot==0)return {};
  synchronizeStates(e,pivot);
  const Eigen::VectorXd x=estimator_.state();const Eigen::Index n=x.size();const Eigen::Vector3d rover_now=antennaEcef(x.head(navN));
  auto raw=makeRawRows(e,pivot,rover_now);
  std::map<int,bool> usable_sats;for(const auto& row:raw)usable_sats[row.svid]=true;
  if(usable_sats.size()<config_.minimum_common_satellites)return {};
  const Eigen::Index m=static_cast<Eigen::Index>(raw.size());if(m==0)return {};
  Eigen::VectorXd innovation(m);Eigen::MatrixXd h=Eigen::MatrixXd::Zero(m,n),rmat=Eigen::MatrixXd::Zero(m,m);
  std::vector<double> predicted(static_cast<std::size_t>(m));
  for(Eigen::Index i=0;i<m;++i) {
    const auto& row=raw[static_cast<std::size_t>(i)];
    const auto& slot=slots_.at(row.svid);const double geom=geometricDoubleDifference(rover_now,e,row.svid,pivot);
    const double alpha=row.frequency==1?1.0:kAlpha2;
    const Eigen::Index ion=slot.ionosphere,amb=row.frequency==1?slot.l1:slot.l2;
    const double pred=row.type=="code"?geom+alpha*x(ion):geom-alpha*x(ion)+(row.frequency==1?kLambda1:kLambda2)*x(amb);
    predicted[static_cast<std::size_t>(i)]=pred;innovation(i)=row.observed-pred;
    for(int j=0;j<navN;++j) {
      const double step=(j>=StateIndex::attitude&&j<StateIndex::attitude+3)?1e-6:1e-3;
      Eigen::VectorXd xp=x.head(navN),xm=x.head(navN);xp(j)+=step;xm(j)-=step;
      const double gp=geometricDoubleDifference(antennaEcef(xp),e,row.svid,pivot);
      const double gm=geometricDoubleDifference(antennaEcef(xm),e,row.svid,pivot);
      h(i,j)=(gp-gm)/(2.0*step);
    }
    h(i,ion)=row.type=="code"?alpha:-alpha;
    if(row.type=="phase")h(i,amb)=row.frequency==1?kLambda1:kLambda2;
    last_rows_.push_back({row.type,row.svid,pivot,row.frequency,row.observed,pred,innovation(i)});
  }
  // Each observable/frequency block has a shared-pivot covariance term.
  for(Eigen::Index i=0;i<m;++i)for(Eigen::Index j=0;j<m;++j) {
    const auto& a=raw[static_cast<std::size_t>(i)];const auto& b=raw[static_cast<std::size_t>(j)];
    if(a.type!=b.type||a.frequency!=b.frequency)continue;
    const auto* sp=findSatellite(e,pivot);
    const double varA=2.0*a.sigma*a.sigma;
    const double pivotSigma=[&] {const double baseSigma=a.type=="code"?(a.frequency==1?config_.c1_sigma_m:config_.p2_sigma_m):(a.frequency==1?config_.l1_sigma_m:config_.l2_sigma_m);
      const double scale=1.0/std::sqrt(std::max(0.15,std::sin(std::min(elevation(*sp),elevation(*sp))*3.14159265358979323846/180.0)));return baseSigma*scale;}();
    const double shared=2.0*pivotSigma*pivotSigma;
    rmat(i,j)=shared+(i==j?varA:0.0);
  }
  nav::core::EstimatorUpdateOptions options;if(config_.nis_gate>0.0)options.nis_gate=config_.nis_gate;
  auto result=estimator_.update(innovation,h,rmat,options);
  return result;
}

void GmisTcEstimatorSession::writeAmbiguities(std::ostream& out,double time) const {
  const auto& x=estimator_.state();const auto& p=estimator_.covariance();
  for(const auto& [svid,slot]:slots_)out<<time<<','<<pivot_svid_<<','<<svid<<','<<x(slot.ionosphere)<<','<<x(slot.l1)<<','<<x(slot.l2)<<','
    <<std::sqrt(std::max(0.0,p(slot.ionosphere,slot.ionosphere)))<<','<<std::sqrt(std::max(0.0,p(slot.l1,slot.l1)))<<','<<std::sqrt(std::max(0.0,p(slot.l2,slot.l2)))<<'\n';
}

} // namespace nav::gmis::tc
