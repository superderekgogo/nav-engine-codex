#include "nav/gmis/gmis_ltc_estimation.hpp"
#include "nav/gmis/gmis_lc_config.hpp"
#include "navcore/config_loader.hpp"
#include "navcore/data_decode.hpp"
#include "navcore/gnss_preprocessor.hpp"
#include "navcore/measurement_models.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double kPi=3.14159265358979323846, kA=6378137.0, kF=1.0/298.257223563;
struct Rtk { double time=0; std::string status; Eigen::Vector3d ecef=Eigen::Vector3d::Zero(); };
std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> v; std::stringstream ss(s); std::string x;
  while(std::getline(ss,x,',')) v.push_back(x); return v;
}
std::filesystem::path resolve(const std::filesystem::path& c,const std::string& s) {
  std::filesystem::path p(s); if(p.is_relative()) p=c.parent_path()/p; return std::filesystem::weakly_canonical(p);
}
Eigen::Vector3d vec3(const navcore::Json& o,const char* k) {
  const auto& a=o.at(k).asArray(); if(a.size()!=3) throw std::runtime_error(std::string("expected 3 elements: ")+k);
  return {a[0].asNumber(),a[1].asNumber(),a[2].asNumber()};
}
Eigen::Vector3d baseCoordinate(const std::filesystem::path& path) {
  std::ifstream in(path); Eigen::Vector3d v; if(!(in>>v.x()>>v.y()>>v.z())) throw std::runtime_error("cannot read base ECEF coordinates: "+path.string()); return v;
}
std::vector<Rtk> readRtk(const std::filesystem::path& path) {
  std::ifstream in(path); if(!in) throw std::runtime_error("cannot open RTK solution: "+path.string());
  std::string line; std::getline(in,line); std::vector<Rtk> rows;
  while(std::getline(in,line)) { const auto f=split(line); if(f.size()<17 || f[1]=="INVALID" || f[1]=="HOLD") continue;
    try { Rtk r; r.time=std::stod(f[0]); r.status=f[1]; r.ecef={std::stod(f[4]),std::stod(f[5]),std::stod(f[6])}; if(std::stoi(f[10])&7) rows.push_back(r); } catch(...) {} }
  return rows;
}
Eigen::Vector3d ned(const Eigen::Vector3d& ecef,const nav::gmis::GeodeticOrigin& o) {
  const double s=std::sin(o.latitude_rad),c=std::cos(o.latitude_rad),so=std::sin(o.longitude_rad),co=std::cos(o.longitude_rad),e2=kF*(2-kF),n=kA/std::sqrt(1-e2*s*s);
  const Eigen::Vector3d origin((n+o.height_m)*c*co,(n+o.height_m)*c*so,(n*(1-e2)+o.height_m)*s);
  Eigen::Matrix3d r; r<<-s*co,-s*so,c,-so,co,0,-c*co,-c*so,-s; return r*(ecef-origin);
}
Eigen::Vector3d lla(const Eigen::Vector3d& p,const nav::gmis::GeodeticOrigin& o) {
  const double e2=kF*(2-kF),s=std::sin(o.latitude_rad),d=std::sqrt(1-e2*s*s),rn=kA/d,rm=kA*(1-e2)/(d*d*d);
  return {(o.latitude_rad+p.x()/(rm+o.height_m))*180/kPi,
          (o.longitude_rad+p.y()/((rn+o.height_m)*std::cos(o.latitude_rad)))*180/kPi,o.height_m-p.z()};
}
Eigen::Matrix<double,21,21> covariance(const navcore::Json& r) {
  Eigen::Matrix<double,21,21> p=Eigen::Matrix<double,21,21>::Zero();
  const auto set=[&](int i,const char* key){p.diagonal().segment<3>(i)=vec3(r,key).array().square();};
  set(0,"initial_position_sigma_m");set(3,"initial_velocity_sigma_mps");set(6,"initial_acceleration_sigma_mps2");set(9,"initial_attitude_sigma_rad");set(12,"initial_angular_rate_sigma_radps");
  const double sig=r.at("initial_sensor_error_sigma").asNumber();p.diagonal().segment<6>(15).setConstant(sig*sig);return p;
}
}

int main(int argc,char** argv) try {
  if(argc!=2) { std::cerr<<"Usage: gmis_ltc_app <gmis_ltc.json>\n"; return 2; }
  const auto config_path=std::filesystem::absolute(argv[1]); navcore::ConfigLoader loader; const auto json=loader.load(config_path);
  const auto model=nav::gmis::GmisLcConfiguration::fromJson(json); const auto& runtime=json.at("runtime");
  const auto& paths=json.at("paths"); const auto& gnss=json.at("gnss");
  const auto imu_path=resolve(config_path,paths.at("imu").asString());
  const auto rover_path=resolve(config_path,paths.at("rover_gnss").asString());
  const auto base_path=resolve(config_path,paths.at("base_gnss").asString());
  const auto base_coord_path=resolve(config_path,paths.at("base_coordinates").asString());
  const auto rtk_path=resolve(config_path,runtime.at("rtk_solution_file").asString());
  const auto output_path=resolve(config_path,runtime.at("output_file").asString());
  const auto innovation_path=resolve(config_path,runtime.at("tdcp_innovations_file").asString());
  navcore::ImuTextDecoder imu_decoder; navcore::GnssTextDecoder gnss_decoder(gnss.at("missing_value").asNumber());
  const auto raw_imu=imu_decoder.decodeFile(imu_path);
  std::vector<nav::gmis::ImuSample> imu; imu.reserve(raw_imu.size());
  for(const auto& v:raw_imu) { nav::gmis::ImuSample s; s.time_seconds=v.gps_seconds;s.gyro_radps=v.gyro_radps;s.specific_force_mps2=v.specific_force_mps2;s.gyro_rate_convention=nav::gmis::ImuSample::GyroRateConvention::BodyRelativeInertial;imu.push_back(s); }
  navcore::GnssPreprocessor pairer; const auto paired=pairer.pair(gnss_decoder.decodeFile(rover_path),gnss_decoder.decodeFile(base_path),runtime.at("gnss_epoch_tolerance_seconds").asNumber());
  const auto& slip=json.at("cycle_slip_preprocessing"); navcore::GnssCycleSlipConfig sc;
  sc.maximum_epoch_gap_seconds=slip.at("maximum_epoch_gap_seconds").asNumber(); sc.doppler_phase_threshold_cycles=slip.at("doppler_phase_threshold_cycles").asNumber();
  sc.integer_cycle_tolerance_cycles=slip.at("integer_cycle_tolerance_cycles").asNumber();sc.geometry_free_threshold_m=slip.at("geometry_free_threshold_m").asNumber();
  sc.l1_wavelength_m=slip.at("l1_wavelength_m").asNumber();sc.l2_wavelength_m=slip.at("l2_wavelength_m").asNumber();sc.minimum_common_satellites=static_cast<std::size_t>(slip.at("minimum_common_satellites").asNumber());
  navcore::GnssCycleSlipPreprocessor slip_filter(sc); navcore::TdDdBuilderConfig tc;
  const auto& td_config=json.at("fusion_model").at("td_ddcp"); tc.max_epoch_gap_seconds=td_config.at("max_epoch_gap_seconds").asNumber();
  tc.minimum_common_satellites=static_cast<std::size_t>(td_config.at("minimum_common_satellites").asNumber());tc.l1_wavelength_m=td_config.at("wavelength_l1_m").asNumber();
  tc.l2_wavelength_m=td_config.at("wavelength_l2_m").asNumber();tc.phase_sigma_cycles=td_config.at("phase_sigma_cycles_per_receiver").asNumber();
  tc.doppler_phase_rate_sign=td_config.at("doppler_phase_rate_sign").asNumber();tc.cycle_slip_threshold_cycles=td_config.at("cycle_slip_threshold_cycles").asNumber();
  navcore::TdDdCarrierPhaseBuilder builder(tc); const auto base=baseCoordinate(base_coord_path); const auto rtk=readRtk(rtk_path);
  std::map<long long,Rtk> rtk_by_ms; for(const auto& r:rtk) rtk_by_ms[std::llround(r.time*1000.0)]=r;
  Eigen::VectorXd x=Eigen::VectorXd::Zero(nav::gmis::StateIndex::dimension);x.segment<3>(9)=vec3(runtime,"initial_attitude_rph_rad");
  nav::gmis::GmisLtcEstimatorSession session(model,runtime.at("clone_regularization").asNumber());session.initialize(imu.front().time_seconds,x,covariance(runtime));
  const double inflation=json.at("tdcp").at("variance_inflation").asNumber();
  std::filesystem::create_directories(output_path.parent_path());std::filesystem::create_directories(innovation_path.parent_path());
  std::ofstream out(output_path),diag(innovation_path);if(!out||!diag)throw std::runtime_error("cannot create GMIS-LTC outputs");
  out<<"gps_seconds,latitude_deg,longitude_deg,height_m,north_m,east_m,down_m,vn_mps,ve_mps,vd_mps,roll_rad,pitch_rad,heading_rad,rms_n_m,rms_e_m,rms_d_m,rms_vn_mps,rms_ve_mps,rms_vd_mps,rms_roll_rad,rms_pitch_rad,rms_heading_rad,imu_update_accepted,imu_nis,rtk_update_accepted,rtk_nis,rtk_status\n";
  diag<<"gps_seconds,measurement_type,svid,pivot_svid,innovation_m,normalized_innovation,nis_joint,accepted,tdcp_variance_inflation\n";
  std::size_t ii=0,ri=0,outputs=0; double last_imu_nis=0,last_gnss_nis=0;bool imu_ok=true;
  for(const auto& epoch:paired.epochs) {
    while(ii<imu.size()&&imu[ii].time_seconds<=epoch.gps_seconds+1e-9) { auto u=session.updateImu(imu[ii++]);imu_ok=u.accepted;last_imu_nis=u.nis; }
    const auto key=std::llround(epoch.gps_seconds*1000.0); auto found=rtk_by_ms.find(key);if(found==rtk_by_ms.end())continue;
    while(ri<rtk.size()&&rtk[ri].time<epoch.gps_seconds-0.01)++ri;
    auto clean=slip_filter.process(epoch);const auto td=builder.push(clean);
    const auto p_ned=ned(found->second.ecef,model.origin); Eigen::Matrix3d r=Eigen::Matrix3d::Zero();
    r.diagonal()=model.rtk_measurement_covariance.diagonal().head<3>();
    const auto update=session.updateEpoch(epoch.gps_seconds,p_ned,r,base,td?&*td:nullptr,inflation);last_gnss_nis=update.nis;
    if(td) { std::size_t l1_row=0; for(std::size_t j=0;j<td->observations.size();++j) if(td->observations[j].band==navcore::CarrierPhaseBand::L1) {
      const Eigen::Index idx=3+static_cast<Eigen::Index>(l1_row++); if(idx>=update.innovation.size())continue;
      const auto& o=td->observations[j];diag<<std::setprecision(12)<<epoch.gps_seconds<<",tdcp_l1,"<<o.svid<<','<<o.pivot_svid<<','<<update.innovation[idx]<<','<<update.normalized_innovation[idx]<<','<<update.nis<<",1,"<<inflation<<'\n';
    }}
    for(int k=0;k<3;++k)diag<<std::setprecision(12)<<epoch.gps_seconds<<",rtk_position,"<<(k+1)<<",0,"<<update.innovation[k]<<','<<update.normalized_innovation[k]<<','<<update.nis<<",1,"<<inflation<<'\n';
    const auto& s=session.state();const auto& p=session.covariance();const auto l=lla(s.tail(21).segment<3>(0),model.origin);
    out<<std::setprecision(12)<<epoch.gps_seconds<<','<<l.x()<<','<<l.y()<<','<<l.z();
    for(int i:{0,1,2,3,4,5,9,10,11})out<<','<<s(21+i);
    for(int i:{0,1,2,3,4,5,9,10,11})out<<','<<std::sqrt(std::max(0.0,p(21+i,21+i)));
    out<<','<<(imu_ok?1:0)<<','<<last_imu_nis<<",1,"<<last_gnss_nis<<','<<found->second.status<<'\n';++outputs;
    session.rollLagWindow();
  }
  std::cout<<"GMIS_LTC replay complete: "<<outputs<<" epochs; IMU="<<imu.size()<<" GNSS="<<paired.epochs.size()<<" RTK="<<rtk.size()<<"; output="<<output_path<<"; innovations="<<innovation_path<<"; TDCP R inflation="<<inflation<<" (report-only NIS)\n";
  return 0;
} catch(const std::exception& e) { std::cerr<<"GMIS_LTC failed: "<<e.what()<<'\n';return 1; }
