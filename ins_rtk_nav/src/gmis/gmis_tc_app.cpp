#include "nav/gmis/gmis_tc_estimation.hpp"
#include "navcore/config_loader.hpp"
#include "navcore/data_decode.hpp"
#include "navcore/gnss_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double kPi=3.14159265358979323846,kA=6378137.0,kF=1.0/298.257223563;
Eigen::Vector3d vec3(const navcore::Json& j,const char* k){const auto&a=j.at(k).asArray();if(a.size()!=3)throw std::runtime_error(std::string("expected vector ")+k);return{a[0].asNumber(),a[1].asNumber(),a[2].asNumber()};}
std::filesystem::path resolve(const std::filesystem::path& c,const std::string&s){std::filesystem::path p(s);if(p.is_relative())p=c.parent_path()/p;return std::filesystem::weakly_canonical(p);}
Eigen::Vector3d readEcef(const std::filesystem::path& p){std::ifstream f(p);Eigen::Vector3d x;if(!(f>>x.x()>>x.y()>>x.z()))throw std::runtime_error("cannot read base ECEF coordinate: "+p.string());return x;}
Eigen::Matrix<double,21,21> initialCovariance(const navcore::Json&r){
  Eigen::Matrix<double,21,21>p=Eigen::Matrix<double,21,21>::Zero();
  const auto set=[&](int i,const char*k){p.diagonal().segment<3>(i)=vec3(r,k).array().square();};
  set(0,"initial_position_sigma_m");set(3,"initial_velocity_sigma_mps");set(6,"initial_acceleration_sigma_mps2");
  set(9,"initial_attitude_sigma_rad");set(12,"initial_angular_rate_sigma_radps");
  const double s=r.at("initial_sensor_error_sigma").asNumber();p.diagonal().segment<6>(15).setConstant(s*s);return p;
}
Eigen::Vector3d lla(const Eigen::Vector3d&p,const nav::gmis::GeodeticOrigin&o){
  const double e2=kF*(2-kF),s=std::sin(o.latitude_rad),den=std::sqrt(1-e2*s*s),rn=kA/den,rm=kA*(1-e2)/(den*den*den);
  return{(o.latitude_rad+p.x()/(rm+o.height_m))*180/kPi,(o.longitude_rad+p.y()/((rn+o.height_m)*std::cos(o.latitude_rad)))*180/kPi,o.height_m-p.z()};
}
}

int main(int argc,char**argv) try {
  if(argc!=2){std::cerr<<"Usage: gmis_tc_app <gmis_tc.json>\n";return 2;}
  const auto configPath=std::filesystem::absolute(argv[1]);navcore::ConfigLoader loader;const auto json=loader.load(configPath);
  const auto config=nav::gmis::tc::Configuration::fromJson(json);const auto&runtime=json.at("runtime");const auto&paths=json.at("paths");
  const auto imuPath=resolve(configPath,paths.at("imu").asString()),roverPath=resolve(configPath,paths.at("rover_gnss").asString());
  const auto basePath=resolve(configPath,paths.at("base_gnss").asString()),baseCoordPath=resolve(configPath,paths.at("base_coordinates").asString());
  const auto outputPath=resolve(configPath,runtime.at("output_file").asString()),innovPath=resolve(configPath,runtime.at("innovations_file").asString());
  const auto ambPath=resolve(configPath,runtime.at("ambiguity_file").asString());const auto baseEcef=readEcef(baseCoordPath);
  navcore::ImuTextDecoder imuDecoder;navcore::GnssTextDecoder gnssDecoder(json.at("gnss").at("missing_value").asNumber());
  const auto rawImu=imuDecoder.decodeFile(imuPath);std::vector<nav::gmis::ImuSample>imu;imu.reserve(rawImu.size());
  for(const auto&r:rawImu){nav::gmis::ImuSample s;s.time_seconds=r.gps_seconds;s.gyro_radps=r.gyro_radps;s.specific_force_mps2=r.specific_force_mps2;s.gyro_rate_convention=nav::gmis::ImuSample::GyroRateConvention::BodyRelativeInertial;imu.push_back(s);}
  navcore::GnssPreprocessor pairer;const auto paired=pairer.pair(gnssDecoder.decodeFile(roverPath),gnssDecoder.decodeFile(basePath),json.at("gnss").at("epoch_tolerance_seconds").asNumber());
  const auto&slip=json.at("cycle_slip_preprocessing");navcore::GnssCycleSlipConfig sc;
  sc.maximum_epoch_gap_seconds=slip.at("maximum_epoch_gap_seconds").asNumber();sc.doppler_phase_threshold_cycles=slip.at("doppler_phase_threshold_cycles").asNumber();
  sc.integer_cycle_tolerance_cycles=slip.at("integer_cycle_tolerance_cycles").asNumber();sc.geometry_free_threshold_m=slip.at("geometry_free_threshold_m").asNumber();
  sc.l1_wavelength_m=slip.at("l1_wavelength_m").asNumber();sc.l2_wavelength_m=slip.at("l2_wavelength_m").asNumber();
  sc.minimum_common_satellites=static_cast<std::size_t>(slip.at("minimum_common_satellites").asNumber());navcore::GnssCycleSlipPreprocessor slipFilter(sc);
  Eigen::VectorXd x=Eigen::VectorXd::Zero(nav::gmis::StateIndex::dimension);x.segment<3>(nav::gmis::StateIndex::attitude)=vec3(runtime,"initial_attitude_rph_rad");
  nav::gmis::tc::GmisTcEstimatorSession session(config);session.initialize(imu.front().time_seconds,x,initialCovariance(runtime));
  std::filesystem::create_directories(outputPath.parent_path());std::filesystem::create_directories(innovPath.parent_path());std::filesystem::create_directories(ambPath.parent_path());
  std::ofstream out(outputPath),innov(innovPath),amb(ambPath);if(!out||!innov||!amb)throw std::runtime_error("cannot create GMIS TC output files");
  out<<"gps_seconds,latitude_deg,longitude_deg,height_m,north_m,east_m,down_m,vn_mps,ve_mps,vd_mps,roll_rad,pitch_rad,heading_rad,rms_n_m,rms_e_m,rms_d_m,rms_vn_mps,rms_ve_mps,rms_vd_mps,rms_roll_rad,rms_pitch_rad,rms_heading_rad,imu_update_accepted,imu_nis,gnss_update_accepted,gnss_nis,gnss_rows,pivot_svid,ambiguity_satellites,ambiguity_status\n";
  innov<<"gps_seconds,measurement_type,svid,pivot_svid,frequency,observed_m,predicted_m,innovation_m,normalized_innovation,nis,rows,accepted,solution_status\n";
  amb<<"gps_seconds,pivot_svid,svid,ionosphere_dd_m,ambiguity_l1_cycles,ambiguity_l2_cycles,sigma_iono_m,sigma_ambiguity_l1_cycles,sigma_ambiguity_l2_cycles\n";
  std::size_t ii=0,outputs=0;double lastImuNis=0.0;bool imuAccepted=true;
  for(auto epoch:paired.epochs){
    while(ii<imu.size()&&imu[ii].time_seconds<=epoch.gps_seconds+1e-9){const auto u=session.updateImu(imu[ii++]);imuAccepted=u.accepted;lastImuNis=u.nis;}
    epoch.base_station_position_ecef_m=baseEcef;const auto clean=slipFilter.process(epoch);const auto u=session.updateGnss(clean);
    const auto&rows=session.lastRows();const bool gnssAccepted=!rows.empty()&&u.accepted;const auto&state=session.state();const auto&p=session.covariance();
    if(!rows.empty())for(std::size_t j=0;j<rows.size();++j){const auto&r=rows[j];const Eigen::Index idx=static_cast<Eigen::Index>(j);
      const double norm=idx<u.normalized_innovation.size()?u.normalized_innovation(idx):0.0;
      innov<<std::setprecision(12)<<epoch.gps_seconds<<','<<r.type<<','<<r.svid<<','<<r.pivot_svid<<','<<r.frequency<<','<<r.observed_m<<','<<r.predicted_m<<','<<r.innovation_m<<','<<norm<<','<<u.nis<<','<<rows.size()<<','<<(u.accepted?1:0)<<",FLOAT\n";}
    session.writeAmbiguities(amb,epoch.gps_seconds);
    const auto ll=lla(state.segment<3>(0),config.common.origin);out<<std::setprecision(12)<<epoch.gps_seconds<<','<<ll.x()<<','<<ll.y()<<','<<ll.z();
    for(int i:{0,1,2,3,4,5,9,10,11})out<<','<<state(i);
    for(int i:{0,1,2,3,4,5,9,10,11})out<<','<<std::sqrt(std::max(0.0,p(i,i)));
    out<<','<<(imuAccepted?1:0)<<','<<lastImuNis<<','<<(gnssAccepted?1:0)<<','<<u.nis<<','<<rows.size()<<','<<session.pivotSvid()<<','<<session.ambiguitySatelliteCount()<<",FLOAT\n";++outputs;
  }
  std::cout<<"GMIS_TC float replay complete: "<<outputs<<" epochs; IMU="<<imu.size()<<" GNSS="<<paired.epochs.size()<<"; output="<<outputPath<<"; innovations="<<innovPath<<"; ambiguities="<<ambPath<<"\n";
  return 0;
}catch(const std::exception&e){std::cerr<<"GMIS_TC failed: "<<e.what()<<'\n';return 1;}
