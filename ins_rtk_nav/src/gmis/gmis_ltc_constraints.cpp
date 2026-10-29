#include "nav/gmis/gmis_ltc_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav::gmis::ltc {
namespace {
double number(const navcore::Json& json, const char* key, double fallback) {
  const auto* value=json.find(key);
  return value?value->asNumber():fallback;
}
}

DynamicStatusConfig DynamicStatusConfig::fromJson(const navcore::Json& j) {
  DynamicStatusConfig c;
  c.window_seconds=number(j,"window_seconds",c.window_seconds);
  c.minimum_dwell_seconds=number(j,"minimum_dwell_seconds",c.minimum_dwell_seconds);
  c.stationary_gyro_rms_radps=number(j,"stationary_gyro_rms_radps",c.stationary_gyro_rms_radps);
  c.stationary_force_norm_std_mps2=number(j,"stationary_force_norm_std_mps2",c.stationary_force_norm_std_mps2);
  c.stationary_speed_mps=number(j,"stationary_speed_mps",c.stationary_speed_mps);
  c.moving_gyro_rms_radps=number(j,"moving_gyro_rms_radps",c.moving_gyro_rms_radps);
  c.moving_force_norm_std_mps2=number(j,"moving_force_norm_std_mps2",c.moving_force_norm_std_mps2);
  c.moving_speed_mps=number(j,"moving_speed_mps",c.moving_speed_mps);
  c.high_dynamic_gyro_rms_radps=number(j,"high_dynamic_gyro_rms_radps",c.high_dynamic_gyro_rms_radps);
  c.high_dynamic_force_norm_std_mps2=number(j,"high_dynamic_force_norm_std_mps2",c.high_dynamic_force_norm_std_mps2);
  if(c.window_seconds<=0||c.minimum_dwell_seconds<0||c.stationary_gyro_rms_radps<=0||
     c.stationary_force_norm_std_mps2<=0||c.stationary_speed_mps<0||
     c.moving_gyro_rms_radps<=c.stationary_gyro_rms_radps||
     c.moving_force_norm_std_mps2<=c.stationary_force_norm_std_mps2||
     c.moving_speed_mps<=c.stationary_speed_mps||
     c.high_dynamic_gyro_rms_radps<=c.moving_gyro_rms_radps||
     c.high_dynamic_force_norm_std_mps2<=c.moving_force_norm_std_mps2)
    throw std::invalid_argument("invalid GMIS-LTC dynamic-status thresholds");
  return c;
}

ConstraintConfig ConstraintConfig::fromJson(const navcore::Json& j) {
  ConstraintConfig c;
  const auto& models=j.at("measurement_models").asArray();
  for(const auto& model:models) {
    const auto id=model.asString();
    c.use_zupt=c.use_zupt||id=="zupt_stationary_v1";
    c.use_nhc=c.use_nhc||id=="vehicle_nhc_v1";
    c.use_zpc=c.use_zpc||id=="zero_position_change_v1";
    c.use_zhc=c.use_zhc||id=="zero_heading_change_v1";
  }
  const auto& p=j.at("constraints");
  c.minimum_confidence=number(p,"minimum_confidence",c.minimum_confidence);
  c.minimum_update_interval_seconds=number(p,"minimum_update_interval_seconds",c.minimum_update_interval_seconds);
  c.zupt_sigma_mps=number(p,"zupt_sigma_mps",c.zupt_sigma_mps);
  c.nhc_sigma_lateral_mps=number(p,"nhc_sigma_lateral_mps",c.nhc_sigma_lateral_mps);
  c.nhc_sigma_vertical_mps=number(p,"nhc_sigma_vertical_mps",c.nhc_sigma_vertical_mps);
  c.zpc_sigma_m=number(p,"zero_position_change_sigma_m",c.zpc_sigma_m);
  c.zhc_sigma_heading_rad=number(p,"zero_heading_change_sigma_deg",c.zhc_sigma_heading_rad*180.0/3.14159265358979323846)*3.14159265358979323846/180.0;
  c.nis_gate=number(p,"nis_gate",c.nis_gate);
  if(c.minimum_confidence<0||c.minimum_confidence>1||c.minimum_update_interval_seconds<=0||
     c.zupt_sigma_mps<=0||c.nhc_sigma_lateral_mps<=0||c.nhc_sigma_vertical_mps<=0||
     c.zpc_sigma_m<=0||c.zhc_sigma_heading_rad<=0||c.nis_gate<0||
     ((c.use_zpc||c.use_zhc)&&!c.use_zupt))
    throw std::invalid_argument("invalid GMIS-LTC constraint configuration (ZPC/ZHC require ZUPT)");
  return c;
}

DynamicStatusModule::DynamicStatusModule(DynamicStatusConfig c):config_(std::move(c)) {}

void DynamicStatusModule::push(const ImuSample& s,const Eigen::Vector3d& gyro_bias,
                               const Eigen::Vector3d& accel_bias) {
  window_.push_back({s.time_seconds,(s.gyro_radps-gyro_bias).norm(),
                     (s.specific_force_mps2-accel_bias).norm()});
  while(window_.size()>2&&s.time_seconds-window_.front().time>config_.window_seconds)window_.pop_front();
}

DynamicStatus DynamicStatusModule::evaluate(double time,double predicted_speed,
                                              std::optional<double> rtk_speed) {
  DynamicStatus out;out.time_seconds=time;out.speed_mps=predicted_speed;
  if(rtk_speed&&std::isfinite(*rtk_speed)&&*rtk_speed>=0)out.speed_mps=*rtk_speed;
  if(window_.size()<5||time-window_.front().time<0.8*config_.window_seconds){out.state=state_;return out;}
  double gyro2=0,force_mean=0,force2=0;
  for(const auto& s:window_){gyro2+=s.gyro_norm*s.gyro_norm;force_mean+=s.force_norm;force2+=s.force_norm*s.force_norm;}
  const double n=static_cast<double>(window_.size());out.gyro_rms_radps=std::sqrt(gyro2/n);
  force_mean/=n;out.specific_force_norm_std_mps2=std::sqrt(std::max(0.0,force2/n-force_mean*force_mean));
  const bool stationary=out.gyro_rms_radps<config_.stationary_gyro_rms_radps&&
      out.specific_force_norm_std_mps2<config_.stationary_force_norm_std_mps2&&
      (!rtk_speed||out.speed_mps<config_.stationary_speed_mps);
  const bool moving=out.gyro_rms_radps>config_.moving_gyro_rms_radps||
      out.specific_force_norm_std_mps2>config_.moving_force_norm_std_mps2||
      (rtk_speed&&out.speed_mps>config_.moving_speed_mps);
  out.high_dynamic=out.gyro_rms_radps>config_.high_dynamic_gyro_rms_radps||
      out.specific_force_norm_std_mps2>config_.high_dynamic_force_norm_std_mps2;
  const std::string proposed=stationary?"stationary":(moving?"moving":"unknown");
  if(proposed==state_){candidate_.clear();candidate_since_=0;}
  else if(proposed!="unknown"){
    if(candidate_!=proposed){candidate_=proposed;candidate_since_=time;}
    if(time-candidate_since_>=config_.minimum_dwell_seconds){state_=proposed;candidate_.clear();}
  }else{candidate_.clear();candidate_since_=0;}
  out.state=state_;out.confidence=state_=="stationary"?0.95:(state_=="moving"?0.75:0.0);return out;
}

void DynamicStatusModule::reset(){window_.clear();state_="unknown";candidate_.clear();candidate_since_=0;}

ConstraintManagerModule::ConstraintManagerModule(ConstraintConfig c):config_(std::move(c)) {}

std::vector<std::string> ConstraintManagerModule::select(const DynamicStatus& s) {
  if(s.state=="unknown"||s.confidence<config_.minimum_confidence||
     s.time_seconds-last_published_seconds_<config_.minimum_update_interval_seconds)return {};
  std::vector<std::string> out;
  if(s.state=="stationary"&&config_.use_zupt){out.push_back("zupt");if(config_.use_zpc)out.push_back("zpc");if(config_.use_zhc)out.push_back("zhc");}
  if(s.state=="moving"&&!s.high_dynamic&&config_.use_nhc)out.push_back("nhc");
  if(!out.empty())last_published_seconds_=s.time_seconds;
  return out;
}

void ConstraintManagerModule::reset(){last_published_seconds_=-1.0;}

} // namespace nav::gmis::ltc
