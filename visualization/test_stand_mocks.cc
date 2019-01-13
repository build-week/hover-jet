
namespace jet {
namespace control {
namespace detail {
SE3 compute_T_vane_unit_from_aerodynamic_frame() {
  const jcc::Vec3 vane_center_of_pressure_vane_unit_frame = jcc::Vec3(0.0, -0.030, 0.0);

  const SO3 R_intermediate_from_aerodynamic_frame =
      SO3::exp(jcc::Vec3(0.0, M_PI * 0.5, 0.0));
  const SO3 R_vane_unit_from_intermediate = SO3::exp(jcc::Vec3(0.0, 0.0, -M_PI * 0.5));
  const SO3 R_vane_unit_from_aerodynamic_frame =
      R_vane_unit_from_intermediate * R_intermediate_from_aerodynamic_frame;

  const SE3 T_vane_unit_from_zero =
      SE3(R_vane_unit_from_aerodynamic_frame, vane_center_of_pressure_vane_unit_frame);
  return T_vane_unit_from_zero;
}
}  // namespace detail

}  // namespace control
}  // namespace jet