namespace jet {
namespace visualization {
void setup_view();
void put_quadraframe(viewer::SimpleGeometry& geo,
                     const control::QuadraframeStatus& status,
                     const control::QuadraframeConfiguration& quad_cfg,
                     const control::VaneConfiguration& vane_cfg);
}  // namespace visualization
}  // namespace jet