extern crate nalgebra as na;
use geometry_msgs::msg::PoseWithCovariance;

pub fn pose_w_cov_to_nalgebra(
    pose_with_cov: &PoseWithCovariance,
) -> (na::Isometry3<f64>, na::Matrix6<f64>) {
    let position = &pose_with_cov.pose.position;
    let covariance = &pose_with_cov.covariance;
    let orientation = &pose_with_cov.pose.orientation;
    let translation = na::Translation3::new(position.x, position.y, position.z);
    let rotation = na::Quaternion::new(orientation.w, orientation.x, orientation.y, orientation.z);
    let unit_quat = na::Unit::from_quaternion(rotation);
    let transform = na::Isometry3::from_parts(translation, unit_quat);
    let cov_mat = na::Matrix6::from_row_slice(covariance);
    (transform, cov_mat)
}

#[cfg(test)]
mod tests {
    use super::*;
}
