use anyhow::{Error, Result};
use geometry_msgs::msg::PoseWithCovariance;
use nav_msgs::msg::Odometry;
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::sync::{Arc, Mutex};
extern crate nalgebra as na;

struct LidarOdomNode {
    worker: Worker<usize>,
    odom: Mutex<Option<Odometry>>,
    prev_scan_odom: Mutex<Option<Odometry>>,
}

impl LidarOdomNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("lidar_odometry_node")?;
        let worker = node.create_worker::<usize>(0);
        let odom = Mutex::new(None);
        let prev_scan_odom = Mutex::new(None);
        Ok(Self {
            worker,
            odom,
            prev_scan_odom,
        })
    }

    fn odom_callback(&self, odom_msg: Odometry) -> () {
        *self.odom.lock().unwrap() = Some(odom_msg);
    }

    fn lidar_callback(&self, scan_msg: LaserScan) -> () {
        println!("Laser Scan: {:?}", scan_msg);
        println!("Odom in Scan Callback, {:?}", self.odom);

        let mut previous_scan_odom_guard = self.prev_scan_odom.lock().unwrap();
        match previous_scan_odom_guard.take() {
            None => {
                println!("prev odom is none initializing");
                *previous_scan_odom_guard = self.odom.lock().unwrap().clone();
            }
            Some(prev_scan_odom) => {
                let (tf_odom2map, prev_scan_cov) = pose_w_cov_to_nalgebra(&prev_scan_odom.pose);
                println!("Previous Scan Odom Isometry, {:?}", tf_odom2map);
                println!("Previous Scan Odom Covariance, {:?}", prev_scan_cov);
                *previous_scan_odom_guard = self.odom.lock().unwrap().clone();
            }
        }
    }
}

fn pose_w_cov_to_nalgebra(
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

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let lidar_odom_node = Arc::new(LidarOdomNode::new(&executor)?);

    let node_ref_odom = Arc::clone(&lidar_odom_node);
    let _odom_subscriber = lidar_odom_node.worker.create_subscription::<Odometry, _>(
        "/odom",
        move |odom_msg: Odometry| {
            node_ref_odom.odom_callback(odom_msg);
        },
    )?;

    let node_ref_scan = Arc::clone(&lidar_odom_node);
    let _lidar_subscriber = lidar_odom_node.worker.create_subscription::<LaserScan, _>(
        "scan",
        move |scan_msg: LaserScan| {
            node_ref_scan.lidar_callback(scan_msg);
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
