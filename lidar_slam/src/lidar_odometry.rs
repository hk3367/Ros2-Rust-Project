use anyhow::{Error, Result};
use lidarslam_lib::pose_w_cov_to_nalgebra;
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
        let mut previous_scan_odom_guard = self.prev_scan_odom.lock().unwrap();
        let mut tf_robot2prev_odom: na::Isometry3<f64> = Default::default();
        let mut prev_scan_odom_cov: na::Matrix6<f64> = Default::default();
        match previous_scan_odom_guard.take() {
            None => {
                println!("prev odom is none initializing");
                *previous_scan_odom_guard = self.odom.lock().unwrap().clone();
            }
            Some(prev_scan_odom) => {
                (tf_robot2prev_odom, prev_scan_odom_cov) =
                    pose_w_cov_to_nalgebra(&prev_scan_odom.pose);
                *previous_scan_odom_guard = self.odom.lock().unwrap().clone();
            }
        }

        let odom_ref_guard = self.odom.lock().unwrap();
        let mut tf_robot2odom: na::Isometry3<f64> = Default::default();
        let mut odom_cov: na::Matrix6<f64> = Default::default();
        match odom_ref_guard.as_ref() {
            None => (),
            Some(odom_ref_guard) => {
                (tf_robot2odom, odom_cov) = pose_w_cov_to_nalgebra(&odom_ref_guard.pose);
            }
        }
        println!("Previous Scan Odom Isometry, {:?}", tf_robot2prev_odom);
        println!("Previous Scan Odom Covariance, {:?}", prev_scan_odom_cov);
        println!("Laser Scan: {:?}", scan_msg);
        println!("Odom in Scan Callback, {:?}", tf_robot2odom);
        println!("Odom Covariance, {:?}", odom_cov);
    }
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
