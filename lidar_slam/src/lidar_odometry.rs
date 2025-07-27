use anyhow::{Error, Result};
use nav_msgs::msg::Odometry;
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::sync::Arc;

struct LidarOdomNode {
    worker: Worker<usize>,
}

impl LidarOdomNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("lidar_odometry_node")?;
        let worker = node.create_worker::<usize>(0);
        Ok(Self {  worker })
    }

    fn odom_callback(&self, odom_msg: Odometry) -> () {
        println!("Odometry: {:?}", odom_msg);
    }

    fn lidar_callback(&self, scan_msg: LaserScan) -> () {
        println!("Laser Scan: {:?}", scan_msg);
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
