use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("minimal_subscriber")?;
    let worker = node.create_worker::<usize>(0);
    let node_clone_odom = node.clone();
    let _odom_subscription = worker.create_subscription::<nav_msgs::msg::Odometry, _>(
        "/odom",
        move |_odom: nav_msgs::msg::Odometry| log!(node_clone_odom.logger(), "got odom"),
    )?;
    let node_clone_scan = node.clone();
    let _lidar_subscription = worker.create_subscription::<sensor_msgs::msg::LaserScan, _>(
        "/scan",
        move |_lidar_scan: sensor_msgs::msg::LaserScan| {
            log!(node_clone_scan.logger(), "got lidar scan")
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
