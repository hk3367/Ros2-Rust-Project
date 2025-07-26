use anyhow::{Error, Result};
use nav_msgs::msg::Odometry;
use rclrs::*;
use sensor_msgs::msg::LaserScan;

struct LidarOdomNode {
    _odom_subscriber: WorkerSubscription<Odometry, usize>,
    _lidar_subscriber: WorkerSubscription<LaserScan, usize>,
    worker: Worker<usize>,
}

impl LidarOdomNode {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let lidar_odom_node = executor.create_node("lidar_odometry_node")?;
        let worker = lidar_odom_node.create_worker::<usize>(0);
        let node_clone_odom = lidar_odom_node.clone();
        let _odom_subscriber = worker
            .create_subscription::<Odometry, _>("/odom", move |_odom: Odometry| {
                log!(node_clone_odom.logger(), "got odom")
            })?;
        let node_clone_scan = lidar_odom_node.clone();
        let _lidar_subscriber = worker
            .create_subscription::<LaserScan, _>("/scan", move |_lidar_scan: LaserScan| {
                log!(node_clone_scan.logger(), "got lidar scan")
            })?;
        Ok(Self {
            _odom_subscriber,
            _lidar_subscriber,
            worker,
        })
    }
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let lidar_odom_node = LidarOdomNode::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
