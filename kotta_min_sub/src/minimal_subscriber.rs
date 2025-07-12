use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("minimal_subscriber")?;
    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_subscription::<nav_msgs::msg::Odometry, _>(
        "/odom",
        move |msg: nav_msgs::msg::Odometry| {
            log!(node.logger(), "Odom Position:{:?}", msg)
        },
    )?;

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
