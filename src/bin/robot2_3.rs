use p9n_interface_2024::p9n_interface;

use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
};
use drobo_interfaces::msg::PointDrive;

#[allow(non_snake_case)]
pub mod DualsenseState {
    pub const SQUARE: usize = 0;
    pub const CIRCLE: usize = 1;
    pub const TRIANGLE: usize = 2;
    pub const CROSS: usize = 3;
    pub const L1: usize = 4;
    pub const L2: usize = 5;
    pub const R1: usize = 6;
    pub const R2: usize = 7;
    pub const D_PAD_UP: usize = 8;
    pub const D_PAD_DOWN: usize = 9;
    pub const D_PAD_LEFT: usize = 10;
    pub const D_PAD_RIGHT: usize = 11;
    pub const START: usize = 12;
    pub const SELECT: usize = 13;
    pub const PS: usize = 14;
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("p9n_robot1", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let robot2_3_publisher = node.create_publisher::<PointDrive>("/point_2_3", None)?;

    worker(
        selector,
        subscriber,
        robot2_3_publisher,
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    robot2_3_publisher: Publisher<PointDrive>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Logger::new("p9n_interface_2024");
    let mut dualsense_state: [bool; 15] = [false; 15];

    let mut robot2_3_msg = PointDrive::new().unwrap();

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_dpad_up() && !dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "up");
                dualsense_state[DualsenseState::D_PAD_UP] = true;
                robot2_3_msg.md2 = if !p9n.pressed_cross() {25} else {125};
                robot2_3_msg.md3 = if !p9n.pressed_cross() {0} else {3};
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "reverse up");
                dualsense_state[DualsenseState::D_PAD_UP] = false;
            }
            if p9n.pressed_dpad_left() && !dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = true;
                robot2_3_msg.md3 = 1;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_dpad_left() && dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "reverse left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = false;
            }
            if p9n.pressed_dpad_right() && !dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = true;
                robot2_3_msg.md3 = 2;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_dpad_right() && dualsense_state[DualsenseState::D_PAD_RIGHT] {
                pr_info!(logger, "reverse right");
                dualsense_state[DualsenseState::D_PAD_RIGHT] = false;
            }
            if p9n.pressed_l2() {
                pr_info!(logger, "l2");
                dualsense_state[DualsenseState::L2] = true;
                robot2_3_msg.md4 = if !p9n.pressed_cross() {127} else {-128};
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_l2() && dualsense_state[DualsenseState::L2] {
                pr_info!(logger, "reverse l2");
                dualsense_state[DualsenseState::L2] = false;
                robot2_3_msg.md4 = 0;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if p9n.pressed_r2() {
                pr_info!(logger, "r2");
                dualsense_state[DualsenseState::R2] = true;
                robot2_3_msg.md5 = if !p9n.pressed_cross() {127} else {-128};
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                pr_info!(logger, "reverse r2");
                dualsense_state[DualsenseState::R2] = false;
                robot2_3_msg.md5 = 0;
                let _ = robot2_3_publisher.send(&robot2_3_msg);
            }            
        }),
    );
    loop {
        selector.wait()?;
    }
}
