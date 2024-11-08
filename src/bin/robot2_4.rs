use p9n_interface_2024::p9n_interface;

use drobo_interfaces::msg::PointDrive;
use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::sensor_msgs,
    pr_info,
    selector::Selector,
    topic::{publisher::Publisher, subscriber::Subscriber},
};

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
    let node = ctx.create_node("p9n_robot2_4", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let robot2_2_publisher = node.create_publisher::<PointDrive>("/point_2_4", None)?;

    worker(selector, subscriber, robot2_2_publisher)?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    robot2_4_publisher: Publisher<PointDrive>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Logger::new("p9n_interface_2024");
    let mut dualsense_state: [bool; 15] = [false; 15];

    let mut robot2_4_msg = PointDrive::new().unwrap();

    let mut arm0_angle = 25;
    let mut arm1_angle = 25;
    let mut arm2_angle = 25;
    let mut arm3_angle = 25;

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());
            if p9n.pressed_dpad_up() && !dualsense_state[DualsenseState::D_PAD_UP] {
                dualsense_state[DualsenseState::D_PAD_UP] = true;
                robot2_4_msg.md0 = if !p9n.pressed_cross() { 125 } else { 80 };
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                dualsense_state[DualsenseState::D_PAD_UP] = false;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }

            if p9n.pressed_l2() && !dualsense_state[DualsenseState::L2] {
                dualsense_state[DualsenseState::L2] = true;
                robot2_4_msg.md3 = 125;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
            if !p9n.pressed_l2() && dualsense_state[DualsenseState::L2] {
                dualsense_state[DualsenseState::L2] = false;
            }
            if p9n.pressed_l1() && !dualsense_state[DualsenseState::L1] {
                dualsense_state[DualsenseState::L1] = true;
                robot2_4_msg.md3 = 25;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
            if !p9n.pressed_l1() && dualsense_state[DualsenseState::L1] {
                dualsense_state[DualsenseState::L1] = false;
            }
            if !dualsense_state[DualsenseState::L2] && !dualsense_state[DualsenseState::L1] {
                robot2_4_msg.md3 = 80;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }

            if p9n.pressed_r2() && !dualsense_state[DualsenseState::R2] {
                dualsense_state[DualsenseState::R2] = true;
                robot2_4_msg.md1 = 25;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
            if !p9n.pressed_r2() && dualsense_state[DualsenseState::R2] {
                dualsense_state[DualsenseState::R2] = false;
            }
            if p9n.pressed_r1() && !dualsense_state[DualsenseState::R1] {
                dualsense_state[DualsenseState::R1] = true;
                robot2_4_msg.md1 = 125;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
            if !p9n.pressed_r1() && dualsense_state[DualsenseState::R1] {
                dualsense_state[DualsenseState::R1] = false;
            }
            if !dualsense_state[DualsenseState::R2] && !dualsense_state[DualsenseState::R1] {
                robot2_4_msg.md1 = 80;
                let _ = robot2_4_publisher.send(&robot2_4_msg);
            }
        }),
    );
    loop {
        selector.wait()?;
    }
}
