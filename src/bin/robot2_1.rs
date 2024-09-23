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

fn calc_degree_to_pulsewidth(degree: u16) -> u8 {
    (((5 * degree) / 9) + 25) as u8
}

fn main() -> Result<(), DynError> {
    let ctx = Context::new()?;
    let node = ctx.create_node("p9n_robot2_1", None, Default::default())?;

    let selector = ctx.create_selector()?;
    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("joy", None)?;

    let robot2_1_publisher = node.create_publisher::<PointDrive>("/point_2_1", None)?;

    worker(
        selector,
        subscriber,
        robot2_1_publisher,
    )?;
    Ok(())
}

fn worker(
    mut selector: Selector,
    subscriber: Subscriber<sensor_msgs::msg::Joy>,
    robot2_1_publisher: Publisher<PointDrive>,
) -> Result<(), DynError> {
    let mut p9n = p9n_interface::PlaystationInterface::new(sensor_msgs::msg::Joy::new().unwrap());
    let logger = Logger::new("p9n_interface_2024");
    let mut dualsense_state: [bool; 15] = [false; 15];

    let mut robot2_1_msg = PointDrive::new().unwrap();

    selector.add_subscriber(
        subscriber,
        Box::new(move |_msg| {
            p9n.set_joy_msg(_msg.get_owned().unwrap());

            if p9n.pressed_dpad_left() && !dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = true;
                robot2_1_msg.md2 = 1;
                robot2_1_msg.md3 = calc_degree_to_pulsewidth(45) as i8;
                let _ = robot2_1_publisher.send(&robot2_1_msg);
            } 
            if !p9n.pressed_dpad_left() && dualsense_state[DualsenseState::D_PAD_LEFT] {
                pr_info!(logger, "reverse left");
                dualsense_state[DualsenseState::D_PAD_LEFT] = false;
                robot2_1_msg.md2 = 1;
                robot2_1_msg.md3 = calc_degree_to_pulsewidth(135) as i8;
                let _ = robot2_1_publisher.send(&robot2_1_msg);
            }
            if p9n.pressed_dpad_up() {
                pr_info!(logger, "up");
                dualsense_state[DualsenseState::D_PAD_UP] = true;
                robot2_1_msg.md2 = 0;
                robot2_1_msg.md4 = if !p9n.pressed_cross() {1} else {-1};
                let _ = robot2_1_publisher.send(&robot2_1_msg);
            }
            if !p9n.pressed_dpad_up() && dualsense_state[DualsenseState::D_PAD_UP] {
                pr_info!(logger, "reverse up");
                dualsense_state[DualsenseState::D_PAD_UP] = false;
                robot2_1_msg.md2 = 0;
                robot2_1_msg.md4 = 0;
                let _ = robot2_1_publisher.send(&robot2_1_msg);
            }
            if p9n.pressed_triangle() && !dualsense_state[DualsenseState::TRIANGLE] {
                pr_info!(logger, "triangle");
                dualsense_state[DualsenseState::TRIANGLE] = true;
                robot2_1_msg.md5 = if robot2_1_msg.md5 <= 0 {127} else {0};
                let _ = robot2_1_publisher.send(&robot2_1_msg);
            }
            if !p9n.pressed_triangle() && dualsense_state[DualsenseState::TRIANGLE] {
                pr_info!(logger, "reverse triangle");
                dualsense_state[DualsenseState::TRIANGLE] = false;
            }    
        }),
    );
    loop {
        selector.wait()?;
    }
}
